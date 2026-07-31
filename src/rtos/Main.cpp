//
// Created by Akshay Gillett on 7/9/26.
//

#include <atomic>

#include "Logger.h"
#include "rtos.h"
#include "stm32f7xx.h"
#include "../../User_Tasks/user_tasks.h"
#include "../flight/flight_control.h"
#include "../flight/BufferPopulation.h"
#include <new>
#include <cstdlib>


volatile uint32_t globalSystemTicks = 0;
volatile bool rtosStarted = false;


TaskControlBlock taskControlBlocks[MAX_TASKS];
int activeTasks = 0;
static int currTaskIndex = 0;
int currLoopIndex;

static int preemptStack[MAX_TASKS];
int preemptDepth = 0;

//sim helpers
#ifdef SIMULATION

    static uint32_t seed = 98765;
    static uint32_t getRandom(int range) { // Used to find a random number
        seed = (1103515245 * seed + 12345) % 2147483648;
        return (seed % range) + 1;
    }

static void executeRandomInterrupts() {
        if (getRandom(100) == 50) { decideNextInterruptTask(&taskControlBlocks[1]); }
        if (getRandom(100) == 50) { decideNextInterruptTask(&taskControlBlocks[2]); }
    }

#endif

//actual rtos logic
static bool initializeNewTask(void (*functionAddress)(), uint32_t timePeriod, const char* textName, uint32_t stackSizeWords = 256) { //set the default to scheduled
    if (activeTasks >= MAX_TASKS) { return false; }

    //dynamic memory management:
    // auto* newStack = new (std::nothrow) uint32_t[stackSizeWords + 1];
        constexpr uint32_t GUARD_REGION_SIZE = 32; //defines the size of the mpu region
        const uint32_t totalBytes = (stackSizeWords * 4) + GUARD_REGION_SIZE;
        auto* newStack = static_cast<uint32_t*>(aligned_alloc(GUARD_REGION_SIZE, totalBytes));

    if (newStack == nullptr) {
        drone.currentSystemState = FlightState::ERROR;
        setEventMessage(drone.errorMSG, ERROR::INIT_HEAP_MEM);
        return false;
    }

    //setup stack
    taskControlBlocks[activeTasks].taskStack = newStack;
    taskControlBlocks[activeTasks].stackSizeWords = stackSizeWords;


    //normal init
    taskControlBlocks[activeTasks].taskCodeAddress = functionAddress;
    taskControlBlocks[activeTasks].executionPeriod = timePeriod;
    taskControlBlocks[activeTasks].taskName = textName;
    taskControlBlocks[activeTasks].taskState = TaskState::BLOCKED;
    taskControlBlocks[activeTasks].taskType = TaskType::SCHEDULED;

        const uint32_t totalWords = stackSizeWords + (GUARD_REGION_SIZE / sizeof(uint32_t));
        for (uint32_t i = 8; i < totalWords; i++) {
            newStack[i] = 0xA5A5A5A5;
        }

        //check floating point registers
        //point to the very ends of the 1024-word array idx = 1023
        taskControlBlocks[activeTasks].topOfStack = &newStack[stackSizeWords - 1];
        taskControlBlocks[activeTasks].topOfStack -= 32;

        //clear the slots for R0-R12 and LR (slots 0 to 13)
        for (int i = 0; i < 32; i++) { taskControlBlocks[activeTasks].topOfStack[i] = 0; }

    //set the PC slot (14) to the function address and xPSR slot (15) to Thumb mode
    taskControlBlocks[activeTasks].topOfStack[29] = 0xFFFFFFFD; //manally set LR (slot 45) to a dedicated address rather than undef/auto assigned
    taskControlBlocks[activeTasks].topOfStack[30] = reinterpret_cast<uint32_t>(functionAddress);
    taskControlBlocks[activeTasks].topOfStack[31] = 0x01000000;

    //initialize time stamp with custom function
    taskControlBlocks[activeTasks].lastRunTime = globalSystemTicks;

    activeTasks++;
    return true;
}

//overload the parameters for an interrupt because it doesn't have an execution period
static bool initializeNewTask(void (*functionAddress)(), const char* textName, uint32_t stackSizeWords = 256) { //set the default to scheduled
    if (activeTasks >= MAX_TASKS) { return false; }

    // auto* newStack = new (std::nothrow) uint32_t[stackSizeWords + 1];
        constexpr uint32_t GUARD_REGION_SIZE = 32; //defines the size of the mpu region
        const uint32_t totalBytes = (stackSizeWords * 4) + GUARD_REGION_SIZE;
        auto* newStack = static_cast<uint32_t*>(aligned_alloc(GUARD_REGION_SIZE, totalBytes));

    if (newStack == nullptr) {
        drone.currentSystemState = FlightState::ERROR;
        setEventMessage(drone.errorMSG, ERROR::INIT_HEAP_MEM);
        return false;
    }

    taskControlBlocks[activeTasks].taskStack = newStack;
    taskControlBlocks[activeTasks].stackSizeWords = stackSizeWords;


    taskControlBlocks[activeTasks].taskCodeAddress = functionAddress;
    taskControlBlocks[activeTasks].taskName = textName;
    taskControlBlocks[activeTasks].taskState = TaskState::BLOCKED;
    taskControlBlocks[activeTasks].taskType = TaskType::INTERRUPT;

        const uint32_t totalWords = stackSizeWords + (GUARD_REGION_SIZE / sizeof(uint32_t));
        for (uint32_t i = 8; i < totalWords; i++) {
            newStack[i] = 0xA5A5A5A5;
        }

        taskControlBlocks[activeTasks].topOfStack = &newStack[stackSizeWords - 1];
        taskControlBlocks[activeTasks].topOfStack -= 32;

        for (int i = 0; i < 32; i++) { taskControlBlocks[activeTasks].topOfStack[i] = 0; } //reset all to 0

    taskControlBlocks[activeTasks].topOfStack[29] = 0xFFFFFFFD;
    taskControlBlocks[activeTasks].topOfStack[30] = reinterpret_cast<uint32_t>(functionAddress);
    taskControlBlocks[activeTasks].topOfStack[31] = 0x01000000;

    taskControlBlocks[activeTasks].lastRunTime = globalSystemTicks;

    activeTasks++;
    return true;
}

static int nextMpuRegion = 0; //tracks how many regions are used (MAX 8)
bool protectStackGuard(const uint32_t guardBaseAddr) {
        if (nextMpuRegion >= 8) {
            return false; //out of MPU regions
        }

        MPU->RNR = nextMpuRegion;
        MPU->RBAR = guardBaseAddr & MPU_RBAR_ADDR_Msk; //sets start mem address for the region
        MPU->RASR = (0x1 << MPU_RASR_ENABLE_Pos) //enable this region
                  | (4  << MPU_RASR_SIZE_Pos) //size field: 4 = 32 bytes (2^(4+1))
                  | (0x0 << MPU_RASR_AP_Pos) //AP=0b000 = no access at all, any privilege
                  | (1   << MPU_RASR_XN_Pos); //no execute either

        nextMpuRegion++;
        return true;
}

uint32_t** currTaskAddress = nullptr;
uint32_t** nextTaskAddress = nullptr;

void yieldCurrentTask() {
    if (!rtosStarted) {
        return;
    }

    __disable_irq();

    taskControlBlocks[currTaskIndex].taskState = TaskState::BLOCKED; //block the current interrupt and go back to the scheduler process

    //dynamically switch to the last task
    int nextTaskIdx = (preemptDepth > 0) ? preemptStack[--preemptDepth] : 0; //pops off the top item in the stack

    currTaskAddress = &(taskControlBlocks[currTaskIndex].topOfStack);
    nextTaskAddress = &(taskControlBlocks[nextTaskIdx].topOfStack);

    currTaskIndex = nextTaskIdx;
    taskControlBlocks[currTaskIndex].taskState = TaskState::RUNNING;

    //fire the context switch back to the scheduled loop using pendSV
    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;

    __enable_irq();
    __ISB();
}


//universal helpers to stop pendSV getting overwritten
bool switchPending = false;
static int pendingWinnerIndex = -1;

void decideNextInterruptTask(TaskControlBlock* interruptTask) {
    __disable_irq(); //guards against pointers being overrwritten by simultaneous interrupts

    int comparisonIndex = switchPending ? pendingWinnerIndex : currTaskIndex;

    if (taskControlBlocks[comparisonIndex].taskType == TaskType::INTERRUPT &&
    taskControlBlocks[comparisonIndex].taskState == TaskState::RUNNING) {
        if (interruptTask->priority <= taskControlBlocks[comparisonIndex].priority) {
            __enable_irq();
            return;
        }
    }

    if (!switchPending) {
        switchPending = true;

        //append to the LIFO stack to have a preemt queue if multiple interrupts activate at once
        preemptStack[preemptDepth++] = currTaskIndex;

        //an interrupt is already pending, but previous is not saved
        //make the last task READY so it can run next tick (or after curr task is saved)
        taskControlBlocks[currTaskIndex].taskState = TaskState::READY;
        currTaskAddress = &(taskControlBlocks[currTaskIndex].topOfStack);
    } else {
        if (preemptDepth >= MAX_TASKS) {
            drone.currentSystemState = FlightState::ERROR;
            setEventMessage(drone.errorMSG, ERROR::PREEMPT_OVERFLOW);
        } else {
            preemptStack[preemptDepth++] = pendingWinnerIndex;
            taskControlBlocks[pendingWinnerIndex].taskState = TaskState::READY;
        }
    }

    //update addresses to point to next and current tasks
    nextTaskAddress = &(interruptTask->topOfStack);

    //update current task index to the interrupt task
    for (int i = 0; i < activeTasks; i++) {
        if (&taskControlBlocks[i] == interruptTask) {
            currTaskIndex = i;
            break;
        }
    }

    //set the next task to running
    interruptTask->taskState = TaskState::RUNNING;
    pendingWinnerIndex = currTaskIndex;

    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk; //trigger pendsv to switch

    __enable_irq();
}

void decideNextScheduledTask() {
    __disable_irq();

    int highestPriorityTaskIndex = -1;
    int highestPriorityValue = -1;

    for (int i = 0; i < activeTasks; i++) {
        if (taskControlBlocks[i].taskType == TaskType::SCHEDULED &&
            taskControlBlocks[i].taskState == TaskState::READY) {

            if (static_cast<int>(taskControlBlocks[i].priority) > highestPriorityValue) {
                highestPriorityValue = taskControlBlocks[i].priority;
                highestPriorityTaskIndex = i;
            }
        }
    }

    if (highestPriorityTaskIndex != -1) {

        int oldTaskIndex = currTaskIndex;

        //force previous periodic tasks back into BLOCKED state until their timer expires
        if (oldTaskIndex != 0 && taskControlBlocks[oldTaskIndex].taskType == TaskType::SCHEDULED) {
            taskControlBlocks[oldTaskIndex].taskState = TaskState::BLOCKED;
        } else if (oldTaskIndex == 0) {
            taskControlBlocks[oldTaskIndex].taskState = TaskState::READY;
        }

        const int nextTaskIdx = highestPriorityTaskIndex;

        currTaskAddress = &(taskControlBlocks[currTaskIndex].topOfStack);
        nextTaskAddress = &(taskControlBlocks[nextTaskIdx].topOfStack);

        // taskControlBlocks[currTaskIndex].taskState = TaskState::READY; //set it to running so it restores where it left off
        currTaskIndex = nextTaskIdx;
        taskControlBlocks[currTaskIndex].taskState = TaskState::RUNNING;
        taskControlBlocks[currTaskIndex].lastRunTime = globalSystemTicks;

        SCB->ICSR = SCB_ICSR_PENDSVSET_Msk; //trigger pendsv

        __enable_irq(); //disable lock
        __ISB(); //forces instructions from memory instantly - refresh
    }

    __enable_irq();
}

extern "C" void SysTick_Handler() {
    globalSystemTicks++;
    switchPending = false;

    printToUSART("============------------ NEW TICK: ");
    printToUSART(globalSystemTicks);
    printToUSART(" ------------============\n");

    for (int i = 0; i < activeTasks; i++) {
        currLoopIndex = i; //DO NOT POLL EVERY TICK - throws errors and crashes

        if (taskControlBlocks[i].taskType == TaskType::INTERRUPT) { continue; } //use instead of a 0 ms execution period

        if (taskControlBlocks[i].taskState == TaskState::BLOCKED) {
            if (globalSystemTicks - taskControlBlocks[i].lastRunTime >= taskControlBlocks[i].executionPeriod) {
                taskControlBlocks[i].taskState = TaskState::READY;
            }
        }
    }

    currLoopIndex = 0;

    #ifdef USER_TASKS
        callUserInterruptTasks();
    #endif

    #ifdef SIMULATION
        executeRandomInterrupts();
    #endif
}

static void printHex32(const uint32_t val) {
    char buf[11];
    snprintf(buf, sizeof(buf), "0x%08X", (unsigned int)val);
    printToUSART(buf);
}

extern "C" void MemManage_Handler(void) {
    __disable_irq();

    const uint32_t faultAddr = SCB->MMFAR;
    uint32_t stackedPC = 0;

    uint32_t* sp = (__get_CONTROL() & 0x2) ?
                   reinterpret_cast<uint32_t*>(__get_PSP()) :
                   reinterpret_cast<uint32_t*>(__get_MSP());

    stackedPC = sp[6];

    printToUSART("\n[MEMFAULT] Illegal Access @ ");
    printHex32(faultAddr);
    printToUSART(" | PC: ");
    printHex32(stackedPC);
    printToUSART("\n");
#ifdef SIMULATON
    while (1) {
        __NOP();
    }
}
#else
    NVIC_SystemReset();
#endif

}

//evaluates priorities and executes ready tasks
[[noreturn]] void executeTaskLoop() {
    while (true) {
        decideNextScheduledTask();

        __asm__ volatile("nop"); //keep CPU idle if nothing is ready
    }
}

extern "C" void start_drone_rtos() {
    SCB->CPACR |= (0xF << 20);
    __DSB(); __ISB();
    FPU->FPCCR |= FPU_FPCCR_ASPEN_Msk;
    FPU->FPCCR &= ~FPU_FPCCR_LSPEN_Msk;

    //enable MPU
    MPU->CTRL = MPU_CTRL_PRIVDEFENA_Msk | MPU_CTRL_ENABLE_Msk;
    SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk; //enable fault with its own handler

    //calibration
    printToUSART("Calibrating sensors. Keep drone still...\n");
    sensorCalibration();

#if !(SIMULATION)
    //start dma
    startCRSF_DMARead();
    startBatteryADC_DMA();
    startBaro_DMATransfer();

    //cannot call imu from here because its not a circular buffer
#endif

    //resets
    currTaskIndex = 0;
    activeTasks = 0;

    //register executeTaskLoop as a true rtos task
    initializeNewTask(executeTaskLoop, "ScheduledZoneRunner", DEFAULT_STACK);
    protectStackGuard(reinterpret_cast<uint32_t>(taskControlBlocks[activeTasks-1].taskStack) + taskControlBlocks[activeTasks-1].stackSizeWords * 4);
    taskControlBlocks[0].priority = 0; //lowest priority background worker
    taskControlBlocks[0].taskState = TaskState::RUNNING;

    currTaskAddress = &(taskControlBlocks[0].topOfStack);

    #ifdef  USER_TASKS
        registerUserTasks();
    #else
    /*===--- START TASK ENTRY ---===*/

    //interrupt tasks
        initializeNewTask(imuControlLoop,"IMUControlLoop", DEFAULT_STACK);
        taskControlBlocks[activeTasks - 1].priority = 14;
        taskControlBlocks[activeTasks - 1].taskState = TaskState::BLOCKED;

        initializeNewTask(crsfParsing, "CRSFParsing", DEFAULT_STACK);
        taskControlBlocks[activeTasks - 1].priority = 13;
        taskControlBlocks[activeTasks - 1].taskState = TaskState::BLOCKED;

        initializeNewTask(dShotGeneration, "dShotGeneration", DEFAULT_STACK);
        taskControlBlocks[activeTasks - 1].priority = 12;
        taskControlBlocks[activeTasks - 1].taskState = TaskState::BLOCKED;

        initializeNewTask(flightStateMachine, "flightStateMachine", DEFAULT_STACK);
        taskControlBlocks[activeTasks - 1].priority = 11;
        taskControlBlocks[activeTasks - 1].taskState = TaskState::BLOCKED;

    //scheduled tasks
        initializeNewTask(osdUpdate, 16, "osdUpdate", DEFAULT_STACK); //about 60 hz
        protectStackGuard(reinterpret_cast<uint32_t>(taskControlBlocks[activeTasks-1].taskStack) + taskControlBlocks[activeTasks-1].stackSizeWords * 4);
        taskControlBlocks[activeTasks - 1].priority = 10;
        taskControlBlocks[activeTasks - 1].taskState = TaskState::READY;

        initializeNewTask(radioLinkFailSafe, 20, "radioLinkFailSafe", DEFAULT_STACK);
        protectStackGuard(reinterpret_cast<uint32_t>(taskControlBlocks[activeTasks-1].taskStack) + taskControlBlocks[activeTasks-1].stackSizeWords * 4);
        taskControlBlocks[activeTasks - 1].priority = 9;
        taskControlBlocks[activeTasks - 1].taskState = TaskState::READY;

        initializeNewTask(lowLevelFailSafe, 50, "lowLevelFailSafe", DEFAULT_STACK);
        protectStackGuard(reinterpret_cast<uint32_t>(taskControlBlocks[activeTasks-1].taskStack) + taskControlBlocks[activeTasks-1].stackSizeWords * 4);
        taskControlBlocks[activeTasks - 1].priority = 8;
        taskControlBlocks[activeTasks - 1].taskState = TaskState::READY;

        initializeNewTask(powerManagement, 20, "powerManagement", DEFAULT_STACK);
        protectStackGuard(reinterpret_cast<uint32_t>(taskControlBlocks[activeTasks-1].taskStack) + taskControlBlocks[activeTasks-1].stackSizeWords * 4);
        taskControlBlocks[activeTasks - 1].priority = 7;
        taskControlBlocks[activeTasks - 1].taskState = TaskState::READY;

        initializeNewTask(gpsParser, 100, "gpsParser", DEFAULT_STACK);
        protectStackGuard(reinterpret_cast<uint32_t>(taskControlBlocks[activeTasks-1].taskStack) + taskControlBlocks[activeTasks-1].stackSizeWords * 4);
        taskControlBlocks[activeTasks - 1].priority = 6;
        taskControlBlocks[activeTasks - 1].taskState = TaskState::READY;

        initializeNewTask(readBarometerRegisters, 20, "readBarometerRegistersadBarometerRegisters", DEFAULT_STACK);
        protectStackGuard(reinterpret_cast<uint32_t>(taskControlBlocks[activeTasks-1].taskStack) + taskControlBlocks[activeTasks-1].stackSizeWords * 4);
        taskControlBlocks[activeTasks - 1].priority = 5;
        taskControlBlocks[activeTasks - 1].taskState = TaskState::READY;

        initializeNewTask(updatePeripherals, 100, "updatePeripherals", DEFAULT_STACK);
        taskControlBlocks[activeTasks - 1].priority = 4;
        taskControlBlocks[activeTasks - 1].taskState = TaskState::READY;

        initializeNewTask(usbCLI, 20, "usbCLI", DEFAULT_STACK);
        taskControlBlocks[activeTasks - 1].priority = 3;
        taskControlBlocks[activeTasks - 1].taskState = TaskState::READY;

        initializeNewTask(iwdgTask, 100, "iwdgTask", DEFAULT_STACK);
        taskControlBlocks[activeTasks - 1].priority = 2;
        taskControlBlocks[activeTasks - 1].taskState = TaskState::READY;

        initializeNewTask(checkStackHealth, 200, "checkStackHealth", DEFAULT_STACK);
        taskControlBlocks[activeTasks - 1].priority = 1;
        taskControlBlocks[activeTasks - 1].taskState = TaskState::READY;

    /*===--- END TASK ENTRY ---===*/
    #endif

    //set pendsv to lowest priority interrupt
    NVIC_SetPriority(PendSV_IRQn, 0xFF);

    SysTick->LOAD = 215999UL;
    SysTick->VAL  = 0UL;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk   |
                    SysTick_CTRL_ENABLE_Msk;

    __set_PSP(reinterpret_cast<uint32_t>(taskControlBlocks[0].topOfStack)); //set the CPU's default PSP stack to the fast task's stack to start safely
    __set_CONTROL(0x02);
    __ISB();

    rtosStarted = true; //enable yieldcurrtask etc. which uses pendsv

    //global interrupt enable
    __asm__ volatile("cpsie i" : : : "memory");

    printToUSART("Booting Drone RTOS Scheduler Kernel...\n\n");
    executeTaskLoop();
}

//EXISTING TASKS (JULY 31ST):
//Debug interrupt race condition as well as new stack overflow in the interrupts
//Add premium betaflight features like expo, gui, and proper cli

//BOOTLOADING (AUGUST 5TH):
//Figure out the proper way which code will be loaded (e.g. invisible pins, or via main microcontroler to others)
//Ensure the bootloader works on a physics stm32 board (nucelo board)

//TASK WORK (JULY 26TH):
//1. OSD
//2. Barometer parsing
//3. CHECK usbc port in parsing in the cli + update hardware specs
//4. CHECK current sensor readings (maybe reading from another instrument right now)
//5. CHECK that dshot is being pushed out the correct port

//DRIVERS WORK (JULY 27TH):
//1. Reciever module config driver (bootloading process)
//2. ESC driver work - DO NOT WRITE BY HAND - Flash other firmware e.g. BL heli
//ALL CODE SHOULD BE DONE, TESTED, AND FLASHED BY AUGUST 10TH
//ALL HARDWARE PCB'S SHOULD BE DONE AND PRINTED BY AUGUST 10TH TO GIVE TIME FOR REVISIONS
//ALL CAD SHOULD BE DONE BY AUGUST 10TH TO GIVE TIME FOR PRINTING

//CAD:
//1. Research aerodynamics and come up with initial frame by August 5th
//2. Simulate the frame + make changes (august 15th)