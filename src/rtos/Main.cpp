//
// Created by Akshay Gillett on 7/9/26.
//

#include "Logger.h"
#include "rtos.h"
#include "stm32f7xx.h"
#include "../../User_Tasks/user_tasks.h"
#include "../flight/flight_control.h"
#include "../flight/BufferPopulation.h"

#define MAX_TASKS 13 //update this later to a value as needed
#define SYSTICK_BASE_ADDRESS (0xE000E010UL)

volatile uint32_t globalSystemTicks = 0;
volatile bool rtosStarted = false;


TaskControlBlock taskControlBlocks[MAX_TASKS];
int activeTasks = 0;
static int currTaskIndex = 0;

static int preemptStack[MAX_TASKS];
static int preemptDepth = 0;

//sim helpers
#ifdef SIMULATION

    static uint32_t userSeed = 98765;
    static uint32_t getRandom(int range) { // Used to find a random number
        userSeed = (1103515245 * userSeed + 12345) % 2147483648;
        return (userSeed % range) + 1;
    }

    void executeRandomInterrupts() {
        // if (getRandom(100) == 50) { decideNextInterruptTask(&taskControlBlocks[2]); }
        // if (getRandom(100) == 50) { decideNextInterruptTask(&taskControlBlocks[1]); }
    }

#endif


//actual rtos logic
bool initializeNewTask(void (*functionAddress)(), uint32_t timePeriod, const char* textName) { //set the default to scheduled
    if (activeTasks >= MAX_TASKS) { return false; }

    taskControlBlocks[activeTasks].taskCodeAddress = functionAddress;
    taskControlBlocks[activeTasks].executionPeriod = timePeriod;
    taskControlBlocks[activeTasks].taskName = textName;
    taskControlBlocks[activeTasks].taskState = TaskState::BLOCKED;

    taskControlBlocks[activeTasks].taskType = TaskType::SCHEDULED;

    //point to the very ends of the 1024-word array idx = 1023
    taskControlBlocks[activeTasks].topOfStack = &taskControlBlocks[activeTasks].taskStack[2047];
    taskControlBlocks[activeTasks].taskStack[0] = 0xDEADBEEF;
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
bool initializeNewTask(void (*functionAddress)(), const char* textName) { //set the default to scheduled
    if (activeTasks >= MAX_TASKS) { return false; }

    taskControlBlocks[activeTasks].taskCodeAddress = functionAddress;
    taskControlBlocks[activeTasks].taskName = textName;
    taskControlBlocks[activeTasks].taskState = TaskState::BLOCKED;

    taskControlBlocks[activeTasks].taskType = TaskType::INTERRUPT;

    taskControlBlocks[activeTasks].topOfStack = &taskControlBlocks[activeTasks].taskStack[2047];
    taskControlBlocks[activeTasks].taskStack[0] = 0xDEADBEEF;
    taskControlBlocks[activeTasks].topOfStack -= 32;

    for (int i = 0; i < 32; i++) { taskControlBlocks[activeTasks].topOfStack[i] = 0; } //reset all to 0
        taskControlBlocks[activeTasks].topOfStack[29] = 0xFFFFFFFD;
        taskControlBlocks[activeTasks].topOfStack[30] = reinterpret_cast<uint32_t>(functionAddress);
        taskControlBlocks[activeTasks].topOfStack[31] = 0x01000000;

    taskControlBlocks[activeTasks].lastRunTime = globalSystemTicks;

    activeTasks++;
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
        preemptStack[preemptDepth++] = pendingWinnerIndex;

        //it won't get called by systick because it doesn't have a time interval
        taskControlBlocks[pendingWinnerIndex].taskState = TaskState::READY;
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

        taskControlBlocks[currTaskIndex].taskState = TaskState::READY; //set it to running so it restores where it left off
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

    // for (int i = 0; i < activeTasks; i++) {
    //     printToUSART("Task "); printToUSART(i); printToUSART(": name=");
    //     printToUSART(taskControlBlocks[i].taskName);
    //     printToUSART(" canary="); printToUSART(taskControlBlocks[i].taskStack[0]);
    //     printToUSART(" state="); printToUSART(static_cast<uint32_t>(taskControlBlocks[i].taskState));
    //     printToUSART(" prio="); printToUSART(static_cast<uint32_t>(taskControlBlocks[i].priority));
    //     printToUSART("\n");
    // }

    for (int i = 0; i < activeTasks; i++) {

        if (taskControlBlocks[i].taskType == TaskType::INTERRUPT) { continue; } //use instead of a 0 ms execution period

        if (taskControlBlocks[i].taskState == TaskState::BLOCKED) {
            if (globalSystemTicks - taskControlBlocks[i].lastRunTime >= taskControlBlocks[i].executionPeriod) {
                taskControlBlocks[i].taskState = TaskState::READY;
            }
        }
    }

    #ifdef USER_TASKS
        callUserInterruptTasks();
    #endif

    #ifdef SIMULATION
        executeRandomInterrupts();
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

    printToUSART("Calibrating sensors. Keep drone still...\n");
    sensorCalibration();
    startCRSF_DMARead();

    currTaskIndex = 0;
    activeTasks = 0;

    //register executeTaskLoop as a true rtos task
    initializeNewTask(executeTaskLoop, "ScheduledZoneRunner");
    taskControlBlocks[0].priority = 0; //lowest priority background worker
    taskControlBlocks[0].taskState = TaskState::RUNNING;

    currTaskAddress = &(taskControlBlocks[0].topOfStack);

    #ifdef  USER_TASKS
        registerUserTasks();
    #else
    /*===--- START TASK ENTRY ---===*/

    //interrupt tasks
        initializeNewTask(imuControlLoop,"IMUControlLoop");
        taskControlBlocks[activeTasks - 1].priority = 13;
        taskControlBlocks[activeTasks - 1].taskState = TaskState::BLOCKED;

        initializeNewTask(crsfParsing, "CRSFParsing");
        taskControlBlocks[activeTasks - 1].priority = 12;
        taskControlBlocks[activeTasks - 1].taskState = TaskState::BLOCKED;

        initializeNewTask(dShotGeneration, "dShotGeneration");
        taskControlBlocks[activeTasks - 1].priority = 11;
        taskControlBlocks[activeTasks - 1].taskState = TaskState::BLOCKED;

        initializeNewTask(flightStateMachine, "flightStateMachine");
        taskControlBlocks[activeTasks - 1].priority = 10;
        taskControlBlocks[activeTasks - 1].taskState = TaskState::BLOCKED;

    //scheduled tasks

        initializeNewTask(radioLinkFailSafe, 50, "RadioLinkFailSafe");
        taskControlBlocks[activeTasks - 1].priority = 9;
        taskControlBlocks[activeTasks - 1].taskState = TaskState::READY;

        // initializeNewTask(lowLevelFailSafe, 50, "lowLevelFailSafe");
        // taskControlBlocks[activeTasks - 1].priority = 8;
        // taskControlBlocks[activeTasks - 1].taskState = TaskState::READY;

        initializeNewTask(powerManagement, 50, "powerManagement");
        taskControlBlocks[activeTasks - 1].priority = 7;
        taskControlBlocks[activeTasks - 1].taskState = TaskState::READY;

        //this function is overflowing
        // initializeNewTask(telemetryTX, 100, "telemetryTX");
        // taskControlBlocks[activeTasks - 1].priority = 6;
        // taskControlBlocks[activeTasks - 1].taskState = TaskState::READY;

        initializeNewTask(updatePeripherals, 100, "updatePeripherals");
        taskControlBlocks[activeTasks - 1].priority = 5;
        taskControlBlocks[activeTasks - 1].taskState = TaskState::READY;

        initializeNewTask(usbCLI, 50, "usbCLI");
        taskControlBlocks[activeTasks - 1].priority = 4;
        taskControlBlocks[activeTasks - 1].taskState = TaskState::READY;

        initializeNewTask(iwdgTask, 100, "iwdgTask");
        taskControlBlocks[activeTasks - 1].priority = 3;
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