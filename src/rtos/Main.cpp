//
// Main.cpp by Akshay Gillett on 7/9/26.
//

#include <atomic>

#include "logging/Logger.h"
#include "rtos.h"
#include "stm32f7xx.h"
#include "../../User_Tasks/user_tasks.h"
#include "../flight/flight_control.h"
#include "../flight/BufferPopulation.h"
#include "../sim/SimInjector.h"
#include <cstdlib>

volatile uint32_t globalSystemTicks = 0;
volatile bool rtosStarted = false;

TaskControlBlock taskControlBlocks[MAX_TASKS];
int activeTasks = 0;

int currTaskIndex = 0;
int nextTaskIndex = 0;

int currLoopIndex;

// uint32_t** currTaskAddress = nullptr;
// uint32_t** nextTaskAddress = nullptr;
// uint32_t* currExcReturnAddress = nullptr; //used for task recollection (bx lr bug) because r0 sometimes gets corrupted
// uint32_t* nextExcReturnAddress = nullptr;
//
// //universal helpers to stop pendSV getting overwritten
// bool switchPending = false;
// static int pendingWinnerIndex = -1;
//
// static int preemptStack[MAX_TASKS];
// int preemptDepth = 0;

//only two variables for scheduling now
static uint32_t pendingInterruptMask = 0;
volatile uint32_t nextExcReturn;

//sim helpers
#ifdef SIMULATION

static void injectMockBuffers() { //used for "mock" dma
    injectMockIMUDMA();
    injectMockCRSFDMA();
    injectMockADCDMA();
    injectMockGPSDMA();
    injectMockOSDDMA();

    //injectMockBaroDMA() was removed as it is called 'on demand' in population when necessary
}

static void executeRandomInterrupts() {
    if (globalSystemTicks % 1 == 0) { //every 1 ms - task 1
        #ifdef DEBUG
        printToUSART("[SIM-ERI] Attempting to call IMU from IRQ\n\n");
        #endif

        mock_LISR |= DMA_LISR_TCIF0; //artificially clear nvic through software buffers because actual registers are read only
        NVIC_SetPendingIRQ(DMA1_Stream0_IRQn); //artificially call nvic
    }

    if (globalSystemTicks % 1 == 0) { //every 1 ms - task 2
        #ifdef DEBUG
        printToUSART("[SIM-ERI] Attempting to call CRSF from IRQ\n\n");
        #endif

        mock_HISR |= DMA_HISR_TCIF5;
        NVIC_SetPendingIRQ(DMA1_Stream5_IRQn);
    }

}
#endif

//irq handlers - get called automatically by dma or manually when simulating
extern "C" void DMA1_Stream0_IRQHandler(void) {
#ifdef DEBUG
    printToUSART("[DMA-IRQ] Calling DMA1_Stream0_IRQHandler | IMUControlLoop\n");
#endif

#ifdef SIMULATION
    uint32_t flags = mock_LISR;
    mock_LISR = 0;
#else
    uint32_t flags = DMA1->LISR;
#endif

    auto callTask = []() {
        TaskControlBlock* task = findTaskByName("imuControlLoop"); //use lambda for less redundant code
        if (task) { decideNextInterruptTask(task); } //ensure the find doesn't throw a nullptr
        else {
            drone.currentSystemState = FlightState::ERROR;
            setEventMessage(drone.errorMSG, ERROR::CANNOT_FIND_TASK_ICL);
        }
    };

    if (flags & DMA_LISR_HTIF0) { //half transfer
        DMA1->LIFCR = DMA_LIFCR_CHTIF0; //clear flag
        callTask(); //call lambda
    }

    if (flags & DMA_LISR_TCIF0) { //full transfer
        DMA1->LIFCR = DMA_LIFCR_CTCIF0; //clear flag
        callTask(); //call lambda
    }
}

extern "C" void DMA1_Stream5_IRQHandler(void) {
#ifdef DEBUG
    printToUSART("[DMA-IRQ] Calling DMA1_Stream5_IRQHandler | crsfControlLoop\n");
#endif

#ifdef SIMULATION
    uint32_t flags = mock_HISR;
    mock_HISR = 0;
#else
    uint32_t flags = DMA1->HISR; //streams 0-3 use LISR while streams 4-7 use HISR
#endif

    auto callTask = []() {
        TaskControlBlock* task = findTaskByName("crsfControlLoop"); //use lambda for less redundant code
        if (task) { decideNextInterruptTask(task); } //ensure the find doesn't throw a nullptr
        else {
            drone.currentSystemState = FlightState::ERROR; //throw errors on higher level tasks as without the drone will crash or fly away
            setEventMessage(drone.errorMSG, ERROR::CANNOT_FIND_TASK_CRSF);
        }
    };

    if (flags & DMA_HISR_HTIF5) {
        DMA1->HIFCR = DMA_HIFCR_CHTIF5;
        callTask();
    }

    if (flags & DMA_HISR_TCIF5) {
        DMA1->HIFCR = DMA_HIFCR_CTCIF5;
        callTask();
    }
}

//actual rtos logic
static bool initializeNewTask(void (*functionAddress)(), uint32_t timePeriod, const char* textName, uint32_t stackSizeWords = 256) { //set the default to scheduled
    if (activeTasks >= MAX_TASKS) { return false; }

    //dynamic memory management:
    // auto* newStack = new (std::nothrow) uint32_t[stackSizeWords + 1];
        constexpr uint32_t GUARD_REGION_SIZE = 32; //defines the size of the mpu region
        const uint32_t totalBytes = (stackSizeWords * 4) + GUARD_REGION_SIZE;
        auto* newStack = static_cast<uint32_t*>(aligned_alloc(GUARD_REGION_SIZE, totalBytes));

    if (newStack == nullptr) { //max mem exceded (out of ram/sram on actual chip - not just software)
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
            newStack[i] = 0xA5A5A5A5; //allocates a specific "word" which is used for task mem management (helps detect overflows + mem management task)
        }

        //check floating point registers
        //point to the very ends of the 1024-word array idx = 1023
        taskControlBlocks[activeTasks].topOfStack = &newStack[stackSizeWords - 1];
        taskControlBlocks[activeTasks].topOfStack -= 32;

        //clear the slots for R0-R12 and LR (slots 0 to 13)
        for (int i = 0; i < 32; i++) { taskControlBlocks[activeTasks].topOfStack[i] = 0; }

    //set the PC slot (14) to the function address and xPSR slot (15) to Thumb mode
    taskControlBlocks[activeTasks].excReturn = 0xFFFFFFFD;//manally set LR (slot 45) to a dedicated address rather than undef/auto assigned
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

    taskControlBlocks[activeTasks].excReturn = 0xFFFFFFFD;
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

static TaskControlBlock* findNextTask(TaskType taskType) {
    int nextTaskPriority = -1;
    int nextTaskIdx = 0; //defaults to scheduler if no tasks are ready

    for (int i = 1; i < activeTasks; i++) { //skip first task as it is the scheduler
        TaskControlBlock* inspectedTask = &taskControlBlocks[i];

        if (inspectedTask->taskType != taskType ||
            inspectedTask->taskState != TaskState::READY ||
            inspectedTask->priority <= nextTaskPriority
           ) { continue; }

        nextTaskPriority = i;
        nextTaskIdx = i;
    }

    return &taskControlBlocks[nextTaskIdx];
}

extern "C" uint32_t* decideNextTask(uint32_t* outgoingSP, uint32_t outgoingExcReturn) {
    taskControlBlocks[currTaskIndex].topOfStack = outgoingSP;
    taskControlBlocks[currTaskIndex].excReturn  = outgoingExcReturn;

    if (taskControlBlocks[currTaskIndex].taskState == TaskState::RUNNING) {
        taskControlBlocks[currTaskIndex].taskState = TaskState::READY;   // preempted, not finished
    }

    int nextIndex = -1, nextPriority = -1;
    for (int i = 0; i < activeTasks; i++) {   // interrupt tasks win over scheduled, same as before
        if (taskControlBlocks[i].taskType == TaskType::INTERRUPT && taskControlBlocks[i].taskState == TaskState::READY) {
            if (static_cast<int>(taskControlBlocks[i].priority) > nextPriority) {
                nextPriority = taskControlBlocks[i].priority;
                nextIndex = i;
            }
        }
    }
    if (nextIndex == -1) {
        for (int i = 0; i < activeTasks; i++) {
            if (taskControlBlocks[i].taskType == TaskType::SCHEDULED && taskControlBlocks[i].taskState == TaskState::READY) {
                if (static_cast<int>(taskControlBlocks[i].priority) > nextPriority) {
                    nextPriority = taskControlBlocks[i].priority;
                    nextIndex = i;
                }
            }
        }
    }
    if (nextIndex == -1) nextIndex = 0;   // idle fallback

    currTaskIndex = nextIndex;
    taskControlBlocks[nextIndex].taskState = TaskState::RUNNING;
    nextExcReturn = taskControlBlocks[nextIndex].excReturn;
    return taskControlBlocks[nextIndex].topOfStack;
}

void yieldCurrentTask() {
    if (!rtosStarted) return;
    __disable_irq();
    taskControlBlocks[currTaskIndex].taskState = TaskState::BLOCKED;
    __enable_irq();
    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
    __ISB();
}

void decideNextInterruptTask(TaskControlBlock* interruptTask) {
    __disable_irq();
    interruptTask->taskState = TaskState::READY;
    __enable_irq();
    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
}

void decideNextScheduledTask() {
    // idle just asks for a switch; decideNextTask finds whichever scheduled task is READY
    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
    __ISB();
}

TaskControlBlock* findTaskByName(const char *textName) {
    for (int i = 0; i < activeTasks; i++) {
        if (taskControlBlocks[i].taskName == textName) {
            return &taskControlBlocks[i];
        }
    }
    return nullptr;
}

extern "C" void SysTick_Handler() {
    HAL_IncTick();
    globalSystemTicks++;

    // if (preemptDepth == 0) {
    //     switchPending = false;
    //     pendingWinnerIndex = -1;
    // }

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

    #if SIMULATION && !(USER_TASKS)
        //populate mock buffers
        injectMockBuffers();
        executeRandomInterrupts();
        //actual dma interrupts are handled automatically via irq
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
#ifdef SIMULATION
    while (1) {
        __NOP();
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

static void initSystemGPIOClocks() {
    //enable AHB1 peripheral clock gates for all active GPIO ports
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN  //IMU CS, UARTs
                 |  RCC_AHB1ENR_GPIOBEN  //status LED, SPI2, SPI3
                 |  RCC_AHB1ENR_GPIOCEN  //buzzer, ADC1, SDMMC Data
                 |  RCC_AHB1ENR_GPIODEN  //baro CS (PD3), SDMMC CMD (PD2)
                 |  RCC_AHB1ENR_GPIOEEN; //TIM1 Motor DShot outputs

    //force a short delay until clock stabilization completes
    const __IO uint32_t reg = RCC->AHB1ENR;
    (void) reg;
}

static void configSTLINK() {
    // 1. Turn on the clocks for GPIOD and USART3
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
    __DSB(); // Wait for clock stabilization

    // 2. Mux PD8 (TX) and PD9 (RX) to Alternate Function Mode (0b10)
    GPIOD->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9);
    GPIOD->MODER |= (GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1);

    // 3. Set the specific alternate function to AF7 (USART)
    GPIOD->AFR[1] &= ~((0xF << 0) | (0xF << 4)); // Clear AFRH8 and AFRH9
    GPIOD->AFR[1] |= (7 << 0) | (7 << 4);        // Set AF7 for both pins
}

extern "C" void start_drone_rtos() {
    //fpu
    SCB->CPACR |= (0xF << 20);
    __DSB(); __ISB();
    FPU->FPCCR |= FPU_FPCCR_ASPEN_Msk;
    FPU->FPCCR &= ~FPU_FPCCR_LSPEN_Msk;

    //enable MPU
    MPU->CTRL = MPU_CTRL_PRIVDEFENA_Msk | MPU_CTRL_ENABLE_Msk;
    SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk; //enable fault with its own handler

    configSTLINK();

    //set baud rate properly
    currentBoardConfig.log_uart->BRR = HAL_RCC_GetPCLK1Freq() / 115200;

    //enable gpio clocks
    initSystemGPIOClocks();

    //calibration
    printToUSART("Calibrating sensors. Keep drone still...\n");

    //get board stats
#ifdef DEBUG
    printToUSART("[BOOT] SYSCLK source (0=HSI,1=HSE,2=PLL): ");
    printToUSART((uint32_t)((RCC->CFGR >> 2) & 3));
    printToUSART("\n");
#endif

    sensorCalibration();

    //resets
    currTaskIndex = 0;
    activeTasks = 0;

    //register executeTaskLoop as a true rtos task
    initializeNewTask(executeTaskLoop, "ScheduledZoneRunner", DEFAULT_STACK);
    protectStackGuard(reinterpret_cast<uint32_t>(taskControlBlocks[activeTasks-1].taskStack) + taskControlBlocks[activeTasks-1].stackSizeWords * 4);
    taskControlBlocks[0].priority = 0; //lowest priority background worker
    taskControlBlocks[0].taskState = TaskState::RUNNING;

    // currTaskAddress = &(taskControlBlocks[0].topOfStack);


    #ifdef  USER_TASKS
        registerUserTasks();
    #else
    /*===--- START TASK ENTRY ---===*/

    //interrupt tasks
        //needs external python simulation (in)
        initializeNewTask(imuControlLoop,"imuControlLoop", DEFAULT_STACK*2);
        protectStackGuard(reinterpret_cast<uint32_t>(taskControlBlocks[activeTasks-1].taskStack) + taskControlBlocks[activeTasks-1].stackSizeWords * 4);
        taskControlBlocks[activeTasks - 1].priority = 16;
        taskControlBlocks[activeTasks - 1].taskState = TaskState::BLOCKED;

        //needs external python simulation (in)
        initializeNewTask(crsfControlLoop, "crsfControlLoop", DEFAULT_STACK*2);
        protectStackGuard(reinterpret_cast<uint32_t>(taskControlBlocks[activeTasks-1].taskStack) + taskControlBlocks[activeTasks-1].stackSizeWords * 4);
        taskControlBlocks[activeTasks - 1].priority = 15;
        taskControlBlocks[activeTasks - 1].taskState = TaskState::BLOCKED;

    //scheduled tasks
        initializeNewTask(respondToVTX, 20, "respondToVTX", DEFAULT_STACK); //was blocking due to dma checks (which don't exist in simulation)
        protectStackGuard(reinterpret_cast<uint32_t>(taskControlBlocks[activeTasks-1].taskStack) + taskControlBlocks[activeTasks-1].stackSizeWords * 4);
        taskControlBlocks[activeTasks - 1].priority = 12;
        taskControlBlocks[activeTasks - 1].taskState = TaskState::READY;

        //needs external python simulation  (out)
        initializeNewTask(osdUpdate, 16, "osdUpdate", DEFAULT_STACK); //about 60 hz
        protectStackGuard(reinterpret_cast<uint32_t>(taskControlBlocks[activeTasks-1].taskStack) + taskControlBlocks[activeTasks-1].stackSizeWords * 4);
        taskControlBlocks[activeTasks - 1].priority = 11;
        taskControlBlocks[activeTasks - 1].taskState = TaskState::READY;

        initializeNewTask(radioLinkFailSafe, 20, "radioLinkFailSafe", DEFAULT_STACK);
        taskControlBlocks[activeTasks - 1].priority = 10;
        taskControlBlocks[activeTasks - 1].taskState = TaskState::READY;

        initializeNewTask(lowLevelFailSafe, 50, "lowLevelFailSafe", DEFAULT_STACK);
        protectStackGuard(reinterpret_cast<uint32_t>(taskControlBlocks[activeTasks-1].taskStack) + taskControlBlocks[activeTasks-1].stackSizeWords * 4);
        taskControlBlocks[activeTasks - 1].priority = 9;
        taskControlBlocks[activeTasks - 1].taskState = TaskState::READY;

        //needs external python simulation (in)
        initializeNewTask(powerManagement, 20, "powerManagement", DEFAULT_STACK);
        protectStackGuard(reinterpret_cast<uint32_t>(taskControlBlocks[activeTasks-1].taskStack) + taskControlBlocks[activeTasks-1].stackSizeWords * 4);
        taskControlBlocks[activeTasks - 1].priority = 8;
        taskControlBlocks[activeTasks - 1].taskState = TaskState::READY;

        initializeNewTask(telemetryTX, 20, "telemetryTX", DEFAULT_STACK);
        taskControlBlocks[activeTasks - 1].priority = 7;
        taskControlBlocks[activeTasks - 1].taskState = TaskState::READY;

        //needs external python simulation (in)
        initializeNewTask(gpsParser, 100, "gpsParser", DEFAULT_STACK); //fix injector
        taskControlBlocks[activeTasks - 1].priority = 6;
        taskControlBlocks[activeTasks - 1].taskState = TaskState::READY;

        //needs external python simulation (in)
        initializeNewTask(readBarometerRegisters, 20, "readBarometerRegisters", DEFAULT_STACK); //need to fix injector
        taskControlBlocks[activeTasks - 1].priority = 5;
        taskControlBlocks[activeTasks - 1].taskState = TaskState::READY;

        //needs external python simulation (out)
        initializeNewTask(updatePeripherals, 100, "updatePeripherals", DEFAULT_STACK);
        taskControlBlocks[activeTasks - 1].priority = 4;
        taskControlBlocks[activeTasks - 1].taskState = TaskState::READY;

        //needs external python simulation (in, out) - already done (in)
        initializeNewTask(usbCLI, 20, "usbCLI", DEFAULT_STACK); //need to add permanent blackbox + changeable osd
        taskControlBlocks[activeTasks - 1].priority = 3;
        taskControlBlocks[activeTasks - 1].taskState = TaskState::READY;

        initializeNewTask(iwdgTask, 200, "iwdgTask", DEFAULT_STACK);
        taskControlBlocks[activeTasks - 1].priority = 2;
        taskControlBlocks[activeTasks - 1].taskState = TaskState::READY;

#ifdef DEBUG
        initializeNewTask(checkStackHealth, 200, "checkStackHealth", DEFAULT_STACK);
        taskControlBlocks[activeTasks - 1].priority = 1;
        taskControlBlocks[activeTasks - 1].taskState = TaskState::READY;
#endif

    /*===--- END TASK ENTRY ---===*/


    #endif

#if !(SIMULATION)
    //start dma
    startIMU_DMARead(); //converted to circular buffer
    startCRSF_DMARead();
    startBatteryADC_DMA();
    startGPS_DMARead();
    startOSD_DMARead();

    //init dma for osd, baro, and telemetry
    initOSD_DMATransfer(); //NOT for normal OSD - used in dual vtx connection
    initTTX_DMATransfer();
    initBaro_DMATransfer();
#endif

    //set pendsv to lowest priority interrupt
    NVIC_SetPriority(PendSV_IRQn, 0xFF); //calls systick which executes task loop

    //start nvic regardless because testing also uses it
    NVIC_SetPriority(DMA1_Stream0_IRQn, 1); //imu
    NVIC_EnableIRQ(DMA1_Stream0_IRQn); //enabled irq interrupt which calls parsing

    NVIC_SetPriority(DMA1_Stream5_IRQn, 2); //crsf
    NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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