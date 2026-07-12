//
// Created by Akshay Gillett on 7/9/26.
//

#include "rtos.h"
#include <chrono>
#include "stm32f7xx.h"

#define MAX_TASKS 20 //update this later to a value as needed
#define SYSTICK_BASE_ADDRESS (0xE000E010UL)

static volatile uint32_t globalSystemTicks = 0;
static volatile uint32_t interruptCounter = 0; //PURELY FOR TESTING - replace with actual usart port handlers for each task needed

static TaskControlBlock taskControlBlocks[MAX_TASKS];
static int activeTasks = 0;
static int currTaskIndex = 0;

void print_char(char c) {
    // STM32F7 USART1 Base is 0x40011000.
    USART1->CR1 = USART_CR1_TE | USART_CR1_UE;

    //wait until Transmit Data Register is empty (bit 7)
    while (!(USART1->ISR & USART_ISR_TXE)) {}

    USART1->TDR = c;
}

void print_str(const char* str) {
    while (*str) {
        print_char(*str++);
    }
}

void print_uint32(uint32_t value) {
    char buffer[11]; // max "4294967295" + null terminator
    int i = 10;
    buffer[i] = '\0';

    if (value == 0) {
        buffer[--i] = '0';
    } else {
        while (value > 0) {
            buffer[--i] = '0' + (value % 10);
            value /= 10;
        }
    }

    print_str(&buffer[i]);
}

bool initializeNewTask(void (*functionAddress)(), uint32_t timePeriod, const char* textName) {
    if (activeTasks >= MAX_TASKS) { return false; }

    taskControlBlocks[activeTasks].taskCodeAddress = functionAddress;
    taskControlBlocks[activeTasks].executionPeriod = timePeriod;
    taskControlBlocks[activeTasks].taskName = textName;

    taskControlBlocks[activeTasks].taskState = TaskState::BLOCKED;

    //point to the very end of the 1024-word array idx = 1023
    taskControlBlocks[activeTasks].topOfStack = &taskControlBlocks[activeTasks].taskStack[1023];

    //back up 16 slots to leave room for the context switcher's registers
    taskControlBlocks[activeTasks].topOfStack -= 16;

    //clear the slots for R0-R12 and LR (slots 0 to 13)
    for (int i = 0; i < 14; i++) { taskControlBlocks[activeTasks].topOfStack[i] = 0; }

    //set the PC slot (14) to the function address and xPSR slot (15) to Thumb mode
    taskControlBlocks[activeTasks].topOfStack[14] = reinterpret_cast<uint32_t>(functionAddress);
    taskControlBlocks[activeTasks].topOfStack[15] = 0x01000000;

    //initialize time stamp with custom function
    taskControlBlocks[activeTasks].lastRunTime = globalSystemTicks;

    activeTasks++;
    return true;
}

extern "C" uint32_t** currTaskAddress = nullptr;
extern "C" uint32_t** nextTaskAddress = nullptr;

extern "C" void yieldCurrentTask() {
    taskControlBlocks[currTaskIndex].taskState = TaskState::BLOCKED; //block the current interrupt and go back to the scheduler process
    int nextTaskIdx = 0;

    currTaskAddress = &(taskControlBlocks[currTaskIndex].topOfStack);
    nextTaskAddress = &(taskControlBlocks[nextTaskIdx].topOfStack);

    currTaskIndex = nextTaskIdx;
    taskControlBlocks[currTaskIndex].taskState = TaskState::RUNNING;

    //fire the context switch back to the scheduled loop using pendSV
    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
    __ISB();
}

extern "C" void decideNextInterruptTask(TaskControlBlock* interruptTask) {
    // Save the state of the scheduled runner before switching
    currTaskAddress = &(taskControlBlocks[currTaskIndex].topOfStack);

    interruptTask->taskState = TaskState::RUNNING;
    nextTaskAddress = &(interruptTask->topOfStack);

    //find its index in the array to update tracking
    for (int i = 0; i < activeTasks; i++) {
        if (&taskControlBlocks[i] == interruptTask) {
            currTaskIndex = i;
            break;
        }
    }

    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
}

extern "C" void decideNextScheduledTask() {
    int highestPriorityTaskIndex = -1;
    int highestPriorityValue = -1;

    for (int i = 0; i < activeTasks; i++) {
        if (taskControlBlocks[i].taskState == TaskState::READY) {
            if (static_cast<int>(taskControlBlocks[i].priority) > highestPriorityValue) {
                highestPriorityValue = taskControlBlocks[i].priority;
                highestPriorityTaskIndex = i;
            }
        }
    }

    if (highestPriorityTaskIndex != -1) {
        TaskControlBlock* task = &taskControlBlocks[highestPriorityTaskIndex];

        task->taskState = TaskState::RUNNING;
        task->taskCodeAddress();

        task->lastRunTime = globalSystemTicks;
        task->taskState = TaskState::BLOCKED;
    }
}


extern "C" void SysTick_Handler() {
    globalSystemTicks++;

    for (int i = 0; i < activeTasks; i++) {

        if (taskControlBlocks[i].executionPeriod == 0) { continue;}

        if (taskControlBlocks[i].taskState == TaskState::BLOCKED) {
            if (globalSystemTicks - taskControlBlocks[i].lastRunTime >= taskControlBlocks[i].executionPeriod) {
                taskControlBlocks[i].taskState = TaskState::READY;
            }
        }
    }

    interruptCounter++;
    if (interruptCounter >= 10000) { //every 10 seconds
        interruptCounter = 0;

        //ensure the interrupt task isn't already running before calling it
        if (taskControlBlocks[1].taskState == TaskState::BLOCKED) {
            decideNextInterruptTask(&taskControlBlocks[1]);
        }
    }
}

//evaluates priorities and executes ready tasks
[[noreturn]] void executeTaskLoop() {
    while (true) {
        decideNextScheduledTask();

        __asm__ volatile("nop"); //keep CPU idle if nothing is ready
    }
}

//REMOVE THESE AFTER CONFIRMING TESTING
//scheduled tasks
void taskFast()   { print_str("F"); } // print a single to eliminate lag in console
void taskMedium() { print_str("M"); }
void taskSlow()   { print_str("S"); }

[[noreturn]] void taskInterrupt() {
    while (true) {
        print_str("\n[INT START]\n");

        for (volatile uint32_t i = 0; i < 20000000UL; i++) {
            __asm__ volatile("nop");
        }

        print_str("[INT END]\n");
        yieldCurrentTask();
    }
}

extern "C" void start_drone_rtos() {
    SysTick->LOAD = 215999UL;
    SysTick->VAL  = 0UL;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk   |
                    SysTick_CTRL_ENABLE_Msk;

    //Global interrupt enable
    __asm__ volatile("cpsie i" : : : "memory");

    //set pendsv to lowest priority interrupt
    NVIC_SetPriority(PendSV_IRQn, 0xFF);

    //register executeTaskLoop as a true rtos task
    initializeNewTask(executeTaskLoop, 0, "ScheduledZoneRunner");
    taskControlBlocks[0].priority = 0; //lowest priority background worker
    taskControlBlocks[0].taskState = TaskState::RUNNING;

    currTaskIndex = 0;
    currTaskAddress = &(taskControlBlocks[0].topOfStack);

    //dummy interrupt tasks
    initializeNewTask(taskInterrupt, 0, "Interrupt Task"); //interrupt tasks MUST be initialized with 0
    taskControlBlocks[activeTasks - 1].priority = 20; //interrupt tasks MUST have highest priority

    //dummy scheduled tasks
    initializeNewTask(taskFast, 10, "FastTask");
    taskControlBlocks[activeTasks - 1].priority = 1;

    initializeNewTask(taskMedium, 500, "MedTask");
    taskControlBlocks[activeTasks - 1].priority = 2;

    initializeNewTask(taskSlow, 1000, "SlowTask");
    taskControlBlocks[activeTasks - 1].priority = 3;

    //set the CPU's default PSP stack to the fast task's stack to start safely
    __set_PSP(reinterpret_cast<uint32_t>(taskControlBlocks[0].topOfStack));
    __set_CONTROL(0x02);
    __ISB(); // Instruction Synchronization Barrier to apply the change instantly

    print_str("Booting Drone RTOS Scheduler Kernel...\n\n");

    executeTaskLoop();
}

//Task code: Instead of returning a value, tasks pass data to each other using global/file-scope thread-safe buffers, lock-free queues, or shared state structs.

//Interrupt zone:
void readIMUDataRegisters(){} //reads imu and does calculations
void stateEstimation(){} //low level pass filters (e.g. kalman filter), and updates structs used for calculations
void flightLoop(){} //pid and mixer calculations
void crsfParsing(){} //parses incoming radio signals
void radioLinkFailSafe(){} //instant disarm if radio link drops for more than 200 ms
void dShotGeneration(){} //cleans up the DMA buffers for next cycle to esc

//Scheduled zone: Must use while (true) loops
//high priority
[[noreturn]] void lowLevelFailSafe(){ while (true){} } //for non emergencies e.g. low battery or gps ping low

//medium priority
[[noreturn]] void telemetryTX(){ while (true){} } //logs telemetry on the radio
[[noreturn]] void osdUpdate(){ while (true){} } //updates osd values for the telemetry in the video footage
[[noreturn]] void gpsParser(){ while (true){} } //updates location and transmits

//low priority
[[noreturn]] void blackboxLogging(){ while (true){} } //most likely remove as we have no onboard storage
[[noreturn]] void updatePeripherals(){ while (true){} } //controls leds, beeper, and smartaudio for the vtx
[[noreturn]] void usbCLI(){ while (true){} } //handles connections via the usbc port e.g. cli adjustments, pid tuning, and config flashing etc.

//iswg (Independent Watchdog)
[[noreturn]] void iswg(){while (true){} } // operates in a different part of the mcu, completely diff from the actual thread

