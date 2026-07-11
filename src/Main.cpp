//
// Created by Akshay Gillett on 7/9/26.
//

#include "Main.h"

#include <chrono>

#define MAX_TASKS 12 //update this later to a value as needed
#define SYSTICK_BASE_ADDRESS (0xE000E010UL)

//simulation setup
//reserve a tiny block of memory for the minimum required ARM Vector Table
//reserve a tiny block of memory for the minimum required ARM Vector Table

__attribute__((section(".isr_vector"), used))
void (* const g_pfnVectors[])(void) = {
    reinterpret_cast<void (*)()>(static_cast<uint32_t>(0x20004000)), // 1. Fake initial stack pointer (Top of RAM)
    reinterpret_cast<void (*)()>(main),                              // 2. Reset Handler (Where the CPU boots)
    nullptr,                                                         // 3. NMI Handler
    reinterpret_cast<void (*)()>(main),                              // 4. HardFault Handler
    nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,   // 5-11. Reserved
    nullptr,                                                         // 12. SVCall Handler
    nullptr, nullptr,                                                // 13-14. Reserved
    nullptr,                                                         // 15. PendSV Handler
    static_cast<void (*)()>(SysTick_Handler)                         // 16. SysTick Exception Vector!
};

enum class TaskState : uint8_t {
    BLOCKED = 0,
    READY   = 1,
    RUNNING = 2
};

struct TaskControlBlock {
    //function to run the code
    void (*taskCodeAddress)();

    uint32_t executionPeriod;
    uint32_t lastRunTime;

    //debug purposes only
    const char* taskName;

    //priority
    uint8_t priority; //1-12 (may increase limit later) - be prepared to refactor priority many times
    TaskState taskState; //READY, RUNNING, BLOCKED/WAITING
};

static volatile uint32_t globalSystemTicks = 0;

static TaskControlBlock taskControlBlocks[MAX_TASKS];
static int activeTasks = 0;

void print_char(char c) {
    // STM32 USART1 Data Register memory location
    volatile uint32_t* USART1_DR = (volatile uint32_t*)0x40011004;
    *USART1_DR = c;
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

    //initialize time stamp with custom function
    taskControlBlocks[activeTasks].lastRunTime = globalSystemTicks;

    activeTasks++;
    return true;
}

extern "C" void SysTick_Handler() { //auto called every 1ms
     //print_str("running SysTick_Handler"); //printing manually every time is risky for terminal buffer

    globalSystemTicks++;

    //loop through all active tasks
    for (int i = 0; i < activeTasks; i++) {
        //if a task is blocked, check if its execution period has elapsed
        if (taskControlBlocks[i].taskState == TaskState::BLOCKED) {
            if (globalSystemTicks - taskControlBlocks[i].lastRunTime >= taskControlBlocks[i].executionPeriod) {

                // Turn the task ON by marking it READY
                taskControlBlocks[i].taskState = TaskState::READY;

                print_str("READY ");
                print_str(taskControlBlocks[i].taskName);
                print_str("\n");
            }
        }
    }
}

//evaluates priorities and executes ready tasks
[[noreturn]] void executeTaskLoop() {
    uint32_t lastPrintedTick = 0xFFFFFFFF; //initialize to a dummy value

    while (true) {

        if (globalSystemTicks != lastPrintedTick) {
            print_str("\n[Tick: ");
            print_uint32(globalSystemTicks);
            print_str("] ");
            lastPrintedTick = globalSystemTicks;
        }

        int highestPriorityTaskIndex = -1;
        int highestPriorityValue = -1; //higher number = higher priority (1-12)

        //scan registry for the highest priority task that is READY
        for (int i = 0; i < activeTasks; i++) {
            if (taskControlBlocks[i].taskState == TaskState::READY) {
                if ((int)taskControlBlocks[i].priority > highestPriorityValue) {
                    highestPriorityValue = taskControlBlocks[i].priority;
                    highestPriorityTaskIndex = i;
                }
            }
        }

        //if a ready task was found, execute it immediately
        if (highestPriorityTaskIndex != -1) {
            TaskControlBlock* task = &taskControlBlocks[highestPriorityTaskIndex];

            task->taskState = TaskState::RUNNING;

            task->taskCodeAddress();

            //update execution tracking and reset state to wait for next SysTick interval
            task->lastRunTime = globalSystemTicks;
            task->taskState = TaskState::BLOCKED;
        }
        else {
            //idle State: If nothing is ready, perform a brief no-op
            //to keep the CPU stable and clear for hardware interrupts
            __asm__ volatile("nop");
        }
    }
}

//REMOVE THESE AFTER CONFIRMING TESTING
void taskFast() { print_str("⚡ Fast 100ms Task Running\n"); }
void taskMedium() { print_str("   🐢 Medium 500ms Task Running\n"); }
void taskSlow() { print_str("      🛑 Slow 1000ms Task Running\n"); }

int main() {

    // QEMU hardware clock init lines we just added
    CPU_SysTick[1] = 15999UL;
    CPU_SysTick[2] = 0UL;
    CPU_SysTick[0] = 7UL;

    __asm__ volatile("cpsie i" : : : "memory");

    // Register your new dummy tasks
    initializeNewTask(taskFast, 100, "FastTask");
    taskControlBlocks[activeTasks - 1].priority = 3;  // Low priority

    initializeNewTask(taskMedium, 500, "MedTask");
    taskControlBlocks[activeTasks - 1].priority = 6;  // Mid priority

    initializeNewTask(taskSlow, 1000, "SlowTask");
    taskControlBlocks[activeTasks - 1].priority = 12; // High priority

    print_str("🎯 Booting RTOS Scheduler Kernel...\n\n");

    // Your exact dispatcher loop call
    executeTaskLoop();
}

//Task code: Instead of returning a value, tasks pass data to each other using global/file-scope thread-safe buffers, lock-free queues, or shared state structs.

//Interrupt zone:
void readIMUDataRegisters(){} //reads imu and does calculations
void stateEstimation(){} //low level pass filters (e.g. kalman filter)
void flightLoop(){} //pid and mixer calculations
void crsfParsing(){} //parses incoming radio signals
void radioLinkFailSafe(){/*instant disarm if radio link drops for more than 100 ms*/}
void dShotGeneration(){} //cleans up the DMA buffers for next cycle to esc

//Scheduled zone: Must use while (true) loops
//high priority
[[noreturn]] void lowLevelFailSafe(){ while (true){} } //for non emergencies e.g. low battery or gps ping low

//medium priority(){}
[[noreturn]] void telemetryTX(){ while (true){} } //logs telemetry on the radio
[[noreturn]] void osdUpdate(){ while (true){} } //updates osd values for the telemetry in the video footage
[[noreturn]] void gpsParser(){ while (true){} } //updates location and transmits

//low priority
[[noreturn]] void blackboxLogging(){ while (true){} } //most likely remove as we have no onboard storage
[[noreturn]] void updatePeripherals(){ while (true){} } //controls leds, beeper, and smartaudio for the vtx
[[noreturn]] void usbCLI(){ while (true){} } //handles connections via the usbc port e.g. cli adjustments, pid tuning, and config flashing etc.

//iswg (Independent Watchdog)
[[noreturn]] void iswg(){while (true){} } // operates in a different part of the mcu, completely diff from the actual thread