#include "user_tasks.h"
#include "../src/rtos/rtos.h"

/*====== BEGIN USER TASK ENTRY ======*/

/**
 * @brief An example of a standard, periodic cooperatively scheduled task.
 * * This task is managed by the main cooperative scheduler loop. It contains
 * an infinite loop that handles routine background processes.
 * * @note To prevent hogging the CPU, this task must cooperatively block itself
 * (by updating its taskState to BLOCKED and setting its executionPeriod) and
 * then call `yieldCurrentTask()` to hand control back to the scheduler.
 */

// The example code uses a uint32 variable to demonstrate the task saving ability.
static volatile uint32_t verificationCounter = 0;

[[noreturn]] void exampleScheduledTask() {
    while (true) {
        print_str("Count: ");
        print_uint32(verificationCounter++);
        print_str("\n");

        // Yield control back to the scheduler via PendSV
        yieldCurrentTask();
    }
}

/**
 * @brief An example of an asynchronous, preemptive interrupt task.
 * * This task is completely ignored by the standard periodic scheduler and remains
 * in a BLOCKED state until triggered by an external hardware event (e.g., an IMU
 * data-ready interrupt or incoming radio packet).
 * * @note Because this task preempts the background scheduler, it MUST call
 * `yieldCurrentTask()` when it finishes to restore the saved register
 * states and resume the background processes.
 */

[[noreturn]] void exampleInterruptTask() {
    while (true) {
        print_str("--- [INTERRUPT PREEMPTION START] ---\n");
        print_str("Captured Counter State: ");
        print_uint32(verificationCounter); //Even though the previous task was interrupted, it will restore from where it left off
        print_str("\n");

        for (volatile uint32_t i = 0; i < 15000000UL; i++) { // Breifly pause for visual testing
            __asm__ volatile("nop");
        }
        print_str("--- [INTERRUPT PREEMPTION END - RESTORING] ---\n");

        yieldCurrentTask();
    }
}


/*====== END USER TASK ENTRY ======*/

/**
 * @brief Registers user-defined tasks into the kernel's Task Control Block (TCB) registry.
 * * This function is called by the kernel during system startup. It allocates the
 * private stack spaces, initializes the register context, sets priority levels,
 * and sets the initial task states.
 * * @details taskControlBlocks[activeTasks - 1] is used to dynamically reference
 * the newly allocated task TCB immediately after initializeNewTask() runs.
 */

void registerUserTasks() {
    /*BEGIN USER TASK REGISTRATION*/

    // 1. Register the Scheduled Task (Time period set to 1000ms)
    initializeNewTask(exampleScheduledTask, 100, "ExampleScheduledTask");
    taskControlBlocks[activeTasks - 1].priority = 5;               ///< Medium priority background worker
    taskControlBlocks[activeTasks - 1].taskState = TaskState::READY; ///< Starts as READY to run on startup

    // 2. Register the Preemptive Interrupt Task (Bypasses execution periods)
    initializeNewTask(exampleInterruptTask, "ExampleInterruptTask");
    taskControlBlocks[activeTasks - 1].priority = 6;                 ///< Must be higher priority than cooperative tasks
    taskControlBlocks[activeTasks - 1].taskState = TaskState::BLOCKED; ///< Starts as BLOCKED until externally signaled

    /*END USER TASK REGISTRATION*/
}

/**
 * @brief Simulates an external hardware trigger or event controller and fires exactly every 1 ms.
 * * For an interrupt task to execute, an external driver or timer must manually
 * trigger it. This function evaluates asynchronous trigger conditions (e.g., clock limits,
 * hardware pins, or random events) and uses `decideNextInterruptTask()` to activate the task.
 * * @warning Be careful with hardcoded indices like `&taskControlBlocks[1]`. If you change the
 * registration order of tasks, index 1 might point to a different task, which could cause a system crash.
 * Remember that all indices are offset by one, meaning that index 1 will point to the first
 * task you registered, index 2 will point to the second task you registered etc.
 */


static uint32_t userSeed = 98765;
static uint32_t getUserRandom(int range) { // Used to find a random number
    userSeed = (1103515245 * userSeed + 12345) % 2147483648;
    return (userSeed % range) + 1;
}

void callUserInterruptTasks() {
    /*BEGIN INTERRUPT TASK CALLING LOGIC*/

    // Simulates an asynchronous hardware interrupt firing (approx. 1 in 1000 SysTicks)
    if (getUserRandom(1000) == 500) {
        // Change the index here to match the exact TCB position of your target Interrupt Task!
        decideNextInterruptTask(&taskControlBlocks[2]);
    }

    /*END INTERRUPT TASK CALLING LOGIC*/
}