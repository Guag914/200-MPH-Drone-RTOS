#ifndef DRONE_RTOS_H
#define DRONE_RTOS_H

#include <stdint.h>

//this section is completely HIDDEN from the C compiler (main.c)
#ifdef __cplusplus

#define MAX_TASKS 13

enum class TaskState : uint8_t {
    BLOCKED = 0,
    READY   = 1,
    RUNNING = 2
};

enum class TaskType : uint8_t {
    SCHEDULED,
    INTERRUPT
};

struct TaskControlBlock {
    void (*taskCodeAddress)();
    alignas(8) uint32_t taskStack[2048];
    uint32_t* topOfStack;
    uint32_t executionPeriod;
    uint32_t lastRunTime;
    const char* taskName;
    uint8_t priority;
    TaskState taskState;

    TaskType taskType;
};

bool initializeNewTask(void (*functionAddress)(), uint32_t timePeriod, const char* textName);
bool initializeNewTask(void (*functionAddress)(), const char* textName);

[[noreturn]] void executeTaskLoop();

#endif // __cplusplus

//this section is visible to BOTH core/src/main.c (C) and src/Main.cpp (C++)
#ifdef __cplusplus
extern "C" {
#endif

void start_drone_rtos(void);
extern uint32_t** currTaskAddress;
extern uint32_t** nextTaskAddress;

extern volatile uint32_t globalSystemTicks;
extern void yieldCurrentTask(void);

//hide from c compiler but public
#ifdef __cplusplus
} // Close extern "C" briefly to allow C++ scope
extern TaskControlBlock taskControlBlocks[MAX_TASKS];
extern int activeTasks;
extern void decideNextInterruptTask(TaskControlBlock* interruptTask);

extern "C" bool switchPending;

extern volatile bool rtosStarted;
extern "C" { // Re-open extern "C"
#endif

#ifdef __cplusplus
}
#endif

#endif // DRONE_RTOS_H