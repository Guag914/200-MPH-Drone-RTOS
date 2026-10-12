//
// rtos.h by Akshay Gillett on 7/9/26.
//

#ifndef DRONE_RTOS_H
#define DRONE_RTOS_H

#include <stdint.h>

//this section is completely HIDDEN from the C compiler (main.c)
#ifdef __cplusplus

#define MAX_TASKS 20
#define DEFAULT_STACK 512
#define SYSTICK_BASE_ADDRESS (0xE000E010UL)

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
    uint32_t* taskStack; //change to ptr for dynamic mem
    uint32_t stackSizeWords; //size

    void (*taskCodeAddress)();
    // alignas(8) uint32_t taskStack[MAX_SYSTEM_MEM];

    uint32_t* topOfStack;
    uint32_t executionPeriod;
    uint32_t lastRunTime;
    const char* taskName;
    uint8_t priority;
    TaskState taskState;

    TaskType taskType;
    uint32_t excReturn;
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
// extern uint32_t** currTaskAddress;
// extern uint32_t** nextTaskAddress;
//
// extern uint32_t* currExcReturnAddress;
// extern uint32_t* nextExcReturnAddress;
//
// extern int preemptDepth;

extern int currTaskIndex;
extern int nextTaskIndex;

extern volatile uint32_t nextExcReturn;

extern int currLoopIndex;

extern volatile uint32_t globalSystemTicks;
extern void yieldCurrentTask(void);

//hide from c compiler but public
#ifdef __cplusplus
} // Close extern "C" briefly to allow C++ scope
extern TaskControlBlock taskControlBlocks[MAX_TASKS];
extern int activeTasks;
extern void decideNextInterruptTask(TaskControlBlock* interruptTask);
extern void decideNextScheduledTask();
extern bool protectStackGuard(uint32_t guardBaseAddr);

extern TaskControlBlock* findTaskByName(const char *textName);

// extern "C" bool switchPending;

extern volatile bool rtosStarted;
extern "C" { // Re-open extern "C"
#endif

#ifdef __cplusplus
}
#endif

#endif // DRONE_RTOS_H