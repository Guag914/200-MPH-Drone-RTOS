#ifndef DRONE_RTOS_H
#define DRONE_RTOS_H

#include <stdint.h>

//this section is visible to BOTH core/src/main.c (C) and src/Main.cpp (C++)
#ifdef __cplusplus
extern "C" {
#endif

void start_drone_rtos(void);

//expose global pointers cleanly to the rest of the system
extern uint32_t** currTaskAddress;
extern uint32_t** nextTaskAddress;

#ifdef __cplusplus
}
#endif

//this section is completely HIDDEN from the C compiler (main.c)
#ifdef __cplusplus

#define MAX_TASKS 20

enum class TaskState : uint8_t {
    BLOCKED = 0,
    READY   = 1,
    RUNNING = 2
};

struct TaskControlBlock {
    void (*taskCodeAddress)();
    alignas(8) uint32_t taskStack[1024];
    uint32_t* topOfStack;
    uint32_t executionPeriod;
    uint32_t lastRunTime;
    const char* taskName;
    uint8_t priority;
    TaskState taskState;
};

bool initializeNewTask(void (*functionAddress)(), uint32_t timePeriod, const char* textName);
[[noreturn]] void executeTaskLoop();

#endif // __cplusplus

#endif // DRONE_RTOS_H