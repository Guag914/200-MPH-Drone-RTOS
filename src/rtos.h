#ifndef DRONE_RTOS_H
#define DRONE_RTOS_H

#include <stdint.h>

// This section is visible to BOTH main.c (C) and Main.cpp (C++)
#ifdef __cplusplus
extern "C" {
#endif

void start_drone_rtos(void);

#ifdef __cplusplus
}
#endif

// This section is completely HIDDEN from the C compiler (main.c)
// It protects C++ exclusive keywords from breaking the build
#ifdef __cplusplus

#define MAX_TASKS 12

enum class TaskState : uint8_t {
    BLOCKED = 0,
    READY   = 1,
    RUNNING = 2
};

struct TaskControlBlock {
    void (*taskCodeAddress)();
    uint32_t executionPeriod;
    uint32_t lastRunTime;
    const char* taskName;
    uint8_t priority;
    TaskState taskState;
};

bool initializeNewTask(void (*functionAddress)(), uint32_t timePeriod, const char* textName);
void executeTaskLoop(void);

#endif // __cplusplus

#endif // DRONE_RTOS_H