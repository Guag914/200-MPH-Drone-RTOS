#ifndef DRONE_RTOS_USER_TASKS_H
#define DRONE_RTOS_USER_TASKS_H

#pragma once

/*====== BEGIN USER TASK DECLARATION ======*/

/**
 * @brief Place your custom task function declarations here. 
 * Ensure your task function is defined with [[noreturn]] and contains an infinite loop.
 */

[[noreturn]] void exampleScheduledTask();
[[noreturn]] void exampleInterruptTask();


/*====== END USER TASK DECLARATION ======*/

/**
 * @brief Hook executed by the kernel during initialization to register all user sandbox tasks.
 */
extern void registerUserTasks();
extern void callUserInterruptTasks();

#endif //DRONE_RTOS_USER_TASKS_H