#ifndef __LED_TASK_H_INCLUDED__
#define __LED_TASK_H_INCLUDED__

#include "FreeRTOS.h"
#include "task.h"

/**
 * @brief LED manager task entry point
 * @param argument: Not used
 *
 * This task:
 * - Receives LED commands from ledCommandQueue
 * - Updates hardware LED state
 */
void ledManagerTask(void *argument);

#endif /* __LED_TASK_H_INCLUDED__ */
