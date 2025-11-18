#ifndef __SERIAL_TASK_H_INCLUDED__
#define __SERIAL_TASK_H_INCLUDED__

#include "cmsis_os2.h"
#include <stdint.h>
#include <stdbool.h>

/* Maximum message size for serial output */
#define SERIAL_MSG_MAX_SIZE    256
#define SERIAL_QUEUE_SIZE      10

/* Serial message structure */
typedef struct {
    char message[SERIAL_MSG_MAX_SIZE];
    uint16_t length;
} serial_message_t;

/* Queue handle for serial messages */
extern osMessageQueueId_t serialQueue;

/**
 * @brief Initialize serial queue
 */
void serial_queue_init(void);

/**
 * @brief Serial task entry point
 * @param argument: Not used
 *
 * This task:
 * - Receives messages from serialQueue
 * - Transmits them via UART without interruption
 */
void serialTask(void *argument);

/**
 * @brief Send a string to serial output
 * @param data: Pointer to data buffer
 * @param len: Length of data
 * @return Number of bytes queued (or sent directly if scheduler not started)
 *
 * If FreeRTOS scheduler is not running, writes directly to UART.
 * Otherwise, queues the message for the serial task.
 */
int serial_write(const char *data, int len);

/**
 * @brief Check if FreeRTOS scheduler is running
 * @return true if scheduler is running, false otherwise
 */
bool serial_is_scheduler_running(void);

#endif /* __SERIAL_TASK_H_INCLUDED__ */
