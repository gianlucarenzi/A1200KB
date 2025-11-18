#include "serial_task.h"
#include "syscall.h"
#include "cmsis_os2.h"
#include <string.h>
#include <stm32f4xx_hal.h>

/* Queue handle */
osMessageQueueId_t serialQueue = NULL;

/* External UART handle from main.c */
extern UART_HandleTypeDef huart2;

/**
 * @brief Initialize serial queue
 */
void serial_queue_init(void)
{
    serialQueue = osMessageQueueNew(SERIAL_QUEUE_SIZE, sizeof(serial_message_t), NULL);
}

/**
 * @brief Check if FreeRTOS scheduler is running
 */
bool serial_is_scheduler_running(void)
{
    return (osKernelGetState() == osKernelRunning);
}

/**
 * @brief Send a string to serial output
 * @param data: Pointer to data buffer
 * @param len: Length of data
 * @return Number of bytes queued or sent
 */
int serial_write(const char *data, int len)
{
    /* Check if scheduler is running */
    if (!serial_is_scheduler_running() || serialQueue == NULL)
    {
        /* Scheduler not started yet - write directly to UART */
        HAL_UART_Transmit(&huart2, (uint8_t*)data, len, 1000);
        return len;
    }
    else
    {
        /* Scheduler is running - queue the message */
        serial_message_t msg;

        /* Limit message size */
        if (len > SERIAL_MSG_MAX_SIZE - 1)
            len = SERIAL_MSG_MAX_SIZE - 1;

        memcpy(msg.message, data, len);
        msg.message[len] = '\0';
        msg.length = len;

        /* Try to send to queue (non-blocking) */
        if (osMessageQueuePut(serialQueue, &msg, 0, 0) == osOK)
        {
            return len;
        }
        else
        {
            /* Queue full - message lost */
            return 0;
        }
    }
}

/**
 * @brief Serial task - transmits queued messages via UART
 */
void serialTask(void *argument)
{
    serial_message_t msg;

    /* Infinite loop */
    for(;;)
    {
        /* Wait for message from queue (blocking) */
        if (osMessageQueueGet(serialQueue, &msg, NULL, osWaitForever) == osOK)
        {
            /* Transmit entire message atomically - no interruptions */
            HAL_UART_Transmit(&huart2, (uint8_t*)msg.message, msg.length, 1000);
        }
    }
}
