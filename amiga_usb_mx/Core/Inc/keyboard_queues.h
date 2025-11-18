#ifndef __KEYBOARD_QUEUES_H_INCLUDED__
#define __KEYBOARD_QUEUES_H_INCLUDED__

#include "cmsis_os2.h"
#include "keyboard.h"
#include <stdint.h>

/* Queue sizes */
#define KEY_EVENT_QUEUE_SIZE    32
#define LED_COMMAND_QUEUE_SIZE  8

/* LED command structure */
typedef struct {
    uint8_t led_status;
} led_command_t;

/* Queue handles (declared extern, defined in keyboard_queues.c) */
extern osMessageQueueId_t keyEventQueue;
extern osMessageQueueId_t ledCommandQueue;

/* Queue initialization function */
void keyboard_queues_init(void);

#endif /* __KEYBOARD_QUEUES_H_INCLUDED__ */
