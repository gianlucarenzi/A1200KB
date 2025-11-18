#include "keyboard_queues.h"

/* Queue handles */
QueueHandle_t keyEventQueue = NULL;
QueueHandle_t ledCommandQueue = NULL;

/**
 * @brief Initialize all keyboard-related queues
 */
void keyboard_queues_init(void)
{
    /* Create queue for key events from keyboard scanner to USB task */
    keyEventQueue = xQueueCreate(KEY_EVENT_QUEUE_SIZE, sizeof(keyevent_t));

    /* Create queue for LED commands from USB task to LED manager */
    ledCommandQueue = xQueueCreate(LED_COMMAND_QUEUE_SIZE, sizeof(led_command_t));
}
