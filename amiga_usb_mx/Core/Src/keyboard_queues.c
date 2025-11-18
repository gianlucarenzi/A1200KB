#include "keyboard_queues.h"

/* Queue handles */
osMessageQueueId_t keyEventQueue = NULL;
osMessageQueueId_t ledCommandQueue = NULL;

/**
 * @brief Initialize all keyboard-related queues
 */
void keyboard_queues_init(void)
{
    /* Create queue for key events from keyboard scanner to USB task */
    keyEventQueue = osMessageQueueNew(KEY_EVENT_QUEUE_SIZE, sizeof(keyevent_t), NULL);

    /* Create queue for LED commands from USB task to LED manager */
    ledCommandQueue = osMessageQueueNew(LED_COMMAND_QUEUE_SIZE, sizeof(led_command_t), NULL);
}
