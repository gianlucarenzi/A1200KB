#include "led_task.h"
#include "keyboard_queues.h"
#include "keyboard.h"
#include "hook.h"
#include "debug.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

static int debuglevel = DBG_INFO;

/**
 * @brief LED manager task - controls hardware LEDs
 * @param argument: Not used
 */
void ledManagerTask(void *argument)
{
	led_command_t cmd;

	DBG_I("LED Manager Task started\r\n");

	/* Infinite loop */
	for(;;)
	{
		/* Wait for LED command (blocking wait) */
		if (xQueueReceive(ledCommandQueue, &cmd, portMAX_DELAY) == pdTRUE)
		{
			/* Update hardware LEDs via hook */
			hook_keyboard_leds_change(cmd.led_status);

			if (debug_keyboard) {
				DBG_I("LED hardware updated: %02X\r\n", cmd.led_status);
			}
		}
	}
}
