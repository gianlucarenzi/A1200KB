#include "led_task.h"
#include "keyboard_queues.h"
#include "keyboard.h"
#include "hook.h"
#include "debug.h"
#include "cmsis_os2.h"

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
		if (osMessageQueueGet(ledCommandQueue, &cmd, NULL, osWaitForever) == osOK)
		{
			/* Update hardware LEDs via hook */
			hook_keyboard_leds_change(cmd.led_status);

			if (debug_keyboard) {
				DBG_I("LED hardware updated: %02X\r\n", cmd.led_status);
			}
		}
	}
}
