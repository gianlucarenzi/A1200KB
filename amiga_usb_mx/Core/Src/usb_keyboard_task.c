#include "usb_keyboard_task.h"
#include "keyboard_queues.h"
#include "keyboard.h"
#include "action.h"
#include "host.h"
#include "hook.h"
#include "debug.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

static int debuglevel = DBG_INFO;

/**
 * @brief USB keyboard task - processes key events and manages USB HID
 * @param argument: Not used
 */
void usbKeyboardTask(void *argument)
{
	keyevent_t event;
	static uint8_t led_status = 0;
	uint8_t current_led_status;

	DBG_I("USB Keyboard Task started\r\n");

	/* Infinite loop */
	for(;;)
	{
		/* Wait for key event from keyboard scanner (blocking wait) */
		if (xQueueReceive(keyEventQueue, &event, portMAX_DELAY) == pdTRUE)
		{
			/* Process the key event through action system */
			action_exec(event);

			/* Call hook for matrix changes (if not TICK event) */
			if (!IS_NOEVENT(event) && !(event.key.row == 255 && event.key.col == 255)) {
				hook_matrix_change(event);
			}

			/* Call keyboard loop hook */
			hook_keyboard_loop();

			/* Check for LED status changes from USB host */
			current_led_status = host_keyboard_leds();
			if (led_status != current_led_status) {
				led_status = current_led_status;

				if (debug_keyboard) {
					DBG_I("LED status changed: %02X\r\n", led_status);
				}

				/* Send LED command to LED manager task */
				if (ledCommandQueue != NULL) {
					led_command_t cmd = { .led_status = led_status };
					xQueueSend(ledCommandQueue, &cmd, 0);
				}
			}
		}
	}
}
