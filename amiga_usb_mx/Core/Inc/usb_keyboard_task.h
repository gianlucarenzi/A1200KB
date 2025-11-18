#ifndef __USB_KEYBOARD_TASK_H_INCLUDED__
#define __USB_KEYBOARD_TASK_H_INCLUDED__

#include "FreeRTOS.h"
#include "task.h"

/**
 * @brief USB keyboard task entry point
 * @param argument: Not used
 *
 * This task:
 * - Receives key events from keyEventQueue
 * - Processes actions via action_exec()
 * - Sends HID reports to USB host
 * - Monitors USB LED status and sends to ledCommandQueue
 */
void usbKeyboardTask(void *argument);

#endif /* __USB_KEYBOARD_TASK_H_INCLUDED__ */
