#include <errno.h>
#include <sys/unistd.h> // STDOUT_FILENO, STDERR_FILENO
#include "syscall.h"
#include <stm32f4xx_hal.h> // per HAL_StatusTypeDef
#include "debug.h"
#include "serial_task.h"
#include "delay_us.h"
#include "cmsis_os2.h"

static t_syscall_status uart_initialize = SYSCALL_NOTREADY;
static UART_HandleTypeDef *uart = NULL;
static int debuglevel = DBG_ERROR;

void _write_ready(t_syscall_status rdy, UART_HandleTypeDef *ptr)
{
	if (ptr != NULL)
	{
		uart_initialize = rdy;
		uart = ptr;
	}
}

int _write(int file, char *data, int len)
{
	if ((file != STDOUT_FILENO) && (file != STDERR_FILENO))
	{
		errno = EBADF;
		return -1;
	}

	if (uart_initialize == SYSCALL_READY)
	{
		/* Use serial task queue if available, otherwise direct UART write */
		return serial_write(data, len);
	}

	return 0;
}

static uint32_t timertick_start_ms = 0;
void timer_start(void)
{
	// When timer starts get the realtime system tick
	timertick_start_ms = HAL_GetTick();
}

/**
 * @brief Millisecond delay - uses osDelay if scheduler running, HAL_Delay otherwise
 * @param millis: Delay in milliseconds
 */
void mdelay(uint32_t millis)
{
	if (osKernelGetState() == osKernelRunning)
	{
		osDelay(millis);
	}
	else
	{
		HAL_Delay(millis);
	}
}
