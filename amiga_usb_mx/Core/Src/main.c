/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
#include <stdio.h>
#include <string.h>
#include "stm32f4xx_ll_gpio.h"
#include "usbd_hid.h"
#include "syscall.h"
#include "debug.h"
#include "main.h"
#include "usb_device.h"
#include "keyboard.h"
#include "host.h"
#include "config.h"
#include "amiga_protocol.h"
#include "FreeRTOS.h"
#include "task.h"
#include "freertos.h"

/* External storage classes */
extern USBD_HandleTypeDef hUsbDeviceFS;
extern host_driver_t usbdriver;

/* Internal functions */
UART_HandleTypeDef huart2;  /* Non-static to allow access from serial_task.c */
static int debuglevel = DBG_INFO;

#define KEYBOARD_INTERFACE "AMIGA COMMODORE COMPUTERS"
static const char *fwBuild = "v0.1rc BUILD: " __TIME__ "-" __DATE__;

static void MX_USART2_UART_Init(int baudrate)
{
	huart2.Instance = USART2;
	huart2.Init.BaudRate = baudrate;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
}

static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level. All leds are HIGH ACTIVE!*/
	HAL_GPIO_WritePin(LED_CAPS_LOCK_GPIO_Port, LED_CAPS_LOCK_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_NUM_LOCK_GPIO_Port, LED_NUM_LOCK_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_SCROLL_LOCK_GPIO_Port, LED_SCROLL_LOCK_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_ACT_GPIO_Port, LED_ACT_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : LEDS MUST BE IN THE SAME PORT! */
	GPIO_InitStruct.Pin = 
		LED_CAPS_LOCK_Pin |
		LED_NUM_LOCK_Pin |
		LED_SCROLL_LOCK_Pin |
		LED_ACT_Pin;

	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LEDS_GPIO_Port, &GPIO_InitStruct);

	/* The following pins are usable only on Amiga Keyboards. We are
	 * using them anyway, just to have a single firmware for more PCBs
	 * Write ONCE, Use EVERYWHERE */

	/*Configure GPIO pins : KB_RST_Pin KB_CLK_Pin KB_DAT_Pin */
	GPIO_InitStruct.Pin = KB_RST_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(KB_RST_GPIO_Port, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = KB_CLK_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(KB_CLK_GPIO_Port, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = KB_DAT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(KB_DAT_GPIO_Port, &GPIO_InitStruct);
}

void Error_Handler(void)
{
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
}

static void banner(void)
{
	printf("\r\n\r\n" ANSI_BLUE "TMK BASED KEYBOARD CORE INTERFACE BOARD for " KEYBOARD_INTERFACE ANSI_RESET "\r\n");
	printf(ANSI_YELLOW);
	printf("FWVER: %s", fwBuild);
	printf(ANSI_RESET "\r\n");
	printf("\r\n\n");
}

__weak void bootloader_jump(void)
{
	DBG_E("Bootloader Jump Not Implemented Yet!\n");
	while (1);
}

void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	*/
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
	/** Initializes the RCC Oscillators according to the specified parameters
	* in the RCC_OscInitTypeDef structure.
	*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 168;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	*/
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
							  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
}

int main(void)
{
	_write_ready(SYSCALL_NOTREADY, &huart2); // printf is not functional here

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();

	/* Initialize all pins to talk with Amiga Hardware and keeps the Amiga
	 * in a RESET state until the system is ready to communicate with
	 * all peripherals as early as possible...
	 */
	amiga_gpio_init();

	MX_USART2_UART_Init(115200);
	_write_ready(SYSCALL_READY, &huart2); // printf is functional from now on...

	banner();

	MX_USB_DEVICE_Init();

	host_set_driver(&usbdriver);

	DBG_V("KEYBOARD TYPE: " KEYBOARD_INTERFACE " CORE DRIVER RUNNING\r\n");

	keyboard_init();

	/* Now it's time to activate Amiga hardware */

	/* This is for handshaking with the motherboard (if it's present)
	 * and ready.
	 */
	amiga_protocol_init();

	LED_ACT_ON();

	/* Initialize FreeRTOS */
	MX_FREERTOS_Init();

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	while (1)
	{
	}
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
		ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	printf("FULL ASSERT: Wrong parametrers value: file %s on line %d\r\n", file, line);
	while (1)
		;
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
