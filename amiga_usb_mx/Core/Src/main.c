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

/* External storage classes */
extern USBD_HandleTypeDef hUsbDeviceFS;
extern host_driver_t usbdriver;
extern void bootloader_jump(void);

/* Internal functions */
static UART_HandleTypeDef huart2;
static int debuglevel = DBG_VERBOSE;

/* Private function prototypes -----------------------------------------------*/
//static void SystemClock_Config(void);
//static void MX_GPIO_Init(void);
//static void MX_USART2_UART_Init(void);

#if defined(__ATARI__)
#	define KEYBOARD_INTERFACE "ATARI XL/XE"
#elif defined(__AMIGA__)
#	define KEYBOARD_INTERFACE "AMIGA COMMODORE COMPUTERS"
#else
#	error "NO PLATFORM DEFINED"
#endif

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
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
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
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

	/*Configure GPIO pin : LEDS MUST BE IN THE SAME PORT! */
	GPIO_InitStruct.Pin = 
		LED_CAPS_LOCK_Pin |
		LED_NUM_LOCK_Pin |
		LED_SCROLL_LOCK_Pin;

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

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

static void banner(void)
{
	DBG_I("STM32 KEYBOARD CORE INTERFACE BOARD for " KEYBOARD_INTERFACE "\r\n");
	DBG_I("(C) RetroBit Lab 2022 written by Gianluca Renzi <icjtqr@gmail.com>\r\n");
}

__weak void bootloader_jump(void)
{
	DBG_E("Bootloader Jump Not Implemented Yet!\n");
	while (1);
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
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

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	_write_ready(SYSCALL_NOTREADY); // printf is not functional here

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();

	MX_USART2_UART_Init();
	_write_ready(SYSCALL_READY); // printf is functional from now on...

	banner();

	/* Initialize all pin to talk with Amiga Hardware and keeps the Amiga
	 * in a RESET state until the system is ready to communicate with
	 * all peripherals...
	 */
	amiga_gpio_init();

	/* This is for handshaking with the motherboard (if it's present)
	 * and ready.
	 */
	amiga_protocol_init();

	MX_USB_DEVICE_Init();

	host_set_driver(&usbdriver);

	DBG_V("KEYBOARD TYPE: " KEYBOARD_INTERFACE "\r\n");

	keyboard_init();

	/* Now it's time to activate Amiga hardware */

	for(;;)
	{
		keyboard_task();
		/* Check for any RST assert from Amiga hardware */
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
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
