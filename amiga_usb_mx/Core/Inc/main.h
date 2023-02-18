/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* Private defines -----------------------------------------------------------*/

#define ROW0_Pin GPIO_PIN_0
#define ROW0_GPIO_Port GPIOB
#define ROW1_Pin GPIO_PIN_1
#define ROW1_GPIO_Port GPIOB
#define ROW2_Pin GPIO_PIN_2
#define ROW2_GPIO_Port GPIOB
#define ROW3_Pin GPIO_PIN_3
#define ROW3_GPIO_Port GPIOB
#define ROW4_Pin GPIO_PIN_4
#define ROW4_GPIO_Port GPIOB
#define ROW5_Pin GPIO_PIN_5
#define ROW5_GPIO_Port GPIOB
#define ROW6_Pin GPIO_PIN_6
#define ROW6_GPIO_Port GPIOB
#define ROW7_Pin GPIO_PIN_7
#define ROW7_GPIO_Port GPIOB
#define ROW8_Pin GPIO_PIN_8
#define ROW8_GPIO_Port GPIOB
#define ROW9_Pin GPIO_PIN_9
#define ROW9_GPIO_Port GPIOB
#define ROW10_Pin GPIO_PIN_10
#define ROW10_GPIO_Port GPIOB
#define ROW11_Pin GPIO_PIN_12
#define ROW11_GPIO_Port GPIOB
#define ROW12_Pin GPIO_PIN_13
#define ROW12_GPIO_Port GPIOB

#define COL0_Pin GPIO_PIN_0
#define COL0_GPIO_Port GPIOC
#define COL1_Pin GPIO_PIN_1
#define COL1_GPIO_Port GPIOC
#define COL2_Pin GPIO_PIN_2
#define COL2_GPIO_Port GPIOC
#define COL3_Pin GPIO_PIN_3
#define COL3_GPIO_Port GPIOC
#define COL4_Pin GPIO_PIN_4
#define COL4_GPIO_Port GPIOC
#define COL5_Pin GPIO_PIN_5
#define COL5_GPIO_Port GPIOC
#define COL6_Pin GPIO_PIN_6
#define COL6_GPIO_Port GPIOC
#define COL7_Pin GPIO_PIN_7
#define COL7_GPIO_Port GPIOC
#define COL8_Pin GPIO_PIN_8
#define COL8_GPIO_Port GPIOC
#define COL9_Pin GPIO_PIN_9
#define COL9_GPIO_Port GPIOC
#define COL10_Pin GPIO_PIN_10
#define COL10_GPIO_Port GPIOC
#define COL11_Pin GPIO_PIN_11
#define COL11_GPIO_Port GPIOC
#define COL12_Pin GPIO_PIN_12
#define COL12_GPIO_Port GPIOC
#define COL13_Pin GPIO_PIN_13
#define COL13_GPIO_Port GPIOC
#define COL14_Pin GPIO_PIN_14
#define COL14_GPIO_Port GPIOC
#define COL15_Pin GPIO_PIN_15
#define COL15_GPIO_Port GPIOC

#define DEBUG_TX_Pin GPIO_PIN_2
#define DEBUG_TX_GPIO_Port GPIOA
#define DEBUG_RX_Pin GPIO_PIN_3
#define DEBUG_RX_GPIO_Port GPIOA

#define KB_DAT_Pin GPIO_PIN_5
#define KB_DAT_GPIO_Port GPIOA
#define KB_CLK_Pin GPIO_PIN_6
#define KB_CLK_GPIO_Port GPIOA
#define KB_RST_Pin GPIO_PIN_7
#define KB_RST_GPIO_Port GPIOA
#define KB_AMIGA_GPIO_Port GPIOA

/**
 * The keyboard can have up to 3 leds:
 * D0: NUM lock
 * D1: CAPS lock
 * D2: SCROLL lock
 **/
#define LED_CAPS_LOCK_Pin GPIO_PIN_8
#define LED_CAPS_LOCK_GPIO_Port GPIOA
#define LED_NUM_LOCK_Pin GPIO_PIN_9
#define LED_NUM_LOCK_GPIO_Port GPIOA
#define LED_SCROLL_LOCK_Pin GPIO_PIN_10
#define LED_SCROLL_LOCK_GPIO_Port GPIOA

#define LED_POWER_Pin GPIO_PIN_0
#define LED_POWER_GPIO_Port GPIOA

/* All leds must be defined in the same port! */
#define LEDS_GPIO_Port GPIOA

extern void led_toggle(void);

/* All leds are active high */
#define LED_CAPS_LOCK_ON()   do { HAL_GPIO_WritePin(LED_CAPS_LOCK_GPIO_Port, LED_CAPS_LOCK_Pin, GPIO_PIN_SET); } while (0)
#define LED_CAPS_LOCK_OFF()    do { HAL_GPIO_WritePin(LED_CAPS_LOCK_GPIO_Port, LED_CAPS_LOCK_Pin, GPIO_PIN_RESET); } while (0)
#define LED_NUM_LOCK_ON()    do { HAL_GPIO_WritePin(LED_NUM_LOCK_GPIO_Port, LED_NUM_LOCK_Pin, GPIO_PIN_SET); } while (0)
#define LED_NUM_LOCK_OFF()     do { HAL_GPIO_WritePin(LED_NUM_LOCK_GPIO_Port, LED_NUM_LOCK_Pin, GPIO_PIN_RESET); } while (0)
#define LED_SCROLL_LOCK_ON() do { HAL_GPIO_WritePin(LED_SCROLL_LOCK_GPIO_Port, LED_SCROLL_LOCK_Pin, GPIO_PIN_SET); } while (0)
#define LED_SCROLL_LOCK_OFF()  do { HAL_GPIO_WritePin(LED_SCROLL_LOCK_GPIO_Port, LED_SCROLL_LOCK_Pin, GPIO_PIN_RESET); } while (0)
#define LED_POWER_ON()        do { HAL_GPIO_WritePin(LED_POWER_GPIO_Port, LED_POWER_Pin, GPIO_PIN_SET); } while (0)
#define LED_POWER_OFF()        do { HAL_GPIO_WritePin(LED_POWER_GPIO_Port, LED_POWER_Pin, GPIO_PIN_RESET); } while (0)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
