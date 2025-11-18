/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : FreeRTOS initialization and task definitions
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "keyboard.h"
#include "keyboard_queues.h"
#include "usb_keyboard_task.h"
#include "led_task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for keyboardTask */
osThreadId_t keyboardTaskHandle;
const osThreadAttr_t keyboardTask_attributes = {
  .name = "keyboardTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for usbKeyboardTask */
osThreadId_t usbKeyboardTaskHandle;
const osThreadAttr_t usbKeyboardTask_attributes = {
  .name = "usbKeyboardTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for ledManagerTask */
osThreadId_t ledManagerTaskHandle;
const osThreadAttr_t ledManagerTask_attributes = {
  .name = "ledManagerTask",
  .stack_size = 1024 * 2,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void keyboardTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* Initialize keyboard queues */
  keyboard_queues_init();
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of keyboardTask */
  keyboardTaskHandle = osThreadNew(keyboardTask, NULL, &keyboardTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* Create USB keyboard task */
  usbKeyboardTaskHandle = osThreadNew(usbKeyboardTask, NULL, &usbKeyboardTask_attributes);

  /* Create LED manager task */
  ledManagerTaskHandle = osThreadNew(ledManagerTask, NULL, &ledManagerTask_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_keyboardTask */
/**
  * @brief  Keyboard task that calls keyboard_task() continuously
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_keyboardTask */
void keyboardTask(void *argument)
{
  /* USER CODE BEGIN keyboardTask */
  /* Infinite loop */
  for(;;)
  {
    keyboard_task();
    /* No delay needed - keyboard_task() handles its own timing */
  }
  /* USER CODE END keyboardTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
