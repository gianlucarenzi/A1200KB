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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_hid.h"

// DWT_Delay_Init initializes the DWT_CYCCNT register for accurate delays.
__attribute__((always_inline)) static inline void DWT_Delay_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // Enable DWT trace
    DWT->CYCCNT = 0; // Reset DWT cycle counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; // Enable DWT cycle counter
}

// _usdelay provides a microsecond delay using the DWT cycle counter.
__attribute__((always_inline)) static inline void _usdelay(uint32_t us)
{
    // Ensure DWT is initialized once before first use.
    // This is a simple check; a more robust solution might use a flag.
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
        DWT_Delay_Init();
    }

    uint32_t cycles = (SystemCoreClock / 1000000U) * us;
    uint32_t start_cycles = DWT->CYCCNT;
    while((DWT->CYCCNT - start_cycles) < cycles);
}
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// --- Definizioni per la Matrice della Tastiera ---
#define MATRIX_ROWS 6
#define MATRIX_COLS 22

// Stato dei tasti
typedef enum {
    KEY_RELEASED = 0,
    KEY_PRESSED = 1
} KeyState;

// Struttura per gli eventi dei tasti da inviare via coda
typedef struct {
    uint8_t scancode;
    KeyState state;
} KeyEvent_t;

// --- Definizioni per il Report HID ---
#define KEY_MOD_LCTRL  0x01
#define KEY_MOD_LSHIFT 0x02
#define KEY_MOD_LALT   0x04
#define KEY_MOD_LGUI   0x08
#define KEY_MOD_RCTRL  0x10
#define KEY_MOD_RSHIFT 0x20
#define KEY_MOD_RALT   0x40
#define KEY_MOD_RGUI   0x80

// --- Definizioni Scancode Modificatori HID ---
#define HID_SCANCODE_MOD_LCTRL  0xE0
#define HID_SCANCODE_MOD_LSHIFT 0xE1
#define HID_SCANCODE_MOD_LALT   0xE2
#define HID_SCANCODE_MOD_LGUI   0xE3
#define HID_SCANCODE_MOD_RCTRL  0xE4
#define HID_SCANCODE_MOD_RSHIFT 0xE5
#define HID_SCANCODE_MOD_RALT   0xE6
#define HID_SCANCODE_MOD_RGUI   0xE7

typedef struct {
    uint8_t modifiers;
    uint8_t reserved;
    uint8_t keys[6];
} HID_KeyboardReport_t;

// --- Nuove Definizioni per la Gestione LED ---
typedef enum {
    LED_NUM_LOCK = 0,
    LED_CAPS_LOCK = 1,
    LED_SCROLL_LOCK = 2
} LedType_t;

typedef enum {
    LED_OFF = 0,
    LED_ON = 1
} LedState_t;

typedef struct {
    LedType_t  led;
    LedState_t state;
} LedCommand_t;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

extern USBD_HandleTypeDef hUsbDeviceFS;



/* Logging variables */

bool isFreeRTOSStarted = false;

QueueHandle_t xLogQueue;



/* Definitions for ScannerTask */

osThreadId_t scannerTaskHandle;

const osThreadAttr_t scannerTask_attributes = {

  .name = "ScannerTask",

  .stack_size = 256 * 4,

  .priority = (osPriority_t) osPriorityNormal,

};

/* Definitions for UsbHidTask */

osThreadId_t usbHidTaskHandle;

const osThreadAttr_t usbHidTask_attributes = {

  .name = "UsbHidTask",

  .stack_size = 256 * 4,

  .priority = (osPriority_t) osPriorityAboveNormal,

};

/* Definitions for ledManagerTask */

osThreadId_t ledManagerTaskHandle;

const osThreadAttr_t ledManagerTask_attributes = {

  .name = "ledManagerTask",

  .stack_size = 128 * 4,

  .priority = (osPriority_t) osPriorityLow,

};

/* Definitions for logTask */

osThreadId_t logTaskHandle; // Correct declaration

const osThreadAttr_t logTask_attributes = {

  .name = "logTask",

  .stack_size = 256 * 4, // Adjust stack size if needed for UART operations

  .priority = (osPriority_t) osPriorityLow,

};

/* Definitions for keyEventQueue */

osMessageQueueId_t keyEventQueueHandle;

const osMessageQueueAttr_t keyEventQueue_attributes = {

  .name = "keyEventQueue"

};

/* Definitions for ledQueue */

osMessageQueueId_t ledQueueHandle;

const osMessageQueueAttr_t ledQueue_attributes = {

  .name = "ledQueue"

};





// --- Pinout della Matrice ---

GPIO_TypeDef* row_ports[MATRIX_ROWS] = {

    ROW0_GPIO_Port, ROW1_GPIO_Port, ROW2_GPIO_Port, ROW3_GPIO_Port, ROW4_GPIO_Port, ROW5_GPIO_Port

};

uint16_t row_pins[MATRIX_ROWS] = {

    ROW0_Pin, ROW1_Pin, ROW2_Pin, ROW3_Pin, ROW4_Pin, ROW5_Pin

};

GPIO_TypeDef* col_ports[MATRIX_COLS] = {

    COL0_GPIO_Port, COL1_GPIO_Port, COL2_GPIO_Port, COL3_GPIO_Port, COL4_GPIO_Port, COL5_GPIO_Port,

    COL6_GPIO_Port, COL7_GPIO_Port, COL8_GPIO_Port, COL9_GPIO_Port, COL10_GPIO_Port, COL11_GPIO_Port,

    COL12_GPIO_Port, COL13_GPIO_Port, COL14_GPIO_Port, COL15_GPIO_Port, COL16_GPIO_Port, COL17_GPIO_Port,

    COL18_GPIO_Port, COL19_GPIO_Port, COL20_GPIO_Port, COL21_GPIO_Port

};

uint16_t col_pins[MATRIX_COLS] = {

    COL0_Pin, COL1_Pin, COL2_Pin, COL3_Pin, COL4_Pin, COL5_Pin,

    COL6_Pin, COL7_Pin, COL8_Pin, COL9_Pin, COL10_Pin, COL11_Pin,

    COL12_Pin, COL13_Pin, COL14_Pin, COL15_Pin, COL16_Pin, COL17_Pin,

    COL18_Pin, COL19_Pin, COL20_Pin, COL21_Pin

};



// --- Look-Up Table (LUT) Scancode ---

const uint8_t scancode_lut[MATRIX_ROWS][MATRIX_COLS] = {

//  C0    C1    C2    C3    C4    C5    C6    C7    C8    C9   C10   C11   C12   C13   C14   C15   C16   C17   C18   C19   C20   C21
// R0
  {0x29, 0x00, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F, 0x40, 0x41, 0x42, 0x43, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
// R1
  {0x35, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x2D, 0x2E, 0x31, 0x2A, 0x4C, 0x00, 0x75, 0x53, 0x47, 0x54, 0x55 },
// R2
  {0x2B, 0x14, 0x1A, 0x08, 0x15, 0x17, 0x1C, 0x18, 0x0C, 0x12, 0x13, 0x2F, 0x30, 0x00, 0x28, 0x00, 0x00, 0x00, 0x5F, 0x60, 0x61, 0x56 },
// R3
  {0xE0, 0x39, 0x04, 0x16, 0x07, 0x09, 0x0A, 0x0B, 0x0D, 0x0E, 0x0F, 0x33, 0x34, 0x87, 0x00, 0x00, 0x52, 0x00, 0x5C, 0x5D, 0x5E, 0x57 },
// R4
  {0xE1, 0x88, 0x1D, 0x1B, 0x06, 0x19, 0x05, 0x11, 0x10, 0x36, 0x37, 0x38, 0x00, 0x00, 0xE5, 0x50, 0x51, 0x4F, 0x59, 0x5A, 0x5B, 0x58 },
// R5
  {0x00, 0xE2, 0xE3, 0x89, 0x00, 0x00, 0x00, 0x00, 0x2C, 0x00, 0x00, 0x00, 0x8A, 0xE7, 0xE6, 0x00, 0x00, 0x00, 0x62, 0x00, 0x63, 0x00 },
};


void logTask(void *argument) {

    char received_string[LOG_MESSAGE_MAX_LEN];

    for (;;) {

        // Wait indefinitely for a log message to arrive

        if (xQueueReceive(xLogQueue, received_string, portMAX_DELAY) == pdPASS) {

            // Transmit the received log message over UART2

            HAL_UART_Transmit(&huart2, (uint8_t *)received_string, strlen(received_string), HAL_MAX_DELAY);

        }

    }

}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void ScannerTask(void *argument);
void UsbHidTask(void *argument);
void ledManager(void *argument);
void USBD_HID_SetReport_Callback(uint8_t *report, uint16_t len);

/* USER CODE BEGIN PFP */
void logTask(void *argument);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  keyEventQueueHandle = osMessageQueueNew (16, sizeof(KeyEvent_t), &keyEventQueue_attributes);
  ledQueueHandle = osMessageQueueNew(4, sizeof(LedCommand_t), &ledQueue_attributes);
  xLogQueue = xQueueCreate(LOG_QUEUE_LENGTH, LOG_MESSAGE_MAX_LEN); // Create log queue

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  scannerTaskHandle = osThreadNew(ScannerTask, NULL, &scannerTask_attributes);
  usbHidTaskHandle = osThreadNew(UsbHidTask, NULL, &usbHidTask_attributes);
  ledManagerTaskHandle = osThreadNew(ledManager, NULL, &ledManagerTask_attributes);
  logTaskHandle = osThreadNew(logTask, NULL, &logTask_attributes); // Create log task

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  // Set FreeRTOS started flag
  isFreeRTOSStarted = true;

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_ACT_Pin|LED_POWER_Pin|KB_DAT_Pin|KB_CLK_Pin
                          |KB_RST_Pin|LED_CAPS_LOCK_Pin|LED_NUM_LOCK_Pin|LED_SCROLL_LOCK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ROW0_Pin|ROW1_Pin|ROW2_Pin|ROW3_Pin
                          |ROW4_Pin|ROW5_Pin, GPIO_PIN_SET); // Set rows high by default

  /*Configure GPIO pins : COL13_Pin COL14_Pin COL15_Pin COL0_Pin
                           BOOT_MODE_Pin COL2_Pin COL3_Pin COL4_Pin
                           COL5_Pin COL6_Pin COL7_Pin COL8_Pin
                           COL9_Pin COL10_Pin COL11_Pin COL12_Pin */
  GPIO_InitStruct.Pin = COL13_Pin|COL14_Pin|COL15_Pin|COL0_Pin
                          |BOOT_MODE_Pin|COL2_Pin|COL3_Pin|COL4_Pin
                          |COL5_Pin|COL6_Pin|COL7_Pin|COL8_Pin
                          |COL9_Pin|COL10_Pin|COL11_Pin|COL12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_ACT_Pin LED_POWER_Pin KB_DAT_Pin KB_CLK_Pin
                           KB_RST_Pin LED_CAPS_LOCK_Pin LED_NUM_LOCK_Pin LED_SCROLL_LOCK_Pin */
  GPIO_InitStruct.Pin = LED_ACT_Pin|LED_POWER_Pin|KB_DAT_Pin|KB_CLK_Pin
                          |KB_RST_Pin|LED_CAPS_LOCK_Pin|LED_NUM_LOCK_Pin|LED_SCROLL_LOCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ROW0_Pin ROW1_Pin ROW2_Pin ROW3_Pin
                           ROW4_Pin ROW5_Pin */
  GPIO_InitStruct.Pin = ROW0_Pin|ROW1_Pin|ROW2_Pin|ROW3_Pin
                          |ROW4_Pin|ROW5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; // High speed for faster scanning
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : COL20_Pin COL21_Pin COL16_Pin COL17_Pin
                           COL18_Pin COL19_Pin */
  GPIO_InitStruct.Pin = COL20_Pin|COL21_Pin|COL16_Pin|COL17_Pin
                          |COL18_Pin|COL19_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : COL1_Pin */
  GPIO_InitStruct.Pin = COL1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(COL1_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void ScannerTask(void *argument) {
    DEBUG_PRINT(1, SCANNER_TASK_DEBUG_LEVEL, "ScannerTask: Entered.\r\n");
    KeyState matrix_state[MATRIX_ROWS][MATRIX_COLS] = {0};
    KeyState matrix_state_prev[MATRIX_ROWS][MATRIX_COLS] = {0};

    uint32_t last_scan_time = osKernelGetTickCount();

    for (;;) {
        // Scansiona la matrice
        for (int r = 0; r < MATRIX_ROWS; r++) {
            HAL_GPIO_WritePin(row_ports[r], row_pins[r], GPIO_PIN_RESET);
            _usdelay(1); // Delay ~1us
            for (int c = 0; c < MATRIX_COLS; c++) {
                if (HAL_GPIO_ReadPin(col_ports[c], col_pins[c]) == GPIO_PIN_RESET) {
                    matrix_state[r][c] = KEY_PRESSED;
                } else {
                    matrix_state[r][c] = KEY_RELEASED;
                }
            }
            HAL_GPIO_WritePin(row_ports[r], row_pins[r], GPIO_PIN_SET);
        }

        // Debouncing e rilevamento cambiamenti
        for (int r = 0; r < MATRIX_ROWS; r++) {
            for (int c = 0; c < MATRIX_COLS; c++) {
                if (matrix_state[r][c] != matrix_state_prev[r][c]) {
                    osDelay(20);
                    HAL_GPIO_WritePin(row_ports[r], row_pins[r], GPIO_PIN_RESET);
                    _usdelay(1); // Delay
                    KeyState confirmed_state = (HAL_GPIO_ReadPin(col_ports[c], col_pins[c]) == GPIO_PIN_RESET) ? KEY_PRESSED : KEY_RELEASED;
                    HAL_GPIO_WritePin(row_ports[r], row_pins[r], GPIO_PIN_SET);

                    if (confirmed_state == matrix_state[r][c]) {
                        matrix_state_prev[r][c] = matrix_state[r][c];
                        if (scancode_lut[r][c] != 0x00) {
                            KeyEvent_t event = { .scancode = scancode_lut[r][c], .state = matrix_state[r][c] };
                            osMessageQueuePut(keyEventQueueHandle, &event, 0U, osWaitForever);
                            DEBUG_PRINT(1, SCANNER_TASK_DEBUG_LEVEL, "ScannerTask: Key R%d C%d (0x%02X) %s\r\n", r, c, event.scancode, (event.state == KEY_PRESSED ? "PRESSED" : "RELEASED"));
                        }
                    }
                }
            }
        }
        last_scan_time += 5;
        osDelay(last_scan_time - osKernelGetTickCount());
        DEBUG_PRINT(2, SCANNER_TASK_DEBUG_LEVEL, "ScannerTask: Scanning loop iteration.\r\n");
    }
}

void UsbHidTask(void *argument) {
    DEBUG_PRINT(1, USBHID_TASK_DEBUG_LEVEL, "UsbHidTask: Entered.\r\n");
    KeyEvent_t received_event;
    HID_KeyboardReport_t hid_report = {0};

    for (;;) {
        osStatus_t status = osMessageQueueGet(keyEventQueueHandle, &received_event, NULL, osWaitForever);
        if (status == osOK) {
            DEBUG_PRINT(2, USBHID_TASK_DEBUG_LEVEL, "UsbHidTask: Received event scancode=0x%02X, state=%s\r\n", received_event.scancode, (received_event.state == KEY_PRESSED ? "PRESSED" : "RELEASED"));
            uint8_t scancode = received_event.scancode;
            if (scancode >= HID_SCANCODE_MOD_LCTRL && scancode <= HID_SCANCODE_MOD_RGUI) {
                if (received_event.state == KEY_PRESSED) {
                    hid_report.modifiers |= (1 << (scancode - HID_SCANCODE_MOD_LCTRL));
                    DEBUG_PRINT(2, USBHID_TASK_DEBUG_LEVEL, "UsbHidTask: Modifier 0x%02X PRESSED, new modifiers=0x%02X\r\n", scancode, hid_report.modifiers);
                } else {
                    hid_report.modifiers &= ~(1 << (scancode - HID_SCANCODE_MOD_LCTRL));
                    DEBUG_PRINT(2, USBHID_TASK_DEBUG_LEVEL, "UsbHidTask: Modifier 0x%02X RELEASED, new modifiers=0x%02X\r\n", scancode, hid_report.modifiers);
                }
            } else {
                if (received_event.state == KEY_PRESSED) {
                    for (int i = 0; i < 6; i++) {
                        if (hid_report.keys[i] == 0x00) {
                            hid_report.keys[i] = scancode;
                            DEBUG_PRINT(2, USBHID_TASK_DEBUG_LEVEL, "UsbHidTask: Key 0x%02X PRESSED, slot %d\r\n", scancode, i);
                            break;
                        }
                    }
                } else {
                    for (int i = 0; i < 6; i++) {
                        if (hid_report.keys[i] == scancode) {
                            hid_report.keys[i] = 0x00;
                            DEBUG_PRINT(2, USBHID_TASK_DEBUG_LEVEL, "UsbHidTask: Key 0x%02X RELEASED, slot %d\r\n", scancode, i);
                        }
                    }
                }
            }
            USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&hid_report, sizeof(hid_report));
            DEBUG_PRINT(1, USBHID_TASK_DEBUG_LEVEL, "UsbHidTask: Sent HID report. Modifiers=0x%02X, Keys={0x%02X, 0x%02X, ..., 0x%02X}\r\n", hid_report.modifiers, hid_report.keys[0], hid_report.keys[1], hid_report.keys[5]);
            // Send empty report on key release to be safe
            if(received_event.state == KEY_RELEASED) {
				osDelay(10); // Debounce release
				// Compact the key array
				uint8_t write_idx = 0;
				for(uint8_t read_idx = 0; read_idx < 6; read_idx++) {
					if(hid_report.keys[read_idx] != 0x00) {
						hid_report.keys[write_idx++] = hid_report.keys[read_idx];
					}
				}
				for(uint8_t i = write_idx; i < 6; i++) {
					hid_report.keys[i] = 0x00;
				}
				USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&hid_report, sizeof(hid_report));
				DEBUG_PRINT(1, USBHID_TASK_DEBUG_LEVEL, "UsbHidTask: Sent empty HID report after release. Modifiers=0x%02X, Keys={0x%02X, ..., 0x%02X}\r\n", hid_report.modifiers, hid_report.keys[0], hid_report.keys[5]);
			}
        }
    }
}

void ledManager(void *argument) {
    DEBUG_PRINT(1, LEDMANAGER_TASK_DEBUG_LEVEL, "ledManager: Entered.\r\n");
    LedCommand_t cmd;
    for (;;) {
        osStatus_t status = osMessageQueueGet(ledQueueHandle, &cmd, NULL, osWaitForever);
        if (status == osOK) {
            DEBUG_PRINT(2, LEDMANAGER_TASK_DEBUG_LEVEL, "ledManager: Received command for LED %d, state %d\r\n", cmd.led, cmd.state);
            GPIO_PinState pin_state = (cmd.state == LED_ON) ? GPIO_PIN_SET : GPIO_PIN_RESET;
            switch (cmd.led) {
                case LED_NUM_LOCK:
                    HAL_GPIO_WritePin(LED_NUM_LOCK_GPIO_Port, LED_NUM_LOCK_Pin, pin_state);
                    DEBUG_PRINT(1, LEDMANAGER_TASK_DEBUG_LEVEL, "ledManager: Num Lock LED set to %s\r\n", (cmd.state == LED_ON ? "ON" : "OFF"));
                    break;
                case LED_CAPS_LOCK:
                    HAL_GPIO_WritePin(LED_CAPS_LOCK_GPIO_Port, LED_CAPS_LOCK_Pin, pin_state);
                    DEBUG_PRINT(1, LEDMANAGER_TASK_DEBUG_LEVEL, "ledManager: Caps Lock LED set to %s\r\n", (cmd.state == LED_ON ? "ON" : "OFF"));
                    break;
                case LED_SCROLL_LOCK:
                    HAL_GPIO_WritePin(LED_SCROLL_LOCK_GPIO_Port, LED_SCROLL_LOCK_Pin, pin_state);
                    DEBUG_PRINT(1, LEDMANAGER_TASK_DEBUG_LEVEL, "ledManager: Scroll Lock LED set to %s\r\n", (cmd.state == LED_ON ? "ON" : "OFF"));
                    break;
            }
        }
    }
}

void USBD_HID_SetReport_Callback(uint8_t *report, uint16_t len) {
    if (len > 0) {
        uint8_t led_status = report[0];
        static uint8_t last_led_status = 0;
        DEBUG_PRINT(1, LEDMANAGER_TASK_DEBUG_LEVEL, "USBD_HID_SetReport_Callback: Received LED status: 0x%02X\r\n", led_status);
        if (((last_led_status ^ led_status) >> 0) & 1) { // Num Lock
            LedCommand_t cmd = { .led = LED_NUM_LOCK, .state = (led_status & 1) ? LED_ON : LED_OFF };
            osMessageQueuePut(ledQueueHandle, &cmd, 0U, 0U);
        }
        if (((last_led_status ^ led_status) >> 1) & 1) { // Caps Lock
            LedCommand_t cmd = { .led = LED_CAPS_LOCK, .state = (led_status & 2) ? LED_ON : LED_OFF };
            osMessageQueuePut(ledQueueHandle, &cmd, 0U, 0U);
        }
        if (((last_led_status ^ led_status) >> 2) & 1) { // Scroll Lock
            LedCommand_t cmd = { .led = LED_SCROLL_LOCK, .state = (led_status & 4) ? LED_ON : LED_OFF };
            osMessageQueuePut(ledQueueHandle, &cmd, 0U, 0U);
        }
        last_led_status = led_status;
    }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartMainTask */
/**
  * @brief  Function implementing the mainTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartMainTask */
void StartMainTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}



/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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

#ifdef USE_FULL_ASSERT
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
