#include "ws2812.h"

TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim2_ch2;

// DMA Buffer for WS2812 data
volatile uint16_t pwm_data[DMA_BUFFER_SIZE];

// Function to initialize TIM2 and DMA for WS2812
void WS2812_Init(void)
{
    // GPIO and Clock configurations will be handled in HAL_TIM_PWM_MspInit()

    // TIM2 Base configuration
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = (HAL_RCC_GetPCLK1Freq() / 84000000) - 1; // Assuming PCLK1 = 42MHz, PCLK1_Timer = 84MHz.
                                                                  // 84MHz / 84MHz = 1. So prescaler = 0.
                                                                  // Timer clock = 84MHz.
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = WS2812_TIM_PERIOD - 1;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_PWM_Init(&htim2);

    // TIM2 Channel 2 PWM configuration
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0; // Initial duty cycle
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2);

    // DMA Configuration for TIM2_CH2 (Stream 6, Channel 3)
    // DMA initialization will be handled in HAL_TIM_PWM_MspInit() by linking DMA to TIM.
    // Ensure the DMA instance and channel are correctly identified from datasheet/CubeMX.
    hdma_tim2_ch2.Instance = DMA1_Stream6; // Check datasheet for correct stream and channel
    hdma_tim2_ch2.Init.Channel = DMA_CHANNEL_3;
    hdma_tim2_ch2.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim2_ch2.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim2_ch2.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim2_ch2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD; // PWM data is 16-bit
    hdma_tim2_ch2.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_tim2_ch2.Init.Mode = DMA_NORMAL; // Not circular for single transfer
    hdma_tim2_ch2.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_tim2_ch2.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdma_tim2_ch2);

    // Link DMA to TIM2_CH2
    // The CubeMX generated code usually links DMA to TIM inside HAL_TIM_PWM_MspInit.
    // For manual setup, this linking needs to be done here if not in MspInit.
    // Assuming hdma is an array in htim.
    __HAL_LINKDMA(&htim2, hdma[TIM_DMA_ID_CC2], hdma_tim2_ch2); // Link TIM2_CH2 DMA to hdma_tim2_ch2

    // Fill DMA buffer with reset pulse (all 0s)
    for (uint16_t i = 0; i < DMA_BUFFER_SIZE; i++)
    {
        pwm_data[i] = WS2812_0_HIGH_TICKS;
    }
}

// Global buffer to hold the GRB values for each LED
uint8_t led_buffer[NUM_LEDS][3]; // [LED_INDEX][GRB]

// Function to set the RGB color for a specific LED
void WS2812_SetRGB(uint16_t led_index, uint8_t r, uint8_t g, uint8_t b)
{
    if (led_index < NUM_LEDS)
    {
        led_buffer[led_index][0] = g; // WS2812 uses GRB order
        led_buffer[led_index][1] = r;
        led_buffer[led_index][2] = b;
    }
}

// Function to convert RGB data to PWM duty cycle values and start DMA transfer
void WS2812_Show(void)
{
    uint32_t index = 0;

    // Convert GRB data for each LED into PWM data
    for (uint16_t i = 0; i < NUM_LEDS; i++)
    {
        for (int8_t j = 7; j >= 0; j--) // Green
        {
            if ((led_buffer[i][0] >> j) & 0x01)
            {
                pwm_data[index++] = WS2812_1_HIGH_TICKS;
            }
            else
            {
                pwm_data[index++] = WS2812_0_HIGH_TICKS;
            }
        }
        for (int8_t j = 7; j >= 0; j--) // Red
        {
            if ((led_buffer[i][1] >> j) & 0x01)
            {
                pwm_data[index++] = WS2812_1_HIGH_TICKS;
            }
            else
            {
                pwm_data[index++] = WS2812_0_HIGH_TICKS;
            }
        }
        for (int8_t j = 7; j >= 0; j--) // Blue
        {
            if ((led_buffer[i][2] >> j) & 0x01)
            {
                pwm_data[index++] = WS2812_1_HIGH_TICKS;
            }
            else
            {
                pwm_data[index++] = WS2812_0_HIGH_TICKS;
            }
        }
    }

    // Add reset pulse (all 0s)
    for (uint16_t i = 0; i < WS2812_RESET_BIT_COUNT; i++)
    {
        pwm_data[index++] = WS2812_0_HIGH_TICKS;
    }

    // Start PWM with DMA
    HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_2, (uint32_t*)pwm_data, DMA_BUFFER_SIZE);
}
