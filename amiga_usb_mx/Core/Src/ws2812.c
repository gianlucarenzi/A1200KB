#include "ws2812.h"
#include "stm32f4xx_hal.h"

// Timer and DMA handles
TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim2_ch2;

// WS2812B timing constants (for 84MHz timer clock)
#define TIMER_CLOCK_FREQ 84000000
#define WS2812B_FREQ     800000
#define TIMER_PERIOD     (TIMER_CLOCK_FREQ / WS2812B_FREQ)
#define PULSE_0_HIGH     34  // (105 * 0.4 / 1.25)
#define PULSE_1_HIGH     67  // (105 * 0.8 / 1.25)

// DMA buffer size
#define DMA_BUFFER_SIZE  (24 + 50) // 24 bits for color, 50 for reset
uint16_t dma_buffer[DMA_BUFFER_SIZE];

void ws2812_init(void) {
    // GPIO Initialization
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // TIM2 and DMA1 clock enable
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    // TIM2 configuration
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = TIMER_PERIOD - 1;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_PWM_Init(&htim2);

    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2);

    // DMA1 Stream 6 Channel 3 for TIM2_CH2
    hdma_tim2_ch2.Instance = DMA1_Stream6;
    hdma_tim2_ch2.Init.Channel = DMA_CHANNEL_3;
    hdma_tim2_ch2.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim2_ch2.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim2_ch2.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim2_ch2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_tim2_ch2.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_tim2_ch2.Init.Mode = DMA_NORMAL;
    hdma_tim2_ch2.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_tim2_ch2.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdma_tim2_ch2);

    __HAL_LINKDMA(&htim2, hdma[TIM_DMA_ID_CC2], hdma_tim2_ch2);

    HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
}

void led_rgb(uint32_t color) {
    for (int i = 23; i >= 0; i--) {
        if ((color >> i) & 1) {
            dma_buffer[23-i] = PULSE_1_HIGH;
        } else {
            dma_buffer[23-i] = PULSE_0_HIGH;
        }
    }
    for (int i = 24; i < DMA_BUFFER_SIZE; i++) {
        dma_buffer[i] = 0;
    }

    HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_2, (uint32_t*)dma_buffer, DMA_BUFFER_SIZE);
}
