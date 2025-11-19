#ifndef __WS2812_H__
#define __WS2812_H__

#include <stdint.h>
#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef htim2;
extern DMA_HandleTypeDef hdma_tim2_ch2;

void ws2812_init(void);
void led_rgb(uint32_t color);

#endif /* __WS2812_H__ */
