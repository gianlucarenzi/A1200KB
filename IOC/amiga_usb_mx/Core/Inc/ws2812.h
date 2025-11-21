#ifndef __WS2812_H
#define __WS2812_H

#include "stm32f4xx_hal.h"

// Define WS2812 protocol timings based on 800kHz data rate
// These values are typically for a TIM counter clock of 84MHz (APB1 Timer clock for STM32F401 is up to 84MHz)
// We need to achieve a PWM frequency of around 800kHz * 24 bits (approx 19.2MHz) for full resolution
// A common approach is to use a PWM period that allows for distinct high pulse widths for '0' and '1'.
//
// For WS2812B:
// T0H (0 code high voltage time): 0.35 us (±150ns)
// T0L (0 code low voltage time): 0.8 us (±150ns)
// T1H (1 code high voltage time): 0.7 us (±150ns)
// T1L (1 code low voltage time): 0.6 us (±150ns)
// TRESET (Reset code): >= 50 us
//
// A common PWM period is 1.25 us (800kHz). With a timer clocked at 84MHz, one tick is 1/84MHz = 0.0119 us.
// A PWM period of 1.25 us = 1.25 / 0.0119 = ~105 ticks. Let's use 105 for simplicity.
//
// For 84MHz timer clock:
// Period = 105 (1.25 us)
// T0H = 0.35 us / 0.0119 us/tick = ~29 ticks
// T1H = 0.7 us / 0.0119 us/tick = ~59 ticks
// Reset pulse: We need to send 0s for ~50us. 50us / 1.25us per bit = 40 bits of 0s.
//
// Let's define these.
#define WS2812_TIM_PERIOD           105  // Total ticks for one bit (approx 1.25us at 84MHz timer clock)
#define WS2812_0_HIGH_TICKS         29   // T0H duration (approx 0.35us)
#define WS2812_1_HIGH_TICKS         59   // T1H duration (approx 0.70us)
#define WS2812_RESET_BIT_COUNT      50   // Number of '0' bits to send for reset (50 * 1.25us = 62.5us > 50us)

// Number of LEDs in the strip (can be changed by user)
#define NUM_LEDS                    1    // User has 1 LED

// Size of the DMA buffer. Each LED needs 24 bits (GRB). Each bit needs 1 PWM cycle.
// Plus an extra number of 0s for the reset pulse.
#define DMA_BUFFER_SIZE             (NUM_LEDS * 24 + WS2812_RESET_BIT_COUNT)

// External variables (defined in .c file)
extern TIM_HandleTypeDef htim2;
extern DMA_HandleTypeDef hdma_tim2_ch2;

// Function prototypes
void WS2812_Init(void);
void WS2812_SetRGB(uint16_t led_index, uint8_t r, uint8_t g, uint8_t b);
void WS2812_Show(void);
void led_rgb(uint32_t RGB); // Add prototype for led_rgb

#endif /* __WS2812_H */
