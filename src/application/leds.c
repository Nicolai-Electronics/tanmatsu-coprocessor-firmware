#include "leds.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "ch32v003fun.h"

#define NUM_LEDS        6
#define TIMER1_MAX_DUTY (176 + 4)

volatile uint8_t user_led_data[NUM_LEDS * 3] = {0};
volatile uint8_t internal_led_data[NUM_LEDS * 3] = {0};
volatile bool led_automatic_control = true;
volatile uint8_t led_brightness = 255;

static bool led_transmit_ready = false;

void DMA1_Channel5_IRQHandler(void) __attribute__((interrupt));
void DMA1_Channel5_IRQHandler(void) {
    // Check if Transfer Complete flag is set
    if (DMA1->INTFR & DMA1_IT_TC5) {
        DMA1->INTFCR = DMA1_IT_TC5;
        led_transmit_ready = true;
        TIM1->CTLR1 &= ~TIM_CEN;
    }
}

static uint8_t combined_led_data[NUM_LEDS * 3] = {0};      // Combined and brightness-adjusted LED data
static uint16_t timer1_data[NUM_LEDS * 3 * 8 + 45] = {0};  // Buffer for DMA transfer

bool write_addressable_leds(void) {
    if (!led_transmit_ready) {
        return false;  // Previous transmission still in progress
    }

    led_transmit_ready = false;

    if (led_automatic_control) {
        for (uint8_t i = 0; i < 4 * 3; i++) {
            combined_led_data[i] = (internal_led_data[i] * led_brightness) / 255;
        }
        for (uint8_t i = 0; i < 2 * 3; i++) {
            combined_led_data[i + (4 * 3)] = (user_led_data[i + (4 * 3)] * led_brightness) / 255;
        }
    } else {
        for (uint8_t i = 0; i < sizeof(user_led_data); i++) {
            combined_led_data[i] = (user_led_data[i] * led_brightness) / 255;
        }
    }

    for (int i = 0; i < NUM_LEDS * 3; i++) {
        for (int j = 0; j < 8; j++) {
            bool bit = (combined_led_data[i] >> (7 - j)) & 1;
            timer1_data[(i * 8) + j] = bit ? (TIMER1_MAX_DUTY / 2) : (TIMER1_MAX_DUTY / 4);
        }
    }

    DMA1_Channel5->PADDR = (uint32_t)&TIM1->CH4CVR;                // Set destination peripheral address
    DMA1_Channel5->CFGR &= ~DMA_CFGR1_EN;                          // Disable DMA channel
    DMA1_Channel5->MADDR = (uint32_t)timer1_data;                  // Set source memory address
    DMA1_Channel5->CNTR = sizeof(timer1_data) / sizeof(uint16_t);  // Set length

    DMA1_Channel5->CFGR |= DMA_CFGR1_EN;  // Enable DMA Channel
    TIM1->DMAINTENR |= TIM_UDE;           // UDE = Update DMA Enable
    TIM1->CTLR1 |= TIM_CEN;               // Enable timer

    return true;
}

void led_init(void) {
    RCC->APB2PCENR |= RCC_APB2Periph_GPIOA;  // Enable clock for GPIOA
    RCC->APB2PCENR |= RCC_APB2Periph_TIM1;   // Enable clock for timer 1
    RCC->AHBPCENR |= RCC_AHBPeriph_DMA1;     // Enable clock for DMA1
    funPinMode(PA11, GPIO_Speed_50MHz | GPIO_CNF_OUT_PP_AF);

    // Reset timer 1 peripheral
    RCC->APB2PRSTR |= RCC_APB2Periph_TIM1;
    RCC->APB2PRSTR &= ~RCC_APB2Periph_TIM1;

    TIM1->PSC = 0;                             // Set prescaler
    TIM1->ATRLR = TIMER1_MAX_DUTY;             // Set maximum counter value for duty cycle
    TIM1->CHCTLR2 |= TIM_OC4M_2 | TIM_OC4M_1;  // | TIM_OC4PE;  // Enable channel 4
    TIM1->CCER |= TIM_CC4E | TIM_CC4NP;        // Enable channel 4 output, negative polarity
    TIM1->BDTR |= TIM_MOE;                     // Main output enable

    NVIC_EnableIRQ(DMA1_Channel5_IRQn);  // Enable DMA end of transfer interrupt

    DMA1_Channel5->CFGR = DMA_CFGR1_DIR |      // Memory to peripheral
                          DMA_CFGR1_MINC |     // Increment memory address
                          DMA_CFGR1_MSIZE_0 |  // 16-bit memory size
                          DMA_CFGR1_PSIZE_0 |  // 16-bit peripheral size
                          DMA_CFGR1_TCIE;      // Transfer Complete Interrupt Enable

    led_transmit_ready = true;
}

void set_led_data_all(uint8_t* source, bool internal) {
    volatile uint8_t* target = internal ? internal_led_data : user_led_data;
    for (uint8_t i = 0; i < sizeof(user_led_data); i++) {
        target[i] = source[i];
    }
}

void set_led_data(uint8_t led_index, uint32_t color, bool internal) {
    if (led_index >= 6) return;
    uint8_t color_data[3] = {(color >> 8) & 0xFF, (color >> 16) & 0xFF, (color >> 0) & 0xFF};  // Green, Red, Blue

    volatile uint8_t* target = internal ? internal_led_data : user_led_data;

    bool changed = false;
    for (uint8_t i = 0; i < 3; i++) {
        if (target[led_index * 3 + i] != color_data[i]) {
            changed = true;
            break;
        }
    }

    if (changed) {
        for (uint8_t i = 0; i < 3; i++) {
            target[led_index * 3 + i] = color_data[i];
        }
    }
}

void set_power_led(uint32_t color) {
    set_led_data(0, color, true);
}

void set_radio_led(uint32_t color) {
    set_led_data(1, color, true);
}

void set_message_led(uint32_t color) {
    set_led_data(2, color, true);
}

void set_powerbutton_led(uint32_t color) {
    set_led_data(3, color, true);
}

void set_led_brightness(uint8_t brightness) {
    led_brightness = brightness;
}

void set_led_mode(uint8_t mode) {
    led_automatic_control = mode & 1;
}
