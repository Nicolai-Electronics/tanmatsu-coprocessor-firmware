#include "leds.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "ch32v003fun.h"

volatile uint8_t user_led_data[6 * 3] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
volatile uint8_t internal_led_data[6 * 3] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
volatile bool leds_need_update = true;
volatile bool led_automatic_control = true;
volatile uint8_t led_brightness = 255;

void set_led_data_all(uint8_t* source, bool internal) {
    volatile uint8_t* target = internal ? internal_led_data : user_led_data;
    for (uint8_t i = 0; i < sizeof(user_led_data); i++) {
        target[i] = source[i];
    }
    leds_need_update = true;
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
        leds_need_update = true;
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
    leds_need_update = true;
}

void set_led_mode(uint8_t mode) {
    led_automatic_control = mode & 1;
    leds_need_update = true;
}

void write_addressable_leds(void) __attribute__((optimize("O0")));
void write_addressable_leds(void) {
    if (!leds_need_update) {
        return;  // No update needed
    }
    leds_need_update = false;
    uint8_t buffer[sizeof(user_led_data)] = {0};
    if (led_automatic_control) {
        for (uint8_t i = 0; i < 4 * 3; i++) {
            buffer[i] = (internal_led_data[i] * led_brightness) / 255;
        }
        for (uint8_t i = 0; i < 2 * 3; i++) {
            buffer[i + (4 * 3)] = (user_led_data[i + (4 * 3)] * led_brightness) / 255;
        }
    } else {
        for (uint8_t i = 0; i < sizeof(user_led_data); i++) {
            buffer[i] = (user_led_data[i] * led_brightness) / 255;
        }
    }
    I2C1->CTLR2 &= ~(I2C_CTLR2_ITEVTEN);  // Disable I2C event interrupt
    for (uint8_t pos_byte = 0; pos_byte < sizeof(buffer); pos_byte++) {
        for (int i = 7; i >= 0; i--) {
            if ((buffer[pos_byte] >> i) & 1) {
                // Send 1
                GPIOA->BSHR |= 1 << (11);
                // T1H: 0.6us
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                GPIOA->BSHR |= 1 << (11 + 16);
                // T1L: 0.6us
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
            } else {
                // Send 0
                GPIOA->BSHR |= 1 << (11);
                // T0H: 0.3us
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                GPIOA->BSHR |= 1 << (11 + 16);
                // T0L: 0.9us
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
                __asm__("nop");
            }
        }
    }
    I2C1->CTLR2 |= I2C_CTLR2_ITEVTEN;  // Enable I2C event interrupt
}