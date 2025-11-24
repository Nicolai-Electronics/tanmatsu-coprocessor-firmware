#pragma once

#include <stdbool.h>
#include <stdint.h>

void set_led_data_all(uint8_t* source, bool internal);
void set_led_data(uint8_t led_index, uint32_t color, bool internal);
void set_power_led(uint32_t color);
void set_radio_led(uint32_t color);
void set_message_led(uint32_t color);
void set_powerbutton_led(uint32_t color);
void set_led_brightness(uint8_t brightness);
void set_led_mode(uint8_t mode);
void write_addressable_leds(void);
