#pragma once

#include <stdint.h>

void leds_init(uint8_t* led_registers);
void set_led_data_all();
void set_led_data(uint8_t led_index, uint32_t color);
void set_power_led(uint32_t color);
void set_radio_led(uint32_t color);
void write_addressable_leds(void);
