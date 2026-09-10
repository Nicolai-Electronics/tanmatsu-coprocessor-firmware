// Tanmatsu coprocessor firmware
// SPDX-FileCopyrightText: 2024 Nicolai Electronics
// SPDX-License-Identifier: MIT

#include "keyboard.h"
#include <stdbool.h>
#include <stdint.h>
#include "ch32fun.h"

// Configuration
static const uint8_t keyboard_rows[] = {PA8, PA9, PA10, PA4, PA3, PA1, PA6, PA5, PA2};
static const uint8_t keyboard_columns[] = {PB14, PB12, PB1, PA7, PB15, PB13, PB2, PB0};
static const uint32_t input_scan_timeout = 20;  // steps

void keyboard_init(void) {
    // Configure GPIOs
    for (uint8_t row = 0; row < sizeof(keyboard_rows); row++) {
        funPinMode(keyboard_rows[row], GPIO_Speed_10MHz | GPIO_CNF_OUT_PP);
        funDigitalWrite(keyboard_rows[row], FUN_LOW);
    }

    for (uint8_t column = 0; column < sizeof(keyboard_columns); column++) {
        funPinMode(keyboard_columns[column], GPIO_Speed_In | GPIO_CNF_IN_FLOATING);
    }
}

bool keyboard_step(volatile uint8_t* registers) {
    static uint32_t timeout = input_scan_timeout;
    static uint8_t row = 0;
    static uint8_t previous_values[sizeof(keyboard_rows)] = {0};
    static bool key_pressed = false;

    bool changed = false;

    if (timeout == 0 && !key_pressed) {
        // Idle mode
        // Enable all rows and scan for any key to be pressed
        for (uint8_t row = 0; row < sizeof(keyboard_rows); row++) {
            funDigitalWrite(keyboard_rows[row], FUN_HIGH);
        }
        for (uint8_t column = 0; column < sizeof(keyboard_columns); column++) {
            if (funDigitalRead(keyboard_columns[column])) {
                for (uint8_t row = 0; row < sizeof(keyboard_rows); row++) {
                    funDigitalWrite(keyboard_rows[row], FUN_LOW);
                }
                timeout = input_scan_timeout;
            }
        }
    }

    if (timeout > 0) {
        // Normal row scanning mode
        // Each step one row gets enabled after which the keys on that row can be read
        if (row == 0) {
            key_pressed = false;
        }

        uint8_t value = 0;
        for (uint8_t column = 0; column < sizeof(keyboard_columns); column++) {
            value |= funDigitalRead(keyboard_columns[column]) << column;
        }
        registers[row] = value;
        changed = previous_values[row] != value;
        previous_values[row] = value;

        if (value > 0) {
            key_pressed = true;
            timeout = input_scan_timeout;
        }

        // Next row
        funDigitalWrite(keyboard_rows[row], FUN_LOW);
        row++;
        if (row >= sizeof(keyboard_rows)) {
            row = 0;
        }
        funDigitalWrite(keyboard_rows[row], FUN_HIGH);
    }

    if (changed) {
        timeout = input_scan_timeout;
    } else if (timeout > 0) {
        timeout--;
    }

    return changed;
}