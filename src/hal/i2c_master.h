/*
 * Library for using the I2C peripheral in master mode
 *
 * MIT License
 *
 * Copyright (c) 2024 Renze Nicolai
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "ch32fun.h"

typedef enum i2c_result {
    i2c_ok = 0,
    i2c_invalid_params = 1,
    i2c_timeout = 2,
    i2c_error = 3,
} i2c_result_t;

/**
 * @brief Initialize the library
 */
void SetupI2CMaster(void);

/**
 * @brief Execute bus transaction
 */
i2c_result_t pm_i2c_transaction(uint8_t address, uint8_t* tx_data, uint32_t tx_length, uint8_t* rx_data,
                                uint32_t rx_length);

/**
 * @brief Write to the I2C bus
 */
i2c_result_t pm_i2c_write_reg(uint8_t address, uint8_t reg, uint8_t* data, uint32_t length);

/**
 * @brief Read from the I2C bus
 */
i2c_result_t pm_i2c_read_reg(uint8_t address, uint8_t reg, uint8_t* data, uint32_t length);