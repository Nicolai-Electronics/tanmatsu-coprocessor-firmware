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

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "ch32v003fun.h"
#include "semphr.h"

// Protects the I2C2 (PMIC) bus: pmic_task, radio_task and the power-off
// sequence in input_task all issue PMIC transactions from separate tasks.
static SemaphoreHandle_t i2c2_mutex = NULL;

void SetupI2CMaster(void) {
    i2c2_mutex = xSemaphoreCreateMutex();

    // Enable I2C2
    RCC->APB1PCENR |= RCC_APB1Periph_I2C2;
    RCC->APB2PCENR |= RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO;

    // Reset I2C2 to init all regs
    RCC->APB1PRSTR |= RCC_APB1Periph_I2C2;
    RCC->APB1PRSTR &= ~RCC_APB1Periph_I2C2;

    I2C2->CTLR1 |= I2C_CTLR1_SWRST;
    I2C2->CTLR1 &= ~I2C_CTLR1_SWRST;

    // Set module clock frequency
    uint32_t prerate = 2000000;  // I2C Logic clock rate, must be higher than the bus clock rate
    I2C2->CTLR2 |= (FUNCONF_SYSTEM_CORE_CLOCK / prerate) & I2C_CTLR2_FREQ;

    // Set clock configuration
    uint32_t clockrate = 1000000;  // I2C Bus clock rate, must be lower than the logic clock rate
    I2C2->CKCFGR =
        ((FUNCONF_SYSTEM_CORE_CLOCK / (3 * clockrate)) & I2C_CKCFGR_CCR) | I2C_CKCFGR_FS;  // Fast mode 33% duty cycle
    // I2C2->CKCFGR = ((FUNCONF_SYSTEM_CORE_CLOCK/(25*clockrate))&I2C_CKCFGR_CCR) | I2C_CKCFGR_DUTY | I2C_CKCFGR_FS; //
    // Fast mode 36% duty cycle I2C2->CKCFGR = (FUNCONF_SYSTEM_CORE_CLOCK/(2*clockrate))&I2C_CKCFGR_CCR; // Standard
    // mode good to 100kHz

    // Enable interrupts
    I2C2->CTLR2 |= I2C_CTLR2_ITBUFEN | I2C_CTLR2_ITEVTEN | I2C_CTLR2_ITERREN;
    NVIC_EnableIRQ(I2C2_EV_IRQn);  // Event interrupt
    NVIC_SetPriority(I2C2_EV_IRQn, 2 << 4);
    NVIC_EnableIRQ(I2C2_ER_IRQn);  // Error interrupt
    NVIC_SetPriority(I2C2_ER_IRQn, 2 << 4);

    // Configure address
    I2C2->OADDR1 = 0;
    I2C2->OADDR2 = 0;

    // Enable I2C
    I2C2->CTLR1 |= I2C_CTLR1_PE;
}

typedef enum i2c_result {
    i2c_ok = 0,
    i2c_invalid_params = 1,
    i2c_timeout = 2,
    i2c_error = 3,
} i2c_result_t;

#define I2C_TIMEOUT 100000

#define I2C_STATE_BUSY ((uint32_t)(I2C_STAR2_BUSY << 16))

#define I2C_STATE_MASTER_MODE_SELECTED ((uint32_t)((I2C_STAR1_SB) | ((I2C_STAR2_BUSY | I2C_STAR2_MSL) << 16)))  // EV5
#define I2C_STATE_MASTER_TRANSMITTER_MODE_SELECTED \
    ((uint32_t)((I2C_STAR1_ADDR | I2C_STAR1_TXE) | ((I2C_STAR2_TRA | I2C_STAR2_BUSY | I2C_STAR2_MSL) << 16)))  // EV6
#define I2C_STATE_MASTER_RECEIVER_MODE_SELECTED \
    ((uint32_t)((I2C_STAR1_ADDR) | ((I2C_STAR2_BUSY | I2C_STAR2_MSL) << 16)))  // EV6
#define I2C_STATE_MASTER_TRANSMITTER_MODE_ADDRESS10 \
    ((uint32_t)((I2C_STAR1_ADD10) | ((I2C_STAR2_BUSY | I2C_STAR2_MSL) << 16)))  // EV9
#define I2C_STATE_MASTER_TRANSMITTER_BYTE_RECEIVED \
    ((uint32_t)((I2C_STAR1_RXNE) | ((I2C_STAR2_BUSY | I2C_STAR2_MSL) << 16)))  // EV7
#define I2C_STATE_MASTER_TRANSMITTER_BYTE_TRANSMITTING \
    ((uint32_t)((I2C_STAR1_TXE) | ((I2C_STAR2_TRA | I2C_STAR2_BUSY | I2C_STAR2_MSL) << 16)))  // EV8
#define I2C_STATE_MASTER_TRANSMITTER_BYTE_TRANSMITTED \
    ((uint32_t)((I2C_STAR1_TXE | I2C_STAR1_BTF) | ((I2C_STAR2_TRA | I2C_STAR2_BUSY | I2C_STAR2_MSL) << 16)))  // EV8_2

#define I2C_STATE_SLAVE_RECEIVER_ADDRESS_MATCHED ((uint32_t)((I2C_STAR1_ADDR) | ((I2C_STAR2_BUSY) << 16)))  // EV1
#define I2C_STATE_SLAVE_TRANSMITTER_ADDRESS_MATCHED \
    ((uint32_t)((I2C_STAR1_ADDR | I2C_STAR1_TXE) | ((I2C_STAR2_TRA | I2C_STAR2_BUSY) << 16)))                   // EV1
#define I2C_STATE_SLAVE_RECEIVER_SECOND_ADDRESS_MATCHED ((uint32_t)((I2C_STAR2_BUSY | I2C_STAR2_DUALF) << 16))  // EV1
#define I2C_STATE_SLAVE_TRANSMITTER_SECOND_ADDRESS_MATCHED \
    ((uint32_t)((I2C_STAR1_TXE) | (I2C_STAR2_TRA | (I2C_STAR2_BUSY | I2C_STAR2_DUALF) << 16)))                    // EV1
#define I2C_STATE_SLAVE_GENERALCALL_ADDRESS_MATCHED ((uint32_t)((I2C_STAR2_GENCALL) | ((I2C_STAR2_BUSY) << 16)))  // EV1
#define I2C_STATE_SLAVE_BYTE_RECEIVED               ((uint32_t)((I2C_STAR1_RXNE) | ((I2C_STAR2_BUSY) << 16)))     // EV2
#define I2C_STATE_SLAVE_STOP_DETECTED ((uint32_t) ((I2C_STAR1_STOPF)) // EV4
#define I2C_STATE_SLAVE_BYTE_TRANSMITTED \
    ((uint32_t)((I2C_STAR1_TXE | I2C_STAR1_BTF) | ((I2C_STAR2_TRA | I2C_STAR2_BUSY) << 16)))  // EV3
#define I2C_STATE_SLAVE_BYTE_TRANSMITTING \
    ((uint32_t)((I2C_STAR1_TXE) | ((I2C_STAR2_TRA | I2C_STAR2_BUSY) << 16)))  // EV3
#define I2C_STATE_SLAVE_ACK_FAILURE ((uint32_t)(I2C_STAR1_AF))                // EV3_2

static bool pm_i2c_check_state(uint32_t state_mask) {
    uint16_t STAR1 = I2C2->STAR1;
    uint16_t STAR2 = (I2C2->STAR2 & ~(I2C_STAR2_PEC));
    uint32_t flags = STAR1 | (STAR2 << 16);
    return (flags & state_mask) == state_mask;
}

static void pm_i2c_send_start() {
    I2C2->CTLR1 |= I2C_CTLR1_START;
}

static void pm_i2c_send_stop() {
    I2C2->CTLR1 |= I2C_CTLR1_STOP;
}

static void pm_i2c_enable_ack() {
    I2C2->CTLR1 |= I2C_CTLR1_ACK;
}

static void pm_i2c_disable_ack() {
    I2C2->CTLR1 &= ~(I2C_CTLR1_ACK);
}

static void pm_i2c_send_address(uint8_t address, bool read) {
    I2C2->DATAR = (address << 1) | (read ? 0x01 : 0x00);
}

/**
 * @brief Execute bus transaction
 */
i2c_result_t pm_i2c_transaction(uint8_t address, uint8_t* tx_data, uint32_t tx_length, uint8_t* rx_data,
                                uint32_t rx_length) {
    I2C2->CTLR2 &= ~(I2C_CTLR2_ITBUFEN | I2C_CTLR2_ITEVTEN | I2C_CTLR2_ITERREN);  // Disable interrupts
    int32_t timeout;

    // Send start condition
    pm_i2c_send_start();

    // Wait for bus to be claimed
    timeout = I2C_TIMEOUT;
    while ((!pm_i2c_check_state(I2C_STATE_MASTER_MODE_SELECTED)) && (timeout--));
    if (timeout < 0) {
        return i2c_timeout;
    }

    // Send address
    pm_i2c_send_address(address, false);

    // Wait for transmiter mode to be selected
    timeout = I2C_TIMEOUT;
    while ((!pm_i2c_check_state(I2C_STATE_MASTER_TRANSMITTER_MODE_SELECTED)) && (timeout--));
    if (timeout < 0) {
        return i2c_timeout;
    }

    for (uint32_t index = 0; index < tx_length; index++) {
        I2C2->DATAR = tx_data[index];
        // Wait for byte to be transmitted
        timeout = I2C_TIMEOUT;
        while ((!pm_i2c_check_state(I2C_STATE_MASTER_TRANSMITTER_BYTE_TRANSMITTED)) && (timeout--));
        if (timeout < 0) {
            return i2c_timeout;
        }
    }

    if (rx_length > 0) {
        // Send start condition
        pm_i2c_send_start();

        // Wait for bus to be claimed
        timeout = I2C_TIMEOUT;
        while ((!pm_i2c_check_state(I2C_STATE_MASTER_MODE_SELECTED)) && (timeout--));
        if (timeout < 0) {
            return i2c_timeout;
        }

        // Send address
        pm_i2c_send_address(address, true);

        // Acknowledge received bytes
        pm_i2c_enable_ack();

        // Wait for receiver mode to be selected
        timeout = I2C_TIMEOUT;
        while ((!pm_i2c_check_state(I2C_STATE_MASTER_RECEIVER_MODE_SELECTED)) && (timeout--));
        if (timeout < 0) {
            return i2c_timeout;
        }

        for (uint32_t index = 0; index < rx_length; index++) {
            if (index == rx_length - 1) {
                // Last byte
                pm_i2c_disable_ack();
            }
            // Wait for byte to be received
            timeout = I2C_TIMEOUT;
            while ((!pm_i2c_check_state(I2C_STATE_MASTER_TRANSMITTER_BYTE_RECEIVED)) && (timeout--));
            if (timeout < 0) {
                return i2c_timeout;
            }
            rx_data[index] = I2C2->DATAR;
        }
    }

    // Send stop condition
    pm_i2c_send_stop();

    return i2c_ok;
}

/**
 * @brief Write to the I2C bus
 */
i2c_result_t pm_i2c_write_reg(uint8_t address, uint8_t reg, uint8_t* data, uint32_t length) {
    uint8_t buffer[65];

    if (length >= sizeof(buffer)) {
        return i2c_invalid_params;
    }

    buffer[0] = reg;
    memcpy(&buffer[1], data, length);

    xSemaphoreTake(i2c2_mutex, portMAX_DELAY);
    i2c_result_t result = pm_i2c_transaction(address, buffer, length + 1, NULL, 0);
    xSemaphoreGive(i2c2_mutex);
    return result;
}

/**
 * @brief Read from the I2C bus
 */
i2c_result_t pm_i2c_read_reg(uint8_t address, uint8_t reg, uint8_t* data, uint32_t length) {
    xSemaphoreTake(i2c2_mutex, portMAX_DELAY);
    i2c_result_t result = pm_i2c_transaction(address, &reg, 1, data, length);
    xSemaphoreGive(i2c2_mutex);
    return result;
}
