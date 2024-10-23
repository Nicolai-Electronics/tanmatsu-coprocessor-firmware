// Tanmatsu coprocessor firmware
// SPDX-FileCopyrightText: 2024 Nicolai Electronics
// SPDX-License-Identifier: MIT

#include "pmic.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "ch32v003fun.h"
#include "i2c_master.h"
#include "pmic_regs.h"

#define BQ25895_ADDR 0x6a

static pmic_result_t pmic_read_reg(uint8_t reg, uint8_t* out_value) {
    return (pm_i2c_read_reg(BQ25895_ADDR, reg, out_value, 1) == i2c_ok) ? PMIC_OK : PMIC_ERROR;
}

static pmic_result_t pmic_write_reg(uint8_t reg, uint8_t value) {
    return (pm_i2c_write_reg(BQ25895_ADDR, reg, &value, 1) == i2c_ok) ? PMIC_OK : PMIC_ERROR;
}

// REG00
pmic_result_t pmic_set_input_current_limit(uint16_t current, bool enable_ilim_pin, bool enable_hiz) {
    bq25895_reg00_t value = {0};

    if (current < 100) {  // Minimum current is 100 mA
        current = 100;
    }
    current -= 100;  // Offset is 100 mA

    value.en_hiz = enable_hiz;
    value.en_ilim = enable_ilim_pin;

    if (current >= 1600) {
        current -= 1600;
        value.iinlim |= (1 << 5);  // Add 1600 mA
    }
    if (current >= 800) {
        current -= 800;
        value.iinlim |= (1 << 4);  // Add 800 mA
    }
    if (current >= 400) {
        current -= 400;
        value.iinlim |= (1 << 3);  // Add 400 mA
    }
    if (current >= 200) {
        current -= 200;
        value.iinlim |= (1 << 2);  // Add 200 mA
    }
    if (current >= 100) {
        current -= 100;
        value.iinlim |= (1 << 1);  // Add 100 mA
    }
    if (current >= 50) {
        current -= 50;
        value.iinlim |= (1 << 0);  // Add 50 mA
    }

    return pmic_write_reg(0x00, value.raw);
}

pmic_result_t pmic_get_input_current_limit(uint16_t* out_current, bool* out_enable_ilim_pin, bool* out_enable_hiz) {
    bq25895_reg00_t value;
    pmic_result_t res = pmic_read_reg(0x00, &value.raw);
    if (res != PMIC_OK) return res;
    if (out_enable_hiz) {
        *out_enable_hiz = value.en_hiz;
    }
    if (out_enable_ilim_pin) {
        *out_enable_ilim_pin = value.en_ilim;
    }
    if (out_current) {
        *out_current = 100;  // Offset is 100mA
        if (value.iinlim & (1 << 5)) *out_current += 1600;
        if (value.iinlim & (1 << 4)) *out_current += 800;
        if (value.iinlim & (1 << 3)) *out_current += 400;
        if (value.iinlim & (1 << 2)) *out_current += 200;
        if (value.iinlim & (1 << 1)) *out_current += 100;
        if (value.iinlim & (1 << 0)) *out_current += 50;
    }
    return PMIC_OK;
}

// REG01
pmic_result_t pmic_set_input_voltage_limit_offset(uint8_t offset) {
    bq25895_reg01_t value = {0};
    pmic_result_t res = pmic_read_reg(0x01, &value.raw);
    if (res != PMIC_OK) return res;
    value.vindpm_os = offset;
    return pmic_write_reg(0x01, value.raw);
}

pmic_result_t pmic_get_input_voltage_limit_offset(uint8_t* out_offset) {
    bq25895_reg01_t value = {0};
    pmic_result_t res = pmic_read_reg(0x01, &value.raw);
    if (res != PMIC_OK) return res;
    if (out_offset) {
        *out_offset = value.vindpm_os;
    }
    return PMIC_OK;
}

pmic_result_t pmic_set_boost_mode_temperature_monitor_thresholds(bool cold, pmic_bhot_t hot) {
    bq25895_reg01_t value = {0};
    pmic_result_t res = pmic_read_reg(0x01, &value.raw);
    if (res != PMIC_OK) return res;
    value.bcold = cold;
    value.bhot = (uint8_t)hot;
    return pmic_write_reg(0x01, value.raw);
}

pmic_result_t pmic_get_boost_mode_temperature_monitor_thresholds(bool* out_cold, pmic_bhot_t* out_hot) {
    bq25895_reg01_t value = {0};
    pmic_result_t res = pmic_read_reg(0x01, &value.raw);
    if (res != PMIC_OK) return res;
    if (out_cold) {
        *out_cold = value.bcold;
    }
    if (out_hot) {
        *out_hot = (pmic_bhot_t)value.bhot;
    }
    return PMIC_OK;
}

// REG02
pmic_result_t pmic_set_otg_boost_frequency(bool low_frequency) {
    // Low frequency: 500kHz, high frequency: 1.5MHz
    bq25895_reg02_t value;
    pmic_result_t res = pmic_read_reg(0x02, &value.raw);
    if (res != PMIC_OK) return res;
    value.boost_freq = low_frequency;
    return pmic_write_reg(0x02, value.raw);
}

pmic_result_t pmic_get_otg_boost_frequency(bool* out_low_frequency) {
    bq25895_reg02_t value;
    pmic_result_t res = pmic_read_reg(0x02, &value.raw);
    if (res != PMIC_OK) return res;
    if (out_low_frequency) {
        *out_low_frequency = value.boost_freq;
    }
    return PMIC_OK;
}

pmic_result_t pmic_set_input_current_optimizer(bool enable) {
    bq25895_reg02_t value;
    pmic_result_t res = pmic_read_reg(0x02, &value.raw);
    if (res != PMIC_OK) return res;
    value.ico_en = enable;
    return pmic_write_reg(0x02, value.raw);
}

pmic_result_t pmic_get_input_current_optimizer(bool* out_enable) {
    bq25895_reg02_t value;
    pmic_result_t res = pmic_read_reg(0x02, &value.raw);
    if (res != PMIC_OK) return res;
    if (out_enable) {
        *out_enable = value.ico_en;
    }
    return PMIC_OK;
}

pmic_result_t pmic_set_high_voltage_dcp(bool enable) {
    bq25895_reg02_t value;
    pmic_result_t res = pmic_read_reg(0x02, &value.raw);
    if (res != PMIC_OK) return res;
    value.hvdcp_en = enable;
    return pmic_write_reg(0x02, value.raw);
}

pmic_result_t pmic_get_high_voltage_dcp(bool* out_enable) {
    bq25895_reg02_t value;
    pmic_result_t res = pmic_read_reg(0x02, &value.raw);
    if (res != PMIC_OK) return res;
    if (out_enable) {
        *out_enable = value.hvdcp_en;
    }
    return PMIC_OK;
}

pmic_result_t pmic_set_maxcharge(bool enable) {
    bq25895_reg02_t value;
    pmic_result_t res = pmic_read_reg(0x02, &value.raw);
    if (res != PMIC_OK) return res;
    value.maxc_en = enable;
    return pmic_write_reg(0x02, value.raw);
}

pmic_result_t pmic_get_maxcharge(bool* out_enable) {
    bq25895_reg02_t value;
    pmic_result_t res = pmic_read_reg(0x02, &value.raw);
    if (res != PMIC_OK) return res;
    if (out_enable) {
        *out_enable = value.maxc_en;
    }
    return PMIC_OK;
}

pmic_result_t pmic_set_adc_configuration(bool start, bool continuous) {
    bq25895_reg02_t value;
    pmic_result_t res = pmic_read_reg(0x02, &value.raw);
    if (res != PMIC_OK) return res;
    value.conv_start = start;
    value.conv_rate = continuous;
    return pmic_write_reg(0x02, value.raw);
}

pmic_result_t pmic_get_adc_configuration(bool* out_busy, bool* out_continuous) {
    bq25895_reg02_t value;
    pmic_result_t res = pmic_read_reg(0x02, &value.raw);
    if (res != PMIC_OK) return res;
    if (out_busy) {
        *out_busy = value.conv_start;
    }
    if (out_continuous) {
        *out_continuous = value.conv_rate;
    }
    return PMIC_OK;
}

// REG03
pmic_result_t pmic_set_minimum_system_voltage_limit(uint16_t millivolt) {
    bq25895_reg03_t value;
    pmic_result_t res = pmic_read_reg(0x03, &value.raw);
    if (res != PMIC_OK) return res;

    value.sys_min = 0;

    if (millivolt < 3000) {
        millivolt = 3000;  // Minimum
    }
    millivolt -= 3000;  // Offset

    if (millivolt > 400) {
        millivolt -= 400;
        value.sys_min |= (1 << 2);
    }

    if (millivolt > 200) {
        millivolt -= 200;
        value.sys_min |= (1 << 1);
    }

    if (millivolt > 100) {
        millivolt -= 100;
        value.sys_min |= (1 << 0);
    }

    return pmic_write_reg(0x03, value.raw);
}

pmic_result_t pmic_get_minimum_system_voltage_limit(uint16_t* out_millivolt) {
    bq25895_reg03_t value;
    pmic_result_t res = pmic_read_reg(0x03, &value.raw);
    if (res != PMIC_OK) return res;

    uint16_t millivolt = 3000;

    if (value.sys_min & (1 << 2)) {
        millivolt += 400;
    }

    if (value.sys_min & (1 << 1)) {
        millivolt += 200;
    }

    if (value.sys_min & (1 << 0)) {
        millivolt += 100;
    }

    if (out_millivolt) {
        *out_millivolt = millivolt;
    }

    return PMIC_OK;
}

pmic_result_t pmic_set_charge_enable(bool enable) {
    bq25895_reg03_t value;
    pmic_result_t res = pmic_read_reg(0x03, &value.raw);
    if (res != PMIC_OK) return res;
    value.chg_config = enable;
    return pmic_write_reg(0x03, value.raw);
}

pmic_result_t pmic_get_charge_enable(bool* out_enable) {
    bq25895_reg03_t value;
    pmic_result_t res = pmic_read_reg(0x03, &value.raw);
    if (res != PMIC_OK) return res;
    if (out_enable) {
        *out_enable = value.chg_config;
    }
    return PMIC_OK;
}

pmic_result_t pmic_set_otg_enable(bool enable) {
    bq25895_reg03_t value;
    pmic_result_t res = pmic_read_reg(0x03, &value.raw);
    if (res != PMIC_OK) return res;
    value.otg_config = enable;
    return pmic_write_reg(0x03, value.raw);
}

pmic_result_t pmic_get_otg_enable(bool* out_enable) {
    bq25895_reg03_t value;
    pmic_result_t res = pmic_read_reg(0x03, &value.raw);
    if (res != PMIC_OK) return res;
    if (out_enable) {
        *out_enable = value.otg_config;
    }
    return PMIC_OK;
}

pmic_result_t pmic_reset_watchdog(void) {
    bq25895_reg03_t value;
    pmic_result_t res = pmic_read_reg(0x03, &value.raw);
    if (res != PMIC_OK) return res;
    value.wd_rst = true;
    return pmic_write_reg(0x03, value.raw);
}

pmic_result_t pmic_set_battery_load_enable(bool enable) {
    // Enabling this puts a 30mA load on the battery
    // for battery presence detection
    bq25895_reg03_t value;
    pmic_result_t res = pmic_read_reg(0x03, &value.raw);
    if (res != PMIC_OK) return res;
    value.bat_loaden = enable;
    return pmic_write_reg(0x03, value.raw);
}

pmic_result_t pmic_get_battery_load_enable(bool* out_enable) {
    bq25895_reg03_t value;
    pmic_result_t res = pmic_read_reg(0x03, &value.raw);
    if (res != PMIC_OK) return res;
    if (out_enable) {
        *out_enable = value.bat_loaden;
    }
    return PMIC_OK;
}

// REG04

pmic_result_t pmic_set_charge_current_fast(uint16_t current) {
    bq25895_reg04_t value;
    pmic_result_t res = pmic_read_reg(0x04, &value.raw);
    if (res != PMIC_OK) return res;
    value.ichg = 0;
    if (current >= 4096) {
        current -= 4096;
        value.ichg |= (1 << 6);  // Add 4096mA
    }
    if (current >= 2048) {
        current -= 2048;
        value.ichg |= (1 << 5);  // Add 2048mA
    }
    if (current >= 1024) {
        current -= 1024;
        value.ichg |= (1 << 4);  // Add 1024mA
    }
    if (current >= 512) {
        current -= 512;
        value.ichg |= (1 << 3);  // Add 512mA
    }
    if (current >= 256) {
        current -= 256;
        value.ichg |= (1 << 2);  // Add 256mA
    }
    if (current >= 128) {
        current -= 128;
        value.ichg |= (1 << 1);  // Add 128mA
    }
    if (current >= 64) {
        current -= 64;
        value.ichg |= (1 << 0);  // Add 64mA
    }
    return pmic_write_reg(0x04, value.raw);
}

pmic_result_t pmic_get_fast_charge_current(uint16_t* out_current) {
    bq25895_reg04_t value;
    pmic_result_t res = pmic_read_reg(0x04, &value.raw);
    if (res != PMIC_OK) return res;
    if (out_current) {
        uint16_t current = 0;
        if (value.ichg & (1 << 6)) {
            current += 4096;
        }
        if (value.ichg & (1 << 5)) {
            current += 2048;
        }
        if (value.ichg & (1 << 4)) {
            current += 1024;
        }
        if (value.ichg & (1 << 3)) {
            current += 512;
        }
        if (value.ichg & (1 << 2)) {
            current += 256;
        }
        if (value.ichg & (1 << 1)) {
            current += 128;
        }
        if (value.ichg & (1 << 0)) {
            current += 64;
        }
        *out_current = current;
    }
    return PMIC_OK;
}

pmic_result_t pmic_set_pumpx_enable(bool enable) {
    bq25895_reg04_t value;
    pmic_result_t res = pmic_read_reg(0x04, &value.raw);
    if (res != PMIC_OK) return res;
    value.en_pumpx = enable;
    return pmic_write_reg(0x04, value.raw);
}

pmic_result_t pmic_get_pumpx_enable(bool* out_enable) {
    bq25895_reg04_t value;
    pmic_result_t res = pmic_read_reg(0x04, &value.raw);
    if (res != PMIC_OK) return res;
    if (out_enable) {
        *out_enable = value.en_pumpx;
    }
    return PMIC_OK;
}

// REG05
pmic_result_t pmic_set_termination_current(uint16_t current) {
    bq25895_reg05_t value;
    pmic_result_t res = pmic_read_reg(0x05, &value.raw);
    if (res != PMIC_OK) return res;
    if (current < 64) {
        current = 64;
    }
    value.iterm = 0;
    current -= 64;  // Offset
    if (current > 512) {
        current -= 512;
        value.iterm |= (1 << 3);
    }
    if (current > 256) {
        current -= 256;
        value.iterm |= (1 << 2);
    }
    if (current > 128) {
        current -= 128;
        value.iterm |= (1 << 1);
    }
    if (current > 64) {
        current -= 64;
        value.iterm |= (1 << 0);
    }
    return pmic_write_reg(0x05, value.raw);
}

pmic_result_t pmic_get_termination_current(uint16_t* out_current) {
    bq25895_reg05_t value;
    pmic_result_t res = pmic_read_reg(0x05, &value.raw);
    if (res != PMIC_OK) return res;
    if (out_current) {
        uint16_t current = 64;
        if (value.iterm & (1 << 3)) {
            current += 512;
        }
        if (value.iterm & (1 << 2)) {
            current += 256;
        }
        if (value.iterm & (1 << 1)) {
            current += 128;
        }
        if (value.iterm & (1 << 0)) {
            current += 64;
        }
        *out_current = current;
    }
    return PMIC_OK;
}

pmic_result_t pmic_set_precharge_current(uint16_t current) {
    bq25895_reg05_t value;
    pmic_result_t res = pmic_read_reg(0x05, &value.raw);
    if (current < 64) {
        current = 64;
    }
    value.iprechg = 0;
    current -= 64;  // Offset
    if (current > 512) {
        current -= 512;
        value.iprechg |= (1 << 3);
    }
    if (current > 256) {
        current -= 256;
        value.iprechg |= (1 << 2);
    }
    if (current > 128) {
        current -= 128;
        value.iprechg |= (1 << 1);
    }
    if (current > 64) {
        current -= 64;
        value.iprechg |= (1 << 0);
    }
    if (res != PMIC_OK) return res;
    return pmic_write_reg(0x05, value.raw);
}

pmic_result_t pmic_get_precharge_current(uint16_t* out_current) {
    bq25895_reg05_t value;
    pmic_result_t res = pmic_read_reg(0x05, &value.raw);
    if (res != PMIC_OK) return res;
    if (out_current) {
        uint16_t current = 64;
        if (value.iprechg & (1 << 3)) {
            current += 512;
        }
        if (value.iprechg & (1 << 2)) {
            current += 256;
        }
        if (value.iprechg & (1 << 1)) {
            current += 128;
        }
        if (value.iprechg & (1 << 0)) {
            current += 64;
        }
        *out_current = current;
    }
    return PMIC_OK;
}

// REG06
pmic_result_t pmic_set_charge_recharge_threshold_200mv_offset(bool enable) {
    // Battery recharge threshold offset
    // 100mV below target voltage if disabled, 200mV if enabled
    bq25895_reg06_t value;
    pmic_result_t res = pmic_read_reg(0x06, &value.raw);
    if (res != PMIC_OK) return res;
    value.vrechg = enable;
    return pmic_write_reg(0x06, value.raw);
}

pmic_result_t pmic_get_charge_recharge_threshold_200mv_offset(bool* out_enable) {
    // Battery recharge threshold offset
    // 100mV below target voltage if disabled, 200mV if enabled
    bq25895_reg06_t value;
    pmic_result_t res = pmic_read_reg(0x06, &value.raw);
    if (res != PMIC_OK) return res;
    if (out_enable) {
        *out_enable = value.vrechg;
    }
    return PMIC_OK;
}

pmic_result_t pmic_set_charge_battery_precharge_threshold_3v(bool enable) {
    // Threshold between precharge and fast charge
    // 2.8v if disabled, 3v if enabled
    bq25895_reg06_t value;
    pmic_result_t res = pmic_read_reg(0x06, &value.raw);
    if (res != PMIC_OK) return res;
    value.batlowv = enable;
    return pmic_write_reg(0x06, value.raw);
}

pmic_result_t pmic_get_charge_battery_precharge_threshold_3v(bool* out_enable) {
    // Threshold between precharge and fast charge
    // 2.8v if disabled, 3v if enabled
    bq25895_reg06_t value;
    pmic_result_t res = pmic_read_reg(0x06, &value.raw);
    if (res != PMIC_OK) return res;
    if (out_enable) {
        *out_enable = value.batlowv;
    }
    return PMIC_OK;
}

pmic_result_t pmic_set_charge_voltage_limit(uint16_t voltage) {
    bq25895_reg06_t value;
    pmic_result_t res = pmic_read_reg(0x06, &value.raw);
    if (res != PMIC_OK) return res;
    value.vreg = 0;
    if (voltage < 3840) {
        voltage = 3840;
    }
    voltage -= 3840;  // Offset
    if (voltage > 512) {
        voltage -= 512;
        value.vreg |= (1 << 5);
    }
    if (voltage > 256) {
        voltage -= 256;
        value.vreg |= (1 << 4);
    }
    if (voltage > 128) {
        voltage -= 128;
        value.vreg |= (1 << 3);
    }
    if (voltage > 64) {
        voltage -= 64;
        value.vreg |= (1 << 2);
    }
    if (voltage > 32) {
        voltage -= 32;
        value.vreg |= (1 << 1);
    }
    if (voltage > 16) {
        voltage -= 16;
        value.vreg |= (1 << 0);
    }
    return pmic_write_reg(0x06, value.raw);
}

pmic_result_t pmic_get_charge_voltage_limit(uint16_t* out_voltage) {
    bq25895_reg06_t value;
    pmic_result_t res = pmic_read_reg(0x06, &value.raw);
    if (res != PMIC_OK) return res;
    if (out_voltage) {
        uint16_t voltage = 3840;
        if (value.vreg & (1 << 5)) {
            voltage += 512;
        }
        if (value.vreg & (1 << 4)) {
            voltage += 256;
        }
        if (value.vreg & (1 << 3)) {
            voltage += 128;
        }
        if (value.vreg & (1 << 2)) {
            voltage += 64;
        }
        if (value.vreg & (1 << 1)) {
            voltage += 32;
        }
        if (value.vreg & (1 << 0)) {
            voltage += 16;
        }
    }
    return PMIC_OK;
}

// REG07
pmic_result_t pmic_set_charge_timer_limit(uint8_t hours) {
    bq25895_reg07_t value;
    pmic_result_t res = pmic_read_reg(0x07, &value.raw);
    if (res != PMIC_OK) return res;
    value.chg_timer = 0;
    if (hours >= 20) {
        value.chg_timer = 3;
    } else if (hours >= 12) {
        value.chg_timer = 2;
    } else if (hours >= 8) {
        value.chg_timer = 1;
    }
    return pmic_write_reg(0x07, value.raw);
}

pmic_result_t pmic_get_charge_timer_limit(uint8_t* out_hours) {
    bq25895_reg07_t value;
    pmic_result_t res = pmic_read_reg(0x07, &value.raw);
    if (out_hours) {
        if (value.chg_timer == 1) {
            *out_hours = 8;
        } else if (value.chg_timer == 2) {
            *out_hours = 12;
        } else if (value.chg_timer == 3) {
            *out_hours = 20;
        } else {
            *out_hours = 5;
        }
    }
    if (res != PMIC_OK) return res;
    return PMIC_OK;
}

pmic_result_t pmic_set_charge_safety_timer_enable(bool enable) {
    bq25895_reg07_t value;
    pmic_result_t res = pmic_read_reg(0x07, &value.raw);
    if (res != PMIC_OK) return res;
    value.en_timer = enable;
    return pmic_write_reg(0x07, value.raw);
}

pmic_result_t pmic_get_charge_safety_timer_enable(bool* out_enable) {
    bq25895_reg07_t value;
    pmic_result_t res = pmic_read_reg(0x07, &value.raw);
    if (res != PMIC_OK) return res;
    if (out_enable) {
        *out_enable = value.en_timer;
    }
    return PMIC_OK;
}

pmic_result_t pmic_set_watchdog_timer_limit(uint8_t seconds) {
    // Set to zero to disable the watchdog timer
    bq25895_reg07_t value;
    pmic_result_t res = pmic_read_reg(0x07, &value.raw);
    if (res != PMIC_OK) return res;
    value.watchdog = 0;  // Disable
    if (seconds >= 160) {
        value.watchdog = 3;
    } else if (seconds >= 80) {
        value.watchdog = 2;
    } else if (seconds >= 40) {
        value.watchdog = 1;
    }
    return pmic_write_reg(0x07, value.raw);
}

pmic_result_t pmic_get_watchdog_timer_limit(uint8_t* out_seconds) {
    bq25895_reg07_t value;
    pmic_result_t res = pmic_read_reg(0x07, &value.raw);
    if (res != PMIC_OK) return res;
    if (out_seconds) {
        if (value.watchdog == 1) {
            *out_seconds = 40;
        } else if (value.watchdog == 2) {
            *out_seconds = 80;
        } else if (value.watchdog == 3) {
            *out_seconds = 160;
        } else {
            *out_seconds = 0;
        }
    }
    return PMIC_OK;
}

pmic_result_t pmic_set_status_pin_disable(bool disable) {
    bq25895_reg07_t value;
    pmic_result_t res = pmic_read_reg(0x07, &value.raw);
    if (res != PMIC_OK) return res;
    value.stat_dis = disable;
    return pmic_write_reg(0x07, value.raw);
}

pmic_result_t pmic_get_status_pin_disable(bool* out_disable) {
    bq25895_reg07_t value;
    pmic_result_t res = pmic_read_reg(0x07, &value.raw);
    if (res != PMIC_OK) return res;
    if (out_disable) {
        *out_disable = value.stat_dis;
    }
    return PMIC_OK;
}

pmic_result_t pmic_set_charge_termination_enable(bool enable) {
    bq25895_reg07_t value;
    pmic_result_t res = pmic_read_reg(0x07, &value.raw);
    if (res != PMIC_OK) return res;
    value.en_term = enable;
    return pmic_write_reg(0x07, value.raw);
}

pmic_result_t pmic_get_charge_termination_enable(bool* out_enable) {
    bq25895_reg07_t value;
    pmic_result_t res = pmic_read_reg(0x07, &value.raw);
    if (res != PMIC_OK) return res;
    if (out_enable) {
        *out_enable = value.en_term;
    }
    return PMIC_OK;
}

// REG08
pmic_result_t pmic_set_thermal_regulation_threshold(uint8_t temperature) {
    bq25895_reg08_t value;
    pmic_result_t res = pmic_read_reg(0x08, &value.raw);
    if (res != PMIC_OK) return res;
    value.treg = 0;  // 60 degrees
    if (temperature >= 120) {
        value.treg = 3;
    } else if (temperature >= 100) {
        value.treg = 2;
    } else if (temperature >= 80) {
        value.treg = 1;
    }
    return pmic_write_reg(0x08, value.raw);
}

pmic_result_t pmic_get_thermal_regulation_threshold(uint8_t* out_temperature) {
    bq25895_reg08_t value;
    pmic_result_t res = pmic_read_reg(0x08, &value.raw);
    if (res != PMIC_OK) return res;
    if (out_temperature) {
        if (value.treg == 1) {
            *out_temperature = 80;
        } else if (value.treg == 2) {
            *out_temperature = 100;
        } else if (value.treg == 3) {
            *out_temperature = 120;
        } else {
            *out_temperature = 60;
        }
    }
    return PMIC_OK;
}

pmic_result_t pmic_set_ir_compensation_voltage_clamp(uint16_t millivolt) {
    bq25895_reg08_t value;
    pmic_result_t res = pmic_read_reg(0x08, &value.raw);
    if (res != PMIC_OK) return res;
    value.vclamp = 0;
    if (millivolt >= 128) {
        millivolt -= 128;
        value.vclamp |= (1 << 2);
    }
    if (millivolt >= 64) {
        millivolt -= 64;
        value.vclamp |= (1 << 1);
    }
    if (millivolt >= 32) {
        millivolt -= 32;
        value.vclamp |= (1 << 0);
    }
    return pmic_write_reg(0x08, value.raw);
}

pmic_result_t pmic_get_ir_compensation_voltage_clamp(uint16_t* out_millivolt) {
    bq25895_reg08_t value;
    pmic_result_t res = pmic_read_reg(0x08, &value.raw);
    if (res != PMIC_OK) return res;
    if (out_millivolt) {
        *out_millivolt = 0;
        if (value.vclamp & (1 << 2)) {
            *out_millivolt += 128;
        }
        if (value.vclamp & (1 << 1)) {
            *out_millivolt += 64;
        }
        if (value.vclamp & (1 << 0)) {
            *out_millivolt += 32;
        }
    }
    return PMIC_OK;
}

pmic_result_t pmic_set_ir_compensation_resistor_setting(uint16_t milliohm) {
    bq25895_reg08_t value;
    pmic_result_t res = pmic_read_reg(0x08, &value.raw);
    if (res != PMIC_OK) return res;
    value.bat_comp = 0;
    if (milliohm >= 80) {
        milliohm -= 80;
        value.bat_comp |= (1 << 2);
    }
    if (milliohm >= 40) {
        milliohm -= 40;
        value.bat_comp |= (1 << 1);
    }
    if (milliohm >= 20) {
        milliohm -= 20;
        value.bat_comp |= (1 << 0);
    }
    return pmic_write_reg(0x08, value.raw);
}

pmic_result_t pmic_get_ir_compensation_resistor_setting(uint16_t* out_milliohm) {
    bq25895_reg08_t value;
    pmic_result_t res = pmic_read_reg(0x08, &value.raw);
    if (res != PMIC_OK) return res;
    if (out_milliohm) {
        *out_milliohm = 0;
        if (value.bat_comp & (1 << 2)) {
            *out_milliohm += 80;
        }
        if (value.bat_comp & (1 << 1)) {
            *out_milliohm += 40;
        }
        if (value.bat_comp & (1 << 0)) {
            *out_milliohm += 20;
        }
    }
    return PMIC_OK;
}

// REG09
pmic_result_t pmic_set_pumpx_down_enable(bool enable) {
    bq25895_reg09_t value;
    pmic_result_t res = pmic_read_reg(0x09, &value.raw);
    if (res != PMIC_OK) return res;
    value.pumpx_dn = enable;
    return pmic_write_reg(0x09, value.raw);
}

pmic_result_t pmic_get_pumpx_down_enable(bool* out_enable) {
    bq25895_reg09_t value;
    pmic_result_t res = pmic_read_reg(0x09, &value.raw);
    if (res != PMIC_OK) return res;
    if (out_enable) {
        *out_enable = value.pumpx_dn;
    }
    return PMIC_OK;
}

pmic_result_t pmic_set_pumpx_up_enable(bool enable) {
    bq25895_reg09_t value;
    pmic_result_t res = pmic_read_reg(0x09, &value.raw);
    if (res != PMIC_OK) return res;
    value.pumpx_up = enable;
    return pmic_write_reg(0x09, value.raw);
}

pmic_result_t pmic_get_pumpx_up_enable(bool* out_enable) {
    bq25895_reg09_t value;
    pmic_result_t res = pmic_read_reg(0x09, &value.raw);
    if (res != PMIC_OK) return res;
    if (out_enable) {
        *out_enable = value.pumpx_up;
    }
    return PMIC_OK;
}

pmic_result_t pmic_set_battery_full_system_reset_enable(bool enable) {
    bq25895_reg09_t value;
    pmic_result_t res = pmic_read_reg(0x09, &value.raw);
    if (res != PMIC_OK) return res;
    value.batfet_rst_en = enable;
    return pmic_write_reg(0x09, value.raw);
}

pmic_result_t pmic_get_battery_full_system_reset_enable(bool* out_enable) {
    bq25895_reg09_t value;
    pmic_result_t res = pmic_read_reg(0x09, &value.raw);
    if (res != PMIC_OK) return res;
    if (out_enable) {
        *out_enable = value.batfet_rst_en;
    }
    return PMIC_OK;
}

pmic_result_t pmic_set_battery_disconnect_delay_enable(bool enable) {
    bq25895_reg09_t value;
    pmic_result_t res = pmic_read_reg(0x09, &value.raw);
    if (res != PMIC_OK) return res;
    value.batfet_dly = enable;
    return pmic_write_reg(0x09, value.raw);
}

pmic_result_t pmic_get_battery_disconnect_delay_enable(bool* out_enable) {
    bq25895_reg09_t value;
    pmic_result_t res = pmic_read_reg(0x09, &value.raw);
    if (res != PMIC_OK) return res;
    if (out_enable) {
        *out_enable = value.batfet_dly;
    }
    return PMIC_OK;
}

pmic_result_t pmic_set_battery_disconnect_enable(bool disconnect) {
    bq25895_reg09_t value;
    pmic_result_t res = pmic_read_reg(0x09, &value.raw);
    if (res != PMIC_OK) return res;
    value.batfet_dis = disconnect;
    return pmic_write_reg(0x09, value.raw);
}

pmic_result_t pmic_get_battery_disconnect_enable(bool* out_disconnected) {
    bq25895_reg09_t value;
    pmic_result_t res = pmic_read_reg(0x09, &value.raw);
    if (res != PMIC_OK) return res;
    if (out_disconnected) {
        *out_disconnected = value.batfet_dis;
    }
    return PMIC_OK;
}

pmic_result_t pmic_set_safety_timer_slow_2x_enable(bool enable) {
    bq25895_reg09_t value;
    pmic_result_t res = pmic_read_reg(0x09, &value.raw);
    if (res != PMIC_OK) return res;
    value.tmr2x_en = enable;
    return pmic_write_reg(0x09, value.raw);
}

pmic_result_t pmic_get_safety_timer_slow_2x_enable(bool* out_enable) {
    bq25895_reg09_t value;
    pmic_result_t res = pmic_read_reg(0x09, &value.raw);
    if (res != PMIC_OK) return res;
    if (out_enable) {
        *out_enable = value.tmr2x_en;
    }
    return PMIC_OK;
}

pmic_result_t pmic_set_force_ico_enable(bool force) {
    bq25895_reg09_t value;
    pmic_result_t res = pmic_read_reg(0x09, &value.raw);
    if (res != PMIC_OK) return res;
    value.force_ico = force;
    return pmic_write_reg(0x09, value.raw);
}

pmic_result_t pmic_get_force_ico_enable(bool* out_forced) {
    bq25895_reg09_t value;
    pmic_result_t res = pmic_read_reg(0x09, &value.raw);
    if (res != PMIC_OK) return res;
    if (out_forced) {
        *out_forced = value.force_ico;
    }
    return PMIC_OK;
}

// REG0A
pmic_result_t pmic_set_boost_mode_voltage(uint16_t millivolt) {
    bq25895_reg0A_t value;
    pmic_result_t res = pmic_read_reg(0x0A, &value.raw);
    if (res != PMIC_OK) return res;
    value.boostv = 0;
    if (millivolt < 4550) {
        millivolt = 4550;
    }
    millivolt -= 4550;  // Offset
    if (millivolt >= 512) {
        millivolt -= 512;
        value.boostv |= (1 << 3);
    }
    if (millivolt >= 256) {
        millivolt -= 256;
        value.boostv |= (1 << 2);
    }
    if (millivolt >= 128) {
        millivolt -= 128;
        value.boostv |= (1 << 1);
    }
    if (millivolt >= 64) {
        millivolt -= 64;
        value.boostv |= (1 << 0);
    }
    return pmic_write_reg(0x0A, value.raw);
}

pmic_result_t pmic_get_boost_mode_voltage(uint16_t* out_millivolt) {
    bq25895_reg0A_t value;
    pmic_result_t res = pmic_read_reg(0x0A, &value.raw);
    if (res != PMIC_OK) return res;
    if (out_millivolt) {
        *out_millivolt = 4550;
        if (value.boostv & (1 << 3)) {
            *out_millivolt += 512;
        }
        if (value.boostv & (1 << 2)) {
            *out_millivolt += 256;
        }
        if (value.boostv & (1 << 1)) {
            *out_millivolt += 128;
        }
        if (value.boostv & (1 << 0)) {
            *out_millivolt += 64;
        }
    }
    return PMIC_OK;
}

// REG0B
pmic_result_t pmic_get_vsys_regulation_status(bool* out_active) {
    bq25895_reg0B_t value;
    pmic_result_t res = pmic_read_reg(0x0B, &value.raw);
    if (res != PMIC_OK) return res;
    if (out_active) {
        *out_active = value.vsys_stat;
    }
    return PMIC_OK;
}

pmic_result_t pmic_get_usb_input_sdp_status(bool* out_usb500) {
    bq25895_reg0B_t value;
    pmic_result_t res = pmic_read_reg(0x0B, &value.raw);
    if (res != PMIC_OK) return res;
    if (out_usb500) {
        *out_usb500 = value.sdp_stat;
    }
    return PMIC_OK;
}

pmic_result_t pmic_get_power_good_status(bool* out_active) {
    bq25895_reg0B_t value;
    pmic_result_t res = pmic_read_reg(0x0B, &value.raw);
    if (res != PMIC_OK) return res;
    if (out_active) {
        *out_active = value.pg_stat;
    }
    return PMIC_OK;
}

pmic_result_t pmic_get_charge_status(pmic_charge_status_t* out_status) {
    bq25895_reg0B_t value;
    pmic_result_t res = pmic_read_reg(0x0B, &value.raw);
    if (res != PMIC_OK) return res;
    if (out_status) {
        *out_status = (pmic_charge_status_t)value.chrg_stat;
    }
    return PMIC_OK;
}

pmic_result_t pmic_get_vbus_status(pmic_vbus_status_t* out_status) {
    bq25895_reg0B_t value;
    pmic_result_t res = pmic_read_reg(0x0B, &value.raw);
    if (res != PMIC_OK) return res;
    if (out_status) {
        *out_status = (pmic_vbus_status_t)value.vbus_stat;
    }
    return PMIC_OK;
}

// REG0C
pmic_result_t pmic_get_faults(uint8_t* out_raw, pmic_faults_t* out_faults) {
    bq25895_reg0C_t value = {0};
    pmic_result_t res = pmic_read_reg(0x0C, &value.raw);
    if (res != PMIC_OK) return res;

    if (out_raw) {
        *out_raw = value.raw;
    }

    if (out_faults) {
        out_faults->watchdog = value.watchdog_fault;
        out_faults->boost = value.boost_fault;
        out_faults->chrg_input = value.chrg_fault == 1;
        out_faults->chrg_thermal = value.chrg_fault == 2;
        out_faults->chrg_safety = value.chrg_fault == 3;
        out_faults->batt_ovp = value.bat_fault;
        out_faults->ntc_cold = (value.ntc_fault & 3) == 1;
        out_faults->ntc_hot = (value.ntc_fault & 3) == 2;
        out_faults->ntc_boost = (value.ntc_fault >> 2) & 1;
    }

    return PMIC_OK;
}

// REG0D
pmic_result_t pmic_set_vindpm(uint16_t millivolt) {
    bq25895_reg0D_t value;
    pmic_result_t res = pmic_read_reg(0x0D, &value.raw);
    if (res != PMIC_OK) return res;
    value.vindpm = 0;
    if (millivolt < 2600) {
        millivolt = 2600;
    }
    millivolt -= 2600;
    if (millivolt >= 6400) {
        millivolt -= 6400;
        value.vindpm |= (1 << 6);
    }
    if (millivolt >= 3200) {
        millivolt -= 3200;
        value.vindpm |= (1 << 5);
    }
    if (millivolt >= 1600) {
        millivolt -= 1600;
        value.vindpm |= (1 << 4);
    }
    if (millivolt >= 800) {
        millivolt -= 800;
        value.vindpm |= (1 << 3);
    }
    if (millivolt >= 400) {
        millivolt -= 400;
        value.vindpm |= (1 << 2);
    }
    if (millivolt >= 200) {
        millivolt -= 200;
        value.vindpm |= (1 << 1);
    }
    if (millivolt >= 100) {
        millivolt -= 100;
        value.vindpm |= (1 << 0);
    }
    return pmic_write_reg(0x0D, value.raw);
}

pmic_result_t pmic_get_vindpm(uint16_t* out_millivolt) {
    bq25895_reg0D_t value;
    pmic_result_t res = pmic_read_reg(0x0D, &value.raw);
    if (res != PMIC_OK) return res;
    if (out_millivolt) {
        *out_millivolt = 2600;
        if (value.vindpm & (1 << 6)) {
            *out_millivolt += 6400;
        }
        if (value.vindpm & (1 << 5)) {
            *out_millivolt += 3200;
        }
        if (value.vindpm & (1 << 4)) {
            *out_millivolt += 1600;
        }
        if (value.vindpm & (1 << 3)) {
            *out_millivolt += 800;
        }
        if (value.vindpm & (1 << 2)) {
            *out_millivolt += 400;
        }
        if (value.vindpm & (1 << 1)) {
            *out_millivolt += 200;
        }
        if (value.vindpm & (1 << 0)) {
            *out_millivolt += 100;
        }
    }
    return PMIC_OK;
}

pmic_result_t pmic_set_force_vindpm(bool absolute) {
    bq25895_reg0D_t value;
    pmic_result_t res = pmic_read_reg(0x0D, &value.raw);
    if (res != PMIC_OK) return res;
    value.force_vindpm = absolute;
    return pmic_write_reg(0x0D, value.raw);
}

pmic_result_t pmic_get_force_vindpm(bool* out_absolute) {
    bq25895_reg0D_t value;
    pmic_result_t res = pmic_read_reg(0x0D, &value.raw);
    if (res != PMIC_OK) return res;
    if (out_absolute) {
        *out_absolute = value.force_vindpm;
    }
    return PMIC_OK;
}

// REG0E
pmic_result_t pmic_get_adc_vbat(uint16_t* out_vbat, bool* out_therm_stat) {
    bq25895_reg0E_t value;
    pmic_result_t res = pmic_read_reg(0x0E, &value.raw);
    if (res != PMIC_OK) return res;

    if (out_vbat) {
        uint16_t vbat = 2304;
        if ((value.vbat >> 6) & 1) vbat += 1280;
        if ((value.vbat >> 5) & 1) vbat += 640;
        if ((value.vbat >> 4) & 1) vbat += 320;
        if ((value.vbat >> 3) & 1) vbat += 160;
        if ((value.vbat >> 2) & 1) vbat += 80;
        if ((value.vbat >> 1) & 1) vbat += 40;
        if ((value.vbat >> 0) & 1) vbat += 20;
        *out_vbat = vbat;
    }

    if (out_therm_stat) {
        *out_therm_stat = value.therm_stat;
    }

    return PMIC_OK;
}

pmic_result_t pmic_get_thermal_regulation_status(bool* out_therm_stat) {
    bq25895_reg0E_t value;
    pmic_result_t res = pmic_read_reg(0x0E, &value.raw);
    if (res != PMIC_OK) return res;
    if (out_therm_stat) {
        *out_therm_stat = value.therm_stat;
    }
    return PMIC_OK;
}

// REG0F
pmic_result_t pmic_get_adc_vsys(uint16_t* out_vsys) {
    bq25895_reg0F_t value;
    pmic_result_t res = pmic_read_reg(0x0F, &value.raw);
    if (res != PMIC_OK) return res;

    if (out_vsys) {
        uint16_t vsys = 2304;
        if ((value.vsys >> 6) & 1) vsys += 1280;
        if ((value.vsys >> 5) & 1) vsys += 640;
        if ((value.vsys >> 4) & 1) vsys += 320;
        if ((value.vsys >> 3) & 1) vsys += 160;
        if ((value.vsys >> 2) & 1) vsys += 80;
        if ((value.vsys >> 1) & 1) vsys += 40;
        if ((value.vsys >> 0) & 1) vsys += 20;
        *out_vsys = vsys;
    }

    return PMIC_OK;
}

// REG10
pmic_result_t pmic_get_adc_tspct(uint16_t* out_tspct) {
    // Divide the output of this function by 100 to get the percentage
    bq25895_reg10_t value;
    pmic_result_t res = pmic_read_reg(0x10, &value.raw);
    if (res != PMIC_OK) return res;

    if (out_tspct) {
        uint16_t tspct = 2100;
        if ((value.tspct >> 6) & 1) tspct += 2976;
        if ((value.tspct >> 5) & 1) tspct += 1488;
        if ((value.tspct >> 4) & 1) tspct += 744;
        if ((value.tspct >> 3) & 1) tspct += 372;
        if ((value.tspct >> 2) & 1) tspct += 186;
        if ((value.tspct >> 1) & 1) tspct += 93;
        if ((value.tspct >> 0) & 1) tspct += 47;
        *out_tspct = tspct;
    }

    return PMIC_OK;
}

// REG11
pmic_result_t pmic_get_adc_vbus(uint16_t* out_vbus, bool* out_vbus_gd) {
    bq25895_reg11_t value;
    pmic_result_t res = pmic_read_reg(0x11, &value.raw);
    if (res != PMIC_OK) return res;

    if (out_vbus) {
        uint16_t vbus = 2600;
        if ((value.vbus >> 6) & 1) vbus += 6400;
        if ((value.vbus >> 5) & 1) vbus += 3200;
        if ((value.vbus >> 4) & 1) vbus += 1600;
        if ((value.vbus >> 3) & 1) vbus += 800;
        if ((value.vbus >> 2) & 1) vbus += 400;
        if ((value.vbus >> 1) & 1) vbus += 200;
        if ((value.vbus >> 0) & 1) vbus += 100;
        *out_vbus = vbus;
    }

    if (out_vbus_gd) {
        *out_vbus_gd = value.vbus_gd;
    }

    return PMIC_OK;
}

pmic_result_t pmic_get_vbus_good(bool* out_vbus_gd) {
    bq25895_reg11_t value;
    pmic_result_t res = pmic_read_reg(0x11, &value.raw);
    if (res != PMIC_OK) return res;
    if (out_vbus_gd) {
        *out_vbus_gd = value.vbus_gd;
    }
    return PMIC_OK;
}

// REG12
pmic_result_t pmic_get_adc_ichgr(uint16_t* out_ichgr) {
    bq25895_reg12_t value;
    pmic_result_t res = pmic_read_reg(0x12, &value.raw);
    if (res != PMIC_OK) return res;

    if (out_ichgr) {
        uint16_t ichgr = 0;
        if ((value.ichgr >> 6) & 1) ichgr += 3200;
        if ((value.ichgr >> 5) & 1) ichgr += 1600;
        if ((value.ichgr >> 4) & 1) ichgr += 800;
        if ((value.ichgr >> 3) & 1) ichgr += 400;
        if ((value.ichgr >> 2) & 1) ichgr += 200;
        if ((value.ichgr >> 1) & 1) ichgr += 100;
        if ((value.ichgr >> 0) & 1) ichgr += 50;
        *out_ichgr = ichgr;
    }

    return PMIC_OK;
}

/////////////////////////////

pmic_result_t pmic_power_off() {
    pmic_result_t res;
    res = pmic_get_watchdog_timer_limit(0);
    if (res != PMIC_OK) return res;
    res = pmic_set_adc_configuration(false, false);
    if (res != PMIC_OK) return res;
    res = pmic_set_otg_enable(false);
    if (res != PMIC_OK) return res;
    res = pmic_set_battery_load_enable(false);
    if (res != PMIC_OK) return res;
    res = pmic_set_battery_disconnect_enable(true);
    return res;
}

pmic_result_t pmic_battery_attached(bool* battery_attached, bool detect_empty_battery) {
    pmic_result_t res;
    *battery_attached = false;
    uint16_t vbatt = 0;
    bool therm_stat = false;

    // Algorithm 1: detect battery with normal voltage
    res = pmic_set_charge_enable(false);  // Disable charging
    if (res != PMIC_OK) return res;
    res = pmic_set_battery_load_enable(true);  // Apply 30mA load on battery
    if (res != PMIC_OK) return res;
    Delay_Ms(5);                                // Wait 5ms
    res = pmic_set_battery_load_enable(false);  // Disable 30mA load on battery
    if (res != PMIC_OK) return res;
    res = pmic_get_adc_vbat(&vbatt, &therm_stat);  // Measure battery voltage
    if (res != PMIC_OK) return res;
    if (vbatt >= 3000 && (!therm_stat)) {  // Assume attached if battery voltage 3v or higher
        *battery_attached = true;
        return res;
    } else if (therm_stat) {  // Battery temperature too high, ignore battery even if attached
        return res;
    }

    // Algorithm 2: detect empty battery by forcing closing of protection circuit
    if (detect_empty_battery) {
        res = pmic_set_minimum_system_voltage_limit(3700);  // Set minimum system voltage to 3.7v
        if (res != PMIC_OK) return res;
        res = pmic_set_charge_voltage_limit(3700);  // Set charging end voltage to 3.7v
        if (res != PMIC_OK) return res;
        res = pmic_set_charge_enable(true);  // Enable charging
        Delay_Ms(50);                        // Wait 50ms
        if (res != PMIC_OK) return res;
        res = pmic_set_charge_enable(false);  // Disable charging
        if (res != PMIC_OK) return res;
        res = pmic_set_battery_load_enable(true);  // Apply 30mA load on battery
        if (res != PMIC_OK) return res;
        Delay_Ms(5);                                // Wait 5ms
        res = pmic_set_battery_load_enable(false);  // Disable 30mA load on battery
        if (res != PMIC_OK) return res;
        res = pmic_get_adc_vbat(&vbatt, &therm_stat);  // Measure battery voltage
        if (res != PMIC_OK) return res;
        res = pmic_set_minimum_system_voltage_limit(3500);  // Set minimum system voltage back to the default 3.5v
        if (res != PMIC_OK) return res;
        if (vbatt >= 2800 && (!therm_stat)) {  // Assume attached if battery voltage 2.8v or higher
            *battery_attached = true;
            return res;
        }
    }

    return res;
}

pmic_result_t pmic_configure_battery_charger(bool enable, uint16_t current) {
    pmic_result_t res;
    res = pmic_set_battery_load_enable(false);  // Disable 30mA load on battery
    if (res != PMIC_OK) return res;
    if (enable) {
        res = pmic_set_charge_enable(true);  // Start charging
        if (res != PMIC_OK) return res;
        res = pmic_set_charge_battery_precharge_threshold_3v(true);  // Switch from precharge to fast charge at 3v
        if (res != PMIC_OK) return res;
        res = pmic_set_charge_recharge_threshold_200mv_offset(
            false);  // Recharge when battery voltage is 100mV below target
        if (res != PMIC_OK) return res;
        res = pmic_set_charge_voltage_limit(4200);  // Charge up to 4.2v
        if (res != PMIC_OK) return res;
        res = pmic_set_charge_current_fast(current);
        if (res != PMIC_OK) return res;
    } else {
        res = pmic_set_charge_enable(false);  // Stop charging
        if (res != PMIC_OK) return res;
    }
    return res;
}
