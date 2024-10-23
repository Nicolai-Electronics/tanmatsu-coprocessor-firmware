// Tanmatsu coprocessor firmware
// SPDX-FileCopyrightText: 2024 Nicolai Electronics
// SPDX-License-Identifier: MIT

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "ch32v003fun.h"
#include "i2c_master.h"
#include "i2c_slave.h"
#include "keyboard.h"
#include "pmic.h"
#include "rtc.h"

// Board revision
#define HW_REV 2

// Firmware version
#define FW_VERSION 8

#define OVERRIDE_C6  false
#define HARDWARE_REV 2  // 1 for prototype 1, 2 for prototype 2

// Pins
const uint8_t pin_c6_enable = PB8;
const uint8_t pin_c6_boot = (HARDWARE_REV > 1 ? PD1 : PB9);
const uint8_t pin_display_backlight = PB4;   // Note: change PWM timer configuration too when changing this pin
const uint8_t pin_keyboard_backlight = PB3;  // Note: change PWM timer configuration too when changing this pin
const uint8_t pin_interrupt = PA0;
const uint8_t pin_sdcard_detect = PA15;
const uint8_t pin_headphone_detect = PB5;
const uint8_t pin_amplifier_enable = PD0;
const uint8_t pin_power_out = PC13;  // Output to power button (for powering on using the RTC alarm)
const uint8_t pin_sda = PB7;         // Uses hardware I2C peripheral
const uint8_t pin_scl = PB6;         // Uses hardware I2C peripheral

#if HARDWARE_REV > 1
const uint8_t pin_camera = PA11;
const uint8_t pin_power_in = PA12;  // Input from power button
const uint8_t pin_pm_sda = PB11;
const uint8_t pin_pm_scl = PB10;
#endif

// Configuration
const uint16_t timer2_pwm_cycle_width = 255;       // Amount of brightness steps for keyboard backlight
const uint16_t timer3_pwm_cycle_width = 255;       // Amount of brightness steps for display backlight
static const uint32_t keyboard_scan_interval = 1;  // milliseconds (per row)
static const uint32_t input_scan_interval = 50;    // milliseconds

// I2C registers

typedef enum {
    I2C_REG_FW_VERSION_0 = 0,  // LSB
    I2C_REG_FW_VERSION_1,      // MSB
    I2C_REG_KEYBOARD_0,
    I2C_REG_KEYBOARD_1,
    I2C_REG_KEYBOARD_2,
    I2C_REG_KEYBOARD_3,
    I2C_REG_KEYBOARD_4,
    I2C_REG_KEYBOARD_5,
    I2C_REG_KEYBOARD_6,
    I2C_REG_KEYBOARD_7,
    I2C_REG_KEYBOARD_8,
    I2C_REG_DISPLAY_BACKLIGHT,
    I2C_REG_KEYBOARD_BACKLIGHT,
    I2C_REG_INTERRUPT,
    I2C_REG_RESERVED_0,
    I2C_REG_INPUT,  // SD card detect (bit 0) & headphone detect (bit 1)
    I2C_REG_OUTPUT,
    I2C_REG_RADIO_CONTROL,
    I2C_REG_RTC_VALUE_0,  // LSB
    I2C_REG_RTC_VALUE_1,
    I2C_REG_RTC_VALUE_2,
    I2C_REG_RTC_VALUE_3,
    I2C_REG_BACKUP_0,
    I2C_REG_BACKUP_1,
    I2C_REG_BACKUP_2,
    I2C_REG_BACKUP_3,
    I2C_REG_BACKUP_4,
    I2C_REG_BACKUP_5,
    I2C_REG_BACKUP_6,
    I2C_REG_BACKUP_7,
    I2C_REG_BACKUP_8,
    I2C_REG_BACKUP_9,
    I2C_REG_BACKUP_10,
    I2C_REG_BACKUP_11,
    I2C_REG_BACKUP_12,
    I2C_REG_BACKUP_13,
    I2C_REG_BACKUP_14,
    I2C_REG_BACKUP_15,
    I2C_REG_BACKUP_16,
    I2C_REG_BACKUP_17,
    I2C_REG_BACKUP_18,
    I2C_REG_BACKUP_19,
    I2C_REG_BACKUP_20,
    I2C_REG_BACKUP_21,
    I2C_REG_BACKUP_22,
    I2C_REG_BACKUP_23,
    I2C_REG_BACKUP_24,
    I2C_REG_BACKUP_25,
    I2C_REG_BACKUP_26,
    I2C_REG_BACKUP_27,
    I2C_REG_BACKUP_28,
    I2C_REG_BACKUP_29,
    I2C_REG_BACKUP_30,
    I2C_REG_BACKUP_31,
    I2C_REG_BACKUP_32,
    I2C_REG_BACKUP_33,
    I2C_REG_BACKUP_34,
    I2C_REG_BACKUP_35,
    I2C_REG_BACKUP_36,
    I2C_REG_BACKUP_37,
    I2C_REG_BACKUP_38,
    I2C_REG_BACKUP_39,
    I2C_REG_BACKUP_40,
    I2C_REG_BACKUP_41,
    I2C_REG_BACKUP_42,
    I2C_REG_BACKUP_43,
    I2C_REG_BACKUP_44,
    I2C_REG_BACKUP_45,
    I2C_REG_BACKUP_46,
    I2C_REG_BACKUP_47,
    I2C_REG_BACKUP_48,
    I2C_REG_BACKUP_49,
    I2C_REG_BACKUP_50,
    I2C_REG_BACKUP_51,
    I2C_REG_BACKUP_52,
    I2C_REG_BACKUP_53,
    I2C_REG_BACKUP_54,
    I2C_REG_BACKUP_55,
    I2C_REG_BACKUP_56,
    I2C_REG_BACKUP_57,
    I2C_REG_BACKUP_58,
    I2C_REG_BACKUP_59,
    I2C_REG_BACKUP_60,
    I2C_REG_BACKUP_61,
    I2C_REG_BACKUP_62,
    I2C_REG_BACKUP_63,
    I2C_REG_BACKUP_64,
    I2C_REG_BACKUP_65,
    I2C_REG_BACKUP_66,
    I2C_REG_BACKUP_67,
    I2C_REG_BACKUP_68,
    I2C_REG_BACKUP_69,
    I2C_REG_BACKUP_70,
    I2C_REG_BACKUP_71,
    I2C_REG_BACKUP_72,
    I2C_REG_BACKUP_73,
    I2C_REG_BACKUP_74,
    I2C_REG_BACKUP_75,
    I2C_REG_BACKUP_76,
    I2C_REG_BACKUP_77,
    I2C_REG_BACKUP_78,
    I2C_REG_BACKUP_79,
    I2C_REG_BACKUP_80,
    I2C_REG_BACKUP_81,
    I2C_REG_BACKUP_82,
    I2C_REG_BACKUP_83,
    I2C_REG_PMIC_COMM_FAULT,
    I2C_REG_PMIC_FAULT,
    I2C_REG_PMIC_ADC_CONTROL,
    I2C_REG_PMIC_ADC_VBAT_0,   // LSB (value in mV)
    I2C_REG_PMIC_ADC_VBAT_1,   // MSB
    I2C_REG_PMIC_ADC_VSYS_0,   // LSB (value in mV)
    I2C_REG_PMIC_ADC_VSYS_1,   // MSB
    I2C_REG_PMIC_ADC_TS_0,     // LSB (In % of REGN)
    I2C_REG_PMIC_ADC_TS_1,     // MSB
    I2C_REG_PMIC_ADC_VBUS_0,   // LSB (value in mV)
    I2C_REG_PMIC_ADC_VBUS_1,   // MSB
    I2C_REG_PMIC_ADC_ICHGR_0,  // LSB (value in mA)
    I2C_REG_PMIC_ADC_ICHGR_1,  // MSB
    I2C_REG_PMIC_CHARGING_CONTROL,
    I2C_REG_PMIC_CHARGING_STATUS,
    I2C_REG_PMIC_OTG_CONTROL,
    I2C_REG_LAST,  // End of list marker
} i2c_register_t;

volatile uint8_t i2c_registers[I2C_REG_LAST];

// Interrupt flags
volatile bool keyboard_interrupt = false;
volatile bool input_interrupt = false;
volatile bool pmic_interrupt = false;

// PMIC flags
volatile bool pmic_adc_trigger = false;
volatile bool pmic_adc_continuous = false;
volatile bool pmic_force_disable_charging = false;
volatile bool pmic_force_detect_battery = false;
volatile uint16_t pmic_target_charging_current = 512;

// Interrupts
void interrupt_update_reg(void) {
    i2c_registers[I2C_REG_INTERRUPT] =
        (keyboard_interrupt & 1) | ((input_interrupt & 1) << 1) | ((pmic_interrupt & 1) << 2);
}

void interrupt_set(bool keyboard, bool input, bool pmic) {
    if (keyboard) {
        keyboard_interrupt = true;
    }
    if (input) {
        input_interrupt = true;
    }
    if (pmic) {
        pmic_interrupt = true;
    }
    interrupt_update_reg();
}

void interrupt_clear(bool keyboard, bool input, bool pmic) {
    if (keyboard) {
        keyboard_interrupt = false;
    }
    if (input) {
        input_interrupt = false;
    }
    if (pmic) {
        pmic_interrupt = false;
    }
    interrupt_update_reg();
}

// Inputs
bool input_step() {
    static uint8_t previous_value = 0;

    uint8_t value = 0;
    value |= (!funDigitalRead(pin_sdcard_detect)) << 0;
    value |= funDigitalRead(pin_headphone_detect) << 1;

#if HARDWARE_REV > 1
    value |= (!funDigitalRead(pin_power_in)) << 2;
#endif

    i2c_registers[I2C_REG_INPUT] = value;

    bool changed = previous_value != value;
    previous_value = value;

    return changed;
}

// Timer 2: keyboard backlight PWM

void timer2_set(uint16_t value) {
    if (value > timer2_pwm_cycle_width) {
        value = timer2_pwm_cycle_width;
    }
    TIM2->CH2CVR = timer2_pwm_cycle_width - value;
    TIM2->SWEVGR |= TIM_UG;  // Apply
}

void timer2_init() {
    RCC->APB1PCENR |= RCC_APB1Periph_TIM2;               // Enable clock for timer 2
    AFIO->PCFR1 |= AFIO_PCFR1_TIM2_REMAP_PARTIALREMAP1;  // Partial mapping (PB3 as channel 2)
    funPinMode(pin_keyboard_backlight, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF);

    // Reset timer 3 peripheral
    RCC->APB1PRSTR |= RCC_APB1Periph_TIM2;
    RCC->APB1PRSTR &= ~RCC_APB1Periph_TIM2;

    TIM2->PSC = 0x10;                      // Clock prescaler divider
    TIM2->ATRLR = timer2_pwm_cycle_width;  // Total PWM cycle width

    TIM2->CHCTLR1 |= TIM_OC2M_2 | TIM_OC2M_1 | TIM_OC2PE;  // Enable channel 2

    TIM2->CTLR1 |= TIM_ARPE;  // Enable auto-reload of preload

    TIM2->CCER |= TIM_CC2E | TIM_CC2P;  // Enable channel 2 output, positive polarity

    timer2_set(0);  // Load default target PWM dutycycle

    TIM2->CTLR1 |= TIM_CEN;  // Enable timer
}

// Timer 3: display backlight PWM

void timer3_set(uint16_t value) {
    if (value > timer3_pwm_cycle_width) {
        value = timer3_pwm_cycle_width;
    }
    TIM3->CH1CVR = timer3_pwm_cycle_width - value;
    TIM3->SWEVGR |= TIM_UG;  // Apply
}

void timer3_init() {
    RCC->APB1PCENR |= RCC_APB1Periph_TIM3;              // Enable clock for timer 3
    AFIO->PCFR1 |= AFIO_PCFR1_TIM3_REMAP_PARTIALREMAP;  // Partial mapping (PB4 as channel 1)
    funPinMode(pin_display_backlight, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF);

    // Reset timer 3 peripheral
    RCC->APB1PRSTR |= RCC_APB1Periph_TIM3;
    RCC->APB1PRSTR &= ~RCC_APB1Periph_TIM3;

    TIM3->PSC = 0x10;                      // Clock prescaler divider
    TIM3->ATRLR = timer3_pwm_cycle_width;  // Total PWM cycle width

    TIM3->CHCTLR1 |= TIM_OC1M_2 | TIM_OC1M_1 | TIM_OC1PE;  // Enable channel 1

    TIM3->CTLR1 |= TIM_ARPE;  // Enable auto-reload of preload

    TIM3->CCER |= TIM_CC1E | TIM_CC1P;  // Enable channel 1 output, positive polarity

    timer3_set(255);  // Load default target PWM dutycycle

    TIM3->CTLR1 |= TIM_CEN;  // Enable timer
}

void set_pmic_status(pmic_result_t pmic_result) {
    i2c_registers[I2C_REG_PMIC_COMM_FAULT] |= (pmic_result & 1);
    if (i2c_registers[I2C_REG_PMIC_COMM_FAULT] & 1) {
        // Latch bit 2 after a communication fault has occured
        // can be reset by writing to the I2C register from the host
        if ((i2c_registers[I2C_REG_PMIC_COMM_FAULT] >> 1) & 1) {
            // First communication fault, generate interrupt
            interrupt_set(false, false, true);
        }
        i2c_registers[I2C_REG_PMIC_COMM_FAULT] |= (1 << 1);
    }
}

// I2C write callback
void i2c_write_cb(uint8_t reg, uint8_t length) {
    static uint32_t new_rtc_value = 0;

    while (length > 0) {
        switch (reg) {
            case I2C_REG_DISPLAY_BACKLIGHT: {
                timer3_set(i2c_registers[I2C_REG_DISPLAY_BACKLIGHT]);
                break;
            }
            case I2C_REG_KEYBOARD_BACKLIGHT:
                timer2_set(i2c_registers[I2C_REG_KEYBOARD_BACKLIGHT]);
                break;
            case I2C_REG_OUTPUT:
                funDigitalWrite(pin_amplifier_enable, i2c_registers[I2C_REG_OUTPUT] & 1);
#if HARDWARE_REV > 1
                funDigitalWrite(pin_camera, i2c_registers[I2C_REG_OUTPUT] & 2);
#endif
                break;
            case I2C_REG_RADIO_CONTROL:
#if OVERRIDE_C6 == false  // Ignore radio control register if radio override is active
                funDigitalWrite(pin_c6_enable, ((i2c_registers[I2C_REG_RADIO_CONTROL] >> 0) & 1) ? FUN_HIGH : FUN_LOW);
                funDigitalWrite(pin_c6_boot, ((i2c_registers[I2C_REG_RADIO_CONTROL] >> 1) & 1) ? FUN_HIGH : FUN_LOW);
#endif
                break;
            case I2C_REG_RTC_VALUE_0:
                new_rtc_value &= 0xFFFFFF00;
                new_rtc_value |= i2c_registers[I2C_REG_RTC_VALUE_0];
                break;
            case I2C_REG_RTC_VALUE_1:
                new_rtc_value &= 0xFFFF00FF;
                new_rtc_value |= i2c_registers[I2C_REG_RTC_VALUE_1] << 8;
                break;
            case I2C_REG_RTC_VALUE_2:
                new_rtc_value &= 0xFF00FFFF;
                new_rtc_value |= i2c_registers[I2C_REG_RTC_VALUE_2] << 16;
                break;
            case I2C_REG_RTC_VALUE_3:
                new_rtc_value &= 0x00FFFFFF;
                new_rtc_value |= i2c_registers[I2C_REG_RTC_VALUE_3] << 24;
                rtc_set_counter(new_rtc_value);
                break;
            case I2C_REG_PMIC_ADC_CONTROL: {
                pmic_adc_trigger = i2c_registers[I2C_REG_PMIC_ADC_CONTROL] & 1;
                pmic_adc_continuous = (i2c_registers[I2C_REG_PMIC_ADC_CONTROL] & 2) >> 1;
                break;
            }
            case I2C_REG_PMIC_CHARGING_CONTROL: {
                pmic_force_disable_charging = (i2c_registers[I2C_REG_PMIC_CHARGING_CONTROL] & (1 << 0)) & 1;
                uint16_t target_current = 512;
                uint8_t add_current = (i2c_registers[I2C_REG_PMIC_CHARGING_CONTROL] >> 1) & 3;
                if (add_current & (1 << 0)) {
                    target_current += 512;
                }
                if (add_current & (1 << 1)) {
                    target_current += 1024;
                }
                pmic_target_charging_current = target_current;
                pmic_force_detect_battery = (i2c_registers[I2C_REG_PMIC_CHARGING_CONTROL] & (1 << 3)) & 1;
                break;
            }
            case I2C_REG_PMIC_OTG_CONTROL:
                set_pmic_status(pmic_set_otg_enable(i2c_registers[I2C_REG_PMIC_OTG_CONTROL] & 1));
                break;
            default:
                if (reg >= I2C_REG_BACKUP_0 && reg <= I2C_REG_BACKUP_83) {
                    bkp_write_byte(reg - I2C_REG_BACKUP_0, i2c_registers[reg]);
                }
                break;
        }
        // Next register
        reg++;
        length--;
    }
}

// I2C read callback
void i2c_read_cb(uint8_t reg) {
    switch (reg) {
        case I2C_REG_KEYBOARD_0:
        case I2C_REG_KEYBOARD_1:
        case I2C_REG_KEYBOARD_2:
        case I2C_REG_KEYBOARD_3:
        case I2C_REG_KEYBOARD_4:
        case I2C_REG_KEYBOARD_5:
        case I2C_REG_KEYBOARD_6:
        case I2C_REG_KEYBOARD_7:
        case I2C_REG_KEYBOARD_8:
            interrupt_clear(true, false, false);  // Clear keyboard interrupt flag
            break;
        case I2C_REG_INPUT:
            interrupt_clear(false, true, false);  // Clear input interrupt flag
            break;
        case I2C_REG_INTERRUPT:
            interrupt_clear(true, true, true);  // Clear all interrupts flag
            break;
        case I2C_REG_PMIC_COMM_FAULT:
        case I2C_REG_PMIC_FAULT:
        case I2C_REG_PMIC_ADC_CONTROL:
        case I2C_REG_PMIC_ADC_VBAT_0:
        case I2C_REG_PMIC_ADC_VBAT_1:
        case I2C_REG_PMIC_ADC_VSYS_0:
        case I2C_REG_PMIC_ADC_VSYS_1:
        case I2C_REG_PMIC_ADC_TS_0:
        case I2C_REG_PMIC_ADC_TS_1:
        case I2C_REG_PMIC_ADC_VBUS_0:
        case I2C_REG_PMIC_ADC_VBUS_1:
        case I2C_REG_PMIC_ADC_ICHGR_0:
        case I2C_REG_PMIC_ADC_ICHGR_1:
            interrupt_clear(false, false, true);  // Clear PMIC interrupt flag
            break;
        default:
            break;
    }
}

void bkp_read_all(void) {
    for (uint8_t index = 0; index < 42; index++) {
        uint16_t value = bkp_read(index);
        i2c_registers[I2C_REG_BACKUP_0 + index * 2 + 0] = (value >> 0) & 0xFF;
        i2c_registers[I2C_REG_BACKUP_0 + index * 2 + 1] = (value >> 8) & 0xFF;
    }
}

void configure_usb_input(void) {
    pmic_set_input_current_limit(3000, false, false);  // Allow up to 2000mA to be sourced from the USB-C port
    // pmic_set_input_current_optimizer(true);           // Reduce current if supply insufficient for 2000mA
    pmic_set_input_current_optimizer(false);  // Take 2000mA regardless of the charger (workaround)
}

void pmic_task(void) {
    // Periodic task for controlling PMIC
    static uint8_t empty_battery_delay = 4;
    static bool prev_adc_contiuous = false;
    static bool adc_active = false;
    static bool prev_vbus_attached = false;
    static bool vbus_attached = false;
    static bool battery_attached = false;
    static bool prev_force_disable_charging = false;
    static uint16_t prev_pmic_target_charging_current = 0;

    pmic_result_t res;

    // Fault reporting
    uint8_t raw_faults = 0;
    pmic_faults_t faults = {0};
    res = pmic_get_faults(&raw_faults, &faults);
    set_pmic_status(res);
    if (res != PMIC_OK) return;  // Stop on communication error
    uint8_t prev_raw_faults = i2c_registers[I2C_REG_PMIC_FAULT];
    i2c_registers[I2C_REG_PMIC_FAULT] = raw_faults;
    if (prev_raw_faults != raw_faults) {
        interrupt_set(false, false, true);
    }

    // ADC: process previous conversion
    if (adc_active || pmic_adc_continuous) {
        uint16_t adc_vbat = 0;
        res = pmic_get_adc_vbat(&adc_vbat, NULL);
        if (res != PMIC_OK) {
            set_pmic_status(res);
            return;  // Stop on communication error
        }

        uint16_t adc_vsys = 0;
        res = pmic_get_adc_vsys(&adc_vsys);
        if (res != PMIC_OK) {
            set_pmic_status(res);
            return;  // Stop on communication error
        }

        uint16_t adc_tspct = 0;
        res = pmic_get_adc_tspct(&adc_tspct);
        if (res != PMIC_OK) {
            set_pmic_status(res);
            return;  // Stop on communication error
        }

        // uint16_t adc_vbus = 0;
        // res = pmic_get_adc_vbus(&adc_vbus, NULL);
        // if (res != PMIC_OK) {
        //    set_pmic_status(res);
        //    return;  // Stop on communication error
        //}

        uint16_t adc_ichgr = 0;
        res = pmic_get_adc_ichgr(&adc_ichgr);
        if (res != PMIC_OK) {
            set_pmic_status(res);
            return;  // Stop on communication error
        }

        LockI2CSlave(true);
        i2c_registers[I2C_REG_PMIC_ADC_VBAT_0] = adc_vbat & 0xFF;
        i2c_registers[I2C_REG_PMIC_ADC_VBAT_1] = (adc_vbat >> 8) & 0xFF;
        i2c_registers[I2C_REG_PMIC_ADC_VSYS_0] = adc_vsys & 0xFF;
        i2c_registers[I2C_REG_PMIC_ADC_VSYS_1] = (adc_vsys >> 8) & 0xFF;
        i2c_registers[I2C_REG_PMIC_ADC_TS_0] = adc_tspct & 0xFF;
        i2c_registers[I2C_REG_PMIC_ADC_TS_1] = (adc_tspct >> 8) & 0xFF;
        // i2c_registers[I2C_REG_PMIC_ADC_VBUS_0] = adc_vbus & 0xFF;
        // i2c_registers[I2C_REG_PMIC_ADC_VBUS_1] = (adc_vbus >> 8) & 0xFF;
        i2c_registers[I2C_REG_PMIC_ADC_ICHGR_0] = adc_ichgr & 0xFF;
        i2c_registers[I2C_REG_PMIC_ADC_ICHGR_1] = (adc_ichgr >> 8) & 0xFF;
        LockI2CSlave(false);

        if (adc_active) {
            interrupt_set(false, false, true);
        }

        adc_active = false;
    }

    // ADC: trigger new conversion
    if (pmic_adc_trigger || prev_adc_contiuous != pmic_adc_continuous) {
        res = pmic_set_adc_configuration(pmic_adc_trigger, prev_adc_contiuous);
        if (res != PMIC_OK) {
            set_pmic_status(res);
            return;  // Stop on communication error
        }

        pmic_adc_trigger = 0;
        prev_adc_contiuous = pmic_adc_continuous;

        adc_active = true;
        i2c_registers[I2C_REG_PMIC_ADC_CONTROL] &= ~(1 << 0);  // Clear the trigger bit
    }

    // Battery detection
    if (empty_battery_delay > 0) {
        empty_battery_delay -= 1;
        if (empty_battery_delay == 0) {
            prev_vbus_attached = false;  // Force redetect
        }
    }

    uint16_t adc_vbus = 0;
    pmic_get_adc_vbus(&adc_vbus, &vbus_attached);
    LockI2CSlave(true);
    i2c_registers[I2C_REG_PMIC_ADC_VBUS_0] = adc_vbus & 0xFF;
    i2c_registers[I2C_REG_PMIC_ADC_VBUS_1] = (adc_vbus >> 8) & 0xFF;
    LockI2CSlave(false);
    if (pmic_force_disable_charging) {
        // Charging has been disabled by user
        if (!prev_force_disable_charging) {
            res = pmic_configure_battery_charger(false, 0);
        }
        prev_force_disable_charging = pmic_force_disable_charging;
    } else {
        if (vbus_attached) {
            uint16_t readback_current = 0;
            res = pmic_get_fast_charge_current(&readback_current);
            if (res != PMIC_OK) {
                set_pmic_status(res);
                return;  // Stop on communication error
            }
            if (readback_current != pmic_target_charging_current) {
                // Increase current if requested (workaround)
                // printf("Reconfigure current %lu %lu\r\n", readback_current, pmic_target_charging_current);
                configure_usb_input();
                pmic_configure_battery_charger(battery_attached || pmic_force_detect_battery,
                                               pmic_target_charging_current);
            }
        }

        if ((!prev_vbus_attached && vbus_attached) ||
            (vbus_attached && (prev_pmic_target_charging_current != pmic_target_charging_current))) {
            //   Badge has been connected to USB supply
            // printf("CHANGE! %lu\r\n", pmic_target_charging_current);
            configure_usb_input();
            pmic_battery_attached(&battery_attached, empty_battery_delay == 0);
            pmic_configure_battery_charger(battery_attached || pmic_force_detect_battery,
                                           512);  // Always start charging at 512mA (workaround)
        } else if (prev_vbus_attached && !vbus_attached) {
            // Badge has been disconnected from USB supply
            pmic_configure_battery_charger(false, 0);  // Disable battery charging
        }
    }
    prev_vbus_attached = vbus_attached;
    prev_pmic_target_charging_current = pmic_target_charging_current;

    uint8_t charging_status = 0;
    if (battery_attached) {
        charging_status |= (1 << 0);  // Bit 0: battery attached
    }
    if (vbus_attached) {
        charging_status |= (1 << 1);  // Bit 1: power input attached
    }
    if (pmic_force_disable_charging) {
        charging_status |= (1 << 2);  // Bit 2: charging disabled by user
    }

    pmic_charge_status_t charge_status = PMIC_CHARGE_STATUS_NOT_CHARGING;
    res = pmic_get_charge_status(&charge_status);
    if (res != PMIC_OK) {
        set_pmic_status(res);
        return;  // Stop on communication error
    }

    charging_status |=
        (((uint8_t)(charge_status) & 3) << 3);  // Charge status is two bits, put at bits 3 and 4 of the register

    i2c_registers[I2C_REG_PMIC_CHARGING_STATUS] = charging_status;
}

// Entry point
int main() {
    SystemInit();
    funGpioInitAll();

    // Initialize keyboard
    keyboard_init();

    // Initialize I2C slave
    funPinMode(pin_sda, GPIO_CFGLR_OUT_10Mhz_AF_OD);  // SDA
    funPinMode(pin_scl, GPIO_CFGLR_OUT_10Mhz_AF_OD);  // SCL
    SetupI2CSlave(0x5f, i2c_registers, sizeof(i2c_registers), i2c_write_cb, i2c_read_cb, false);

    // Initialize I2C master
#if HARDWARE_REV > 1
    funPinMode(pin_pm_sda, GPIO_CFGLR_OUT_10Mhz_AF_OD);  // SDA
    funPinMode(pin_pm_scl, GPIO_CFGLR_OUT_10Mhz_AF_OD);  // SCL
    SetupI2CMaster();

    // Disable PMIC I2C watchdog
    pmic_set_watchdog_timer_limit(0);

    // Connect battery if previously disabled
    pmic_set_battery_disconnect_enable(false);

    // Configure USB power input
    configure_usb_input();

    // Configure other stuff
    pmic_set_battery_load_enable(false);          // Disable 30mA load on battery
    pmic_set_minimum_system_voltage_limit(3500);  // 3.5v (default)
    pmic_set_adc_configuration(false, false);     // Disable continuous ADC mode

    // Configure battery charger
    pmic_set_charge_enable(false);              // Disable battery charging
    pmic_set_pumpx_enable(false);               // Disable current pulse control
    pmic_configure_battery_charger(true, 512);  // Battery attached and charge at 512mA

    pmic_set_otg_enable(true);  // Enable OTG booster (for testing)
#endif

    // ESP32-C6
    funPinMode(pin_c6_enable, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP);
    funDigitalWrite(pin_c6_enable, FUN_LOW);
    funPinMode(pin_c6_boot, GPIO_Speed_10MHz | GPIO_CNF_OUT_OD);
    funDigitalWrite(pin_c6_boot, FUN_HIGH);

#if OVERRIDE_C6
    funDigitalWrite(pin_c6_enable, FUN_HIGH);  // Turn on radio
#endif

    // Display backlight
    timer3_init();  // Use timer 3 channel 1 as PWM output for controlling display backlight

    // Keyboard backlight
    timer2_init();  // Use timer 2 channel 2 as PWM output for controlling keyboard backlight

    // Interrupt
    funPinMode(pin_interrupt, GPIO_Speed_10MHz | GPIO_CNF_OUT_OD);
    funDigitalWrite(pin_interrupt, FUN_HIGH);

    // SD card detect
    funPinMode(pin_sdcard_detect, GPIO_Speed_In | GPIO_CNF_IN_FLOATING);

    // Headphone detect
    funPinMode(pin_headphone_detect, GPIO_Speed_In | GPIO_CNF_IN_FLOATING);

    // Amplifier enable
    AFIO->PCFR1 |= AFIO_PCFR1_PD01_REMAP;

    funPinMode(pin_amplifier_enable, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP);
    funDigitalWrite(pin_amplifier_enable, FUN_LOW);

    // Real time clock
    rtc_init();

    // Backup registers
    bkp_read_all();

#if HARDWARE_REV > 1
    bool power_button_latch = false;
    uint8_t power_button_counter = 0;
#endif

    while (1) {
        i2c_registers[I2C_REG_FW_VERSION_0] = (FW_VERSION) & 0xFF;
        i2c_registers[I2C_REG_FW_VERSION_1] = (FW_VERSION >> 8) & 0xFF;

        uint32_t now = SysTick->CNT;

        static uint32_t keyboard_scan_previous = 0;
        if (now - keyboard_scan_previous >= keyboard_scan_interval * DELAY_MS_TIME) {
            keyboard_scan_previous = now;
            bool set_keyboard_interrupt =
                keyboard_step(&i2c_registers[I2C_REG_KEYBOARD_0]);  // Scans one row when called
            if (set_keyboard_interrupt) {
                interrupt_set(true, false, false);
            }
        }

        static uint32_t input_scan_previous = 0;
        if (now - input_scan_previous >= input_scan_interval * DELAY_MS_TIME) {
            input_scan_previous = now;
            bool set_input_interrupt = input_step();  // Scans all inputs
            if (set_input_interrupt) {
                interrupt_set(false, true, false);
            }

#if HARDWARE_REV > 1
            if (!funDigitalRead(pin_power_in)) {
                if (power_button_latch && power_button_counter > 500 / input_scan_interval) {
                    pmic_power_off();
                }
                power_button_counter++;
            } else {
                power_button_latch = true;
                power_button_counter = 0;
            }
#endif
        }

        static uint32_t rtc_previous = 0;
        if (now - rtc_previous >= 1 * DELAY_MS_TIME) {
            rtc_previous = now;
            uint32_t value = rtc_get_counter();
            LockI2CSlave(true);
            i2c_registers[I2C_REG_RTC_VALUE_0] = (value >> 0) & 0xFF;
            i2c_registers[I2C_REG_RTC_VALUE_1] = (value >> 8) & 0xFF;
            i2c_registers[I2C_REG_RTC_VALUE_2] = (value >> 16) & 0xFF;
            i2c_registers[I2C_REG_RTC_VALUE_3] = (value >> 24) & 0xFF;
            LockI2CSlave(false);
        }

        /*static uint32_t bl_previous = 0;
        if (now - bl_previous >= 20 * DELAY_MS_TIME) {
            bl_previous = now;
            if (backlight_fade_value < timer3_pwm_cycle_width) {
                timer3_set(backlight_fade_value);
                backlight_fade_value++;
            }
        }*/

#if HARDWARE_REV > 1
        static uint32_t pmic_previous = 0;
        if (now - pmic_previous >= 500 * DELAY_MS_TIME) {
            pmic_previous = now;
            pmic_task();
        }
#endif

        funDigitalWrite(pin_interrupt,
                        (keyboard_interrupt | input_interrupt | pmic_interrupt)
                            ? FUN_LOW
                            : FUN_HIGH);  // Update interrupt pin state
    }
}
