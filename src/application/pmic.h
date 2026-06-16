// Tanmatsu coprocessor firmware
// SPDX-FileCopyrightText: 2024 Nicolai Electronics
// SPDX-License-Identifier: MIT

#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    PMIC_OK = 0,
    PMIC_ERROR = 1,
} pmic_result_t;

typedef struct {
    bool watchdog;
    bool boost;
    bool chrg_input;
    bool chrg_thermal;
    bool chrg_safety;
    bool batt_ovp;
    bool ntc_cold;
    bool ntc_hot;
    bool ntc_boost;
} pmic_faults_t;

typedef enum {
    PMIC_BHOT_1 = 0,
    PMIC_BHOT_0 = 1,
    PMIC_BHOT_2 = 2,
    PMIC_BHOT_DISABLE = 3,
} pmic_bhot_t;

typedef enum {
    PMIC_CHARGE_STATUS_NOT_CHARGING = 0,
    PMIC_CHARGE_STATUS_PRE_CHARGING = 1,
    PMIC_CHARGE_STATUS_FAST_CHARGING = 2,
    PMIC_CHARGE_STATUS_CHARGE_TERMINATION_DONE = 3,
} pmic_charge_status_t;

typedef enum {
    PMIC_VBUS_STATUS_NO_INPUT = 0,
    PMIC_VBUS_STATUS_USB_CDP = 1,
    PMIC_VBUS_STATUS_USB_DCP = 2,
    PMIC_VBUS_STATUS_MAXCHARGE = 3,
    PMIC_VBUS_STATUS_UNKNOWN_ADAPTER = 4,
    PMIC_VBUS_STATUS_NON_STANDARD_ADAPTER = 5,
    PMIC_VBUS_STATUS_OTG = 6,
} pmic_vbus_status_t;

// REG00
pmic_result_t pmic_set_input_current_limit(uint16_t current, bool enable_ilim_pin, bool enable_hiz);
pmic_result_t pmic_get_input_current_limit(uint16_t* out_current, bool* out_enable_ilim_pin, bool* out_enable_hiz);

// REG01
pmic_result_t pmic_set_input_voltage_limit_offset(uint8_t offset);
pmic_result_t pmic_get_input_voltage_limit_offset(uint8_t* out_offset);
pmic_result_t pmic_set_boost_mode_temperature_monitor_thresholds(bool cold, pmic_bhot_t hot);
pmic_result_t pmic_get_boost_mode_temperature_monitor_thresholds(bool* out_cold, pmic_bhot_t* out_hot);

// REG02
pmic_result_t pmic_set_vbus_detection_mode(bool auto_dpdm_en, bool force_dpdm);
pmic_result_t pmic_get_vbus_detection_mode(bool* auto_dpdm_en, bool* force_dpdm);
pmic_result_t pmic_set_otg_boost_frequency(bool low_frequency);
pmic_result_t pmic_get_otg_boost_frequency(bool* out_low_frequency);
pmic_result_t pmic_set_input_current_optimizer(bool enable);
pmic_result_t pmic_get_input_current_optimizer(bool* out_enable);
pmic_result_t pmic_set_high_voltage_dcp(bool enable);
pmic_result_t pmic_get_high_voltage_dcp(bool* out_enable);
pmic_result_t pmic_set_maxcharge(bool enable);
pmic_result_t pmic_get_maxcharge(bool* out_enable);
pmic_result_t pmic_set_adc_configuration(bool start, bool continuous);
pmic_result_t pmic_get_adc_configuration(bool* out_busy, bool* out_continuous);

// REG03
pmic_result_t pmic_set_minimum_system_voltage_limit(uint16_t millivolt);
pmic_result_t pmic_get_minimum_system_voltage_limit(uint16_t* out_millivolt);
pmic_result_t pmic_set_charge_enable(bool enable);
pmic_result_t pmic_get_charge_enable(bool* out_enable);
pmic_result_t pmic_set_otg_enable(bool enable);
pmic_result_t pmic_get_otg_enable(bool* out_enable);
pmic_result_t pmic_reset_watchdog(void);
pmic_result_t pmic_set_battery_load_enable(bool enable);
pmic_result_t pmic_get_battery_load_enable(bool* out_enable);

// REG04
pmic_result_t pmic_set_charge_current_fast(uint16_t current);
pmic_result_t pmic_get_fast_charge_current(uint16_t* out_current);
pmic_result_t pmic_set_pumpx_enable(bool enable);
pmic_result_t pmic_get_pumpx_enable(bool* out_enable);

// REG05
pmic_result_t pmic_set_termination_current(uint16_t current);
pmic_result_t pmic_get_termination_current(uint16_t* out_current);
pmic_result_t pmic_set_precharge_current(uint16_t current);
pmic_result_t pmic_get_precharge_current(uint16_t* out_current);

// REG06
pmic_result_t pmic_set_charge_recharge_threshold_200mv_offset(bool enable);
pmic_result_t pmic_get_charge_recharge_threshold_200mv_offset(bool* out_enable);
pmic_result_t pmic_set_charge_battery_precharge_threshold_3v(bool enable);
pmic_result_t pmic_get_charge_battery_precharge_threshold_3v(bool* out_enable);
pmic_result_t pmic_set_charge_voltage_limit(uint16_t voltage);
pmic_result_t pmic_get_charge_voltage_limit(uint16_t* out_voltage);

void pmic_battery_threshold(uint16_t voltage_limit, bool batlowv, bool vrechg);

// REG07
pmic_result_t pmic_set_charge_timer_limit(uint8_t hours);
pmic_result_t pmic_get_charge_timer_limit(uint8_t* out_hours);
pmic_result_t pmic_set_charge_safety_timer_enable(bool enable);
pmic_result_t pmic_get_charge_safety_timer_enable(bool* out_enable);
pmic_result_t pmic_set_watchdog_timer_limit(uint8_t seconds);
pmic_result_t pmic_get_watchdog_timer_limit(uint8_t* out_seconds);
pmic_result_t pmic_set_status_pin_disable(bool disable);
pmic_result_t pmic_get_status_pin_disable(bool* out_disable);
pmic_result_t pmic_set_charge_termination_enable(bool enable);
pmic_result_t pmic_get_charge_termination_enable(bool* out_enable);

// REG08
pmic_result_t pmic_set_thermal_regulation_threshold(uint8_t temperature);
pmic_result_t pmic_get_thermal_regulation_threshold(uint8_t* out_temperature);
pmic_result_t pmic_set_ir_compensation_voltage_clamp(uint16_t millivolt);
pmic_result_t pmic_get_ir_compensation_voltage_clamp(uint16_t* out_millivolt);
pmic_result_t pmic_set_ir_compensation_resistor_setting(uint16_t milliohm);
pmic_result_t pmic_get_ir_compensation_resistor_setting(uint16_t* out_milliohm);

// REG09
pmic_result_t pmic_set_pumpx_down_enable(bool enable);
pmic_result_t pmic_get_pumpx_down_enable(bool* out_enable);
pmic_result_t pmic_set_pumpx_up_enable(bool enable);
pmic_result_t pmic_get_pumpx_up_enable(bool* out_enable);
pmic_result_t pmic_set_battery_full_system_reset_enable(bool enable);
pmic_result_t pmic_get_battery_full_system_reset_enable(bool* out_enable);
pmic_result_t pmic_set_battery_disconnect_delay_enable(bool enable);
pmic_result_t pmic_get_battery_disconnect_delay_enable(bool* out_enable);
pmic_result_t pmic_set_battery_disconnect_enable(bool enable);
pmic_result_t pmic_get_battery_disconnect_enable(bool* out_enable);
pmic_result_t pmic_set_safety_timer_slow_2x_enable(bool enable);
pmic_result_t pmic_get_safety_timer_slow_2x_enable(bool* out_enable);
pmic_result_t pmic_set_force_ico_enable(bool enable);
pmic_result_t pmic_get_force_ico_enable(bool* out_enable);

// REG0A
pmic_result_t pmic_set_boost_mode_voltage(uint16_t millivolt);
pmic_result_t pmic_get_boost_mode_voltage(uint16_t* out_millivolt);

// REG0B
pmic_result_t pmic_get_vsys_regulation_status(bool* out_active);
pmic_result_t pmic_get_usb_input_sdp_status(bool* out_usb500);
pmic_result_t pmic_get_power_good_status(bool* out_active);
pmic_result_t pmic_get_charge_status(pmic_charge_status_t* out_status);
pmic_result_t pmic_get_vbus_status(pmic_vbus_status_t* out_status);

// REG0C
pmic_result_t pmic_get_faults(uint8_t* out_raw, pmic_faults_t* out_faults);

// REG0D
pmic_result_t pmic_set_vindpm(uint16_t millivolt);
pmic_result_t pmic_get_vindpm(uint16_t* out_millivolt);
pmic_result_t pmic_set_force_vindpm(bool absolute);
pmic_result_t pmic_get_force_vindpm(bool* out_absolute);

// REG0E
pmic_result_t pmic_get_adc_vbat(uint16_t* out_vbat, bool* out_therm_stat);
pmic_result_t pmic_get_thermal_regulation_status(bool* out_therm_stat);

// REG0F
pmic_result_t pmic_get_adc_vsys(uint16_t* out_vsys);

// REG10
pmic_result_t pmic_get_adc_tspct(uint16_t* out_tspct);

// REG11
pmic_result_t pmic_get_adc_vbus(uint16_t* out_vbus, bool* out_vbus_gd);
pmic_result_t pmic_get_vbus_good(bool* out_vbus_gd);

// REG12
pmic_result_t pmic_get_adc_ichgr(uint16_t* out_ichgr);

// REG13
pmic_result_t pmic_get_effective_input_current_limit(uint16_t* out_milliampere);
pmic_result_t pmic_get_effective_input_current_limit_status(bool* out_active);
pmic_result_t pmic_get_voltage_input_limit_status(bool* out_active);

// REG14
pmic_result_t pmic_get_device_revision(uint8_t* out_revision);
pmic_result_t pmic_get_temperature_profile(bool* out_value);
pmic_result_t pmic_get_device_configuration(uint8_t* out_devcfg);
pmic_result_t pmic_get_ico_optimized_status(bool* out_active);
pmic_result_t pmic_register_reset(void);

// Helper functions
pmic_result_t pmic_power_off();
pmic_result_t pmic_battery_attached(bool* battery_attached, bool detect_empty_battery);
pmic_result_t pmic_configure_battery_charger(bool enable, uint16_t current);
