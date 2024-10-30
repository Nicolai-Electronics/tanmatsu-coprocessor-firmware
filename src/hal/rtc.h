// Tanmatsu coprocessor firmware
// SPDX-FileCopyrightText: 2024 Nicolai Electronics
// SPDX-License-Identifier: MIT

#pragma once
#include <stdbool.h>
#include <stdint.h>

void rtc_disable_wp(void);
void rtc_enable_wp(void);
void rtc_enter_config(void);
void rtc_exit_config(void);
void rtc_wait_for_last_task(void);
void rtc_wait_for_sync(void);
void rtc_set_prescaler(uint32_t value);
uint32_t rtc_get_prescaler(void);
void rtc_set_counter(uint32_t value);
uint32_t rtc_get_counter(void);
uint32_t rtc_read_divider(void);
void rtc_init(void);
uint16_t bkp_read(uint8_t position);
void bkp_write(uint8_t position, uint16_t value);
void bkp_write_byte(uint8_t position, uint8_t value);
void rtc_set_alarm(uint32_t wakeup_time);
void rtc_get_alarm(uint32_t* out_wakeup_time);
void rtc_configure_wakeup_pin(bool enable);