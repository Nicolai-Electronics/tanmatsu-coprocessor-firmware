// Tanmatsu coprocessor firmware
// SPDX-FileCopyrightText: 2024 Nicolai Electronics
// SPDX-License-Identifier: MIT

#include "rtc.h"
#include <stdbool.h>
#include <stdint.h>
#include "ch32v003fun.h"

#define PWR_CTLR_R2KSTY   ((uint32_t)0x00010000) /* 2K/20K enable flag (standby) */
#define PWR_CTLR_R30KSTY  ((uint32_t)0x00020000) /* 30K RAM enable flag (standby) */
#define PWR_CTLR_R2KVBAT  ((uint32_t)0x00040000) /* 2K/20K RAM enable flag (vbat) */
#define PWR_CTLR_R30KVBAT ((uint32_t)0x00080000) /* 30K RAM enable flag (vbat) */
#define PWR_CTLR_RAMLV    ((uint32_t)0x00100000) /* RAM low voltage mode enable */

void rtc_disable_wp(void) {
    PWR->CTLR |= PWR_CTLR_DBP;
}

void rtc_enable_wp(void) {
    PWR->CTLR &= ~PWR_CTLR_DBP;
}

void rtc_enter_config(void) {
    RTC->CTLRL |= RTC_CTLRL_CNF;
}

void rtc_exit_config(void) {
    RTC->CTLRL &= (uint16_t) ~((uint16_t)RTC_CTLRL_CNF);
}

void rtc_wait_for_last_task(void) {
    while ((RTC->CTLRL & RTC_FLAG_RTOFF) == 0) {
    }
}

void rtc_wait_for_sync(void) {
    RTC->CTLRL &= (uint16_t)~RTC_FLAG_RSF;
    while ((RTC->CTLRL & RTC_FLAG_RSF) == 0) {
    }
}

void rtc_set_prescaler(uint32_t value) {
    rtc_enter_config();
    RTC->PSCRH = (value >> 16) & 0xF;
    RTC->PSCRL = value & 0xFFFF;
    rtc_exit_config();
}

uint32_t rtc_get_prescaler(void) {
    return ((RTC->PSCRH & 0xF) << 16) | RTC->PSCRL;
}

void rtc_set_counter(uint32_t value) {
    rtc_disable_wp();
    rtc_enter_config();
    rtc_wait_for_last_task();
    rtc_wait_for_sync();
    RTC->CNTH = value >> 16;
    RTC->CNTL = value & 0xFFFF;
    rtc_wait_for_last_task();
    rtc_exit_config();
    rtc_wait_for_last_task();
    rtc_enable_wp();
}

uint32_t rtc_get_counter(void) {
    uint16_t high1 = RTC->CNTH;
    uint16_t high2 = RTC->CNTH;
    uint16_t low = RTC->CNTL;
    if (high1 != high2) {
        return (((uint32_t)high2 << 16) | RTC->CNTL);
    } else {
        return (((uint32_t)high1 << 16) | low);
    }
}

uint32_t rtc_read_divider(void) {
    uint32_t tmp = 0x00;
    tmp = ((uint32_t)RTC->DIVH & (uint32_t)0x000F) << 16;
    tmp |= RTC->DIVL;
    return tmp;
}

void rtc_init(void) {
    RCC->APB1PCENR |= RCC_APB1Periph_PWR | RCC_APB1Periph_BKP;

    bool rtc_not_ready = false;
    rtc_not_ready |= !(RCC->BDCTLR | RCC_LSEON);       // If LSE oscillator is not enabled
    rtc_not_ready |= !(RCC->BDCTLR & RCC_LSERDY);      // If LSE oscillator is not running
    rtc_not_ready |= !(RCC->BDCTLR | RCC_RTCSEL_0);    // If LSE is not selected as clock source
    rtc_not_ready |= !(RCC->BDCTLR | RCC_RTCEN);       // If RTC is not enabled
    rtc_not_ready |= !(rtc_get_prescaler() == 32768);  // If RTC is not set to tick once per second

    if (rtc_not_ready) {
        rtc_disable_wp();          // Disable backup domain write protection
        RCC->BDCTLR |= RCC_LSEON;  // Enable LSE

        // Wait for LSE oscillator ready
        while (!(RCC->BDCTLR & RCC_LSERDY)) {
        }

        RCC->BDCTLR |= RCC_RTCSEL_0;  // Use LSE oscillator as RTC clock source
        RCC->BDCTLR |= RCC_RTCEN;     // Enable RTC

        rtc_wait_for_last_task();
        rtc_wait_for_last_task();
        rtc_set_prescaler(32768);  // 1 tick per second
        rtc_wait_for_last_task();
        rtc_set_counter(0);
        rtc_wait_for_last_task();
        rtc_enable_wp();  // Enable backup domain write protection
    }
}

// Backup registers
uint16_t bkp_read(uint8_t position) {
    switch (position) {
        case 0:
            return BKP->DATAR1;
        case 1:
            return BKP->DATAR2;
        case 2:
            return BKP->DATAR3;
        case 3:
            return BKP->DATAR4;
        case 4:
            return BKP->DATAR5;
        case 5:
            return BKP->DATAR6;
        case 6:
            return BKP->DATAR7;
        case 7:
            return BKP->DATAR8;
        case 8:
            return BKP->DATAR9;
        case 9:
            return BKP->DATAR10;
        case 10:
            return BKP->DATAR11;
        case 11:
            return BKP->DATAR12;
        case 12:
            return BKP->DATAR13;
        case 13:
            return BKP->DATAR14;
        case 14:
            return BKP->DATAR15;
        case 15:
            return BKP->DATAR16;
        case 16:
            return BKP->DATAR17;
        case 17:
            return BKP->DATAR18;
        case 18:
            return BKP->DATAR19;
        case 19:
            return BKP->DATAR20;
        case 20:
            return BKP->DATAR21;
        case 21:
            return BKP->DATAR22;
        case 22:
            return BKP->DATAR23;
        case 23:
            return BKP->DATAR24;
        case 24:
            return BKP->DATAR25;
        case 25:
            return BKP->DATAR26;
        case 26:
            return BKP->DATAR27;
        case 27:
            return BKP->DATAR28;
        case 28:
            return BKP->DATAR29;
        case 29:
            return BKP->DATAR30;
        case 30:
            return BKP->DATAR31;
        case 31:
            return BKP->DATAR32;
        case 32:
            return BKP->DATAR33;
        case 33:
            return BKP->DATAR34;
        case 34:
            return BKP->DATAR35;
        case 35:
            return BKP->DATAR36;
        case 36:
            return BKP->DATAR37;
        case 37:
            return BKP->DATAR38;
        case 38:
            return BKP->DATAR39;
        case 39:
            return BKP->DATAR40;
        case 40:
            return BKP->DATAR41;
        case 41:
            return BKP->DATAR42;
        default:
            return 0;
    }
}

void bkp_write(uint8_t position, uint16_t value) {
    rtc_disable_wp();
    switch (position) {
        case 0:
            BKP->DATAR1 = value;
        case 1:
            BKP->DATAR2 = value;
        case 2:
            BKP->DATAR3 = value;
        case 3:
            BKP->DATAR4 = value;
        case 4:
            BKP->DATAR5 = value;
        case 5:
            BKP->DATAR6 = value;
        case 6:
            BKP->DATAR7 = value;
        case 7:
            BKP->DATAR8 = value;
        case 8:
            BKP->DATAR9 = value;
        case 9:
            BKP->DATAR10 = value;
        case 10:
            BKP->DATAR11 = value;
        case 11:
            BKP->DATAR12 = value;
        case 12:
            BKP->DATAR13 = value;
        case 13:
            BKP->DATAR14 = value;
        case 14:
            BKP->DATAR15 = value;
        case 15:
            BKP->DATAR16 = value;
        case 16:
            BKP->DATAR17 = value;
        case 17:
            BKP->DATAR18 = value;
        case 18:
            BKP->DATAR19 = value;
        case 19:
            BKP->DATAR20 = value;
        case 20:
            BKP->DATAR21 = value;
        case 21:
            BKP->DATAR22 = value;
        case 22:
            BKP->DATAR23 = value;
        case 23:
            BKP->DATAR24 = value;
        case 24:
            BKP->DATAR25 = value;
        case 25:
            BKP->DATAR26 = value;
        case 26:
            BKP->DATAR27 = value;
        case 27:
            BKP->DATAR28 = value;
        case 28:
            BKP->DATAR29 = value;
        case 29:
            BKP->DATAR30 = value;
        case 30:
            BKP->DATAR31 = value;
        case 31:
            BKP->DATAR32 = value;
        case 32:
            BKP->DATAR33 = value;
        case 33:
            BKP->DATAR34 = value;
        case 34:
            BKP->DATAR35 = value;
        case 35:
            BKP->DATAR36 = value;
        case 36:
            BKP->DATAR37 = value;
        case 37:
            BKP->DATAR38 = value;
        case 38:
            BKP->DATAR39 = value;
        case 39:
            BKP->DATAR40 = value;
        case 40:
            BKP->DATAR41 = value;
        case 41:
            BKP->DATAR42 = value;
        default:
            break;
    }
    rtc_enable_wp();
}

void bkp_write_byte(uint8_t position, uint8_t value) {
    uint16_t register_value = bkp_read(position >> 1);
    if (position & 1) {
        register_value &= 0x00FF;
        register_value |= value << 8;
    } else {
        register_value &= 0xFF00;
        register_value |= value;
    }
    bkp_write(position >> 1, register_value);
}

void rtc_set_alarm(uint32_t wakeup_time) {
    rtc_disable_wp();
    rtc_enter_config();
    rtc_wait_for_last_task();
    rtc_wait_for_sync();
    RTC->ALRMH = wakeup_time >> 16;
    RTC->ALRML = wakeup_time & 0xFFFF;
    rtc_wait_for_last_task();
    rtc_exit_config();
    rtc_wait_for_last_task();
    rtc_enable_wp();
}

void rtc_get_alarm(uint32_t* out_wakeup_time) {
    *out_wakeup_time = RTC->ALRML + (RTC->ALRMH << 16);
}

void rtc_configure_wakeup_pin(bool enable) {
    rtc_disable_wp();
    rtc_enter_config();
    BKP->TPCTLR &= ~(BKP_TPE);  // Disable tamper function
    if (enable) {
        BKP->OCTLR |= BKP_ASOE;  // Enable alarm output
    } else {
        BKP->OCTLR &= ~(BKP_ASOE);  // Disable alarm output
    }
    rtc_exit_config();
    rtc_enable_wp();
}
