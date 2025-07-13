#pragma once

// I2C to host
#define pin_sda       PB7
#define pin_scl       PB6
#define pin_interrupt PA0

// Radio control
#define pin_c6_enable PB8
#define pin_c6_boot   PD1

// Backlight
#define pin_display_backlight  PB4
#define pin_keyboard_backlight PB3

// Audio
#define pin_headphone_detect PB5
#define pin_amplifier_enable PD0

// SD card
#define pin_sdcard_detect PA15

// Power
#define pin_power_in  PA12  // Input from power button
#define pin_power_out PC13  // Output to power button latch

// LEDs
#define pin_led_data PA11

// PMIC
#define pin_pm_sda PB11
#define pin_pm_scl PB10
