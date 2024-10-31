# Tanmatsu coprocessor firmware

Firmware for the CH32V203 microcontroller used as coprocessor on the Tanmatsu.

## Features

- I2C peripheral device presenting at address 0x5F for access from the host device
- Interrupt line for notifying host of important events
- Keyboard matrix scanning
- Real time clock
- Backup registers (data stored in the RTC power domain)
- PMIC (BQ25895) control and monitoring, including battery detection, ADC reading and and soft power control
- PWM backlight brightness control

## Compatibility

This firmware is compatible with both the Tanmatsu and the WHY2025 badge. Please be sure to correctly adjust the HW_REV define in [main.c](main.c) to configure the firmware for the correct board.

Currently supported values:

| Value | Device              |
|-------|---------------------|
| 1     | WHY2025 prototype 1 |
| 2     | WHY2025 prototype 2 |

## Build instructions

### Setup the tool chain

TODO: Write setup toolchain here.

### Building the .bin file

The build configuration is configured to use the compiler prefix 'riscv64-elf-'. 
(for exammple: riscv64-elf-gcc, riscv64-elf-g++)

When your system has a toolchain with this prefix, you can just use:

```bash
make
```

When your compiler has another prefix, for example 'riscv32-unknown-elf-' use the following:

```bash
make CMAKE_FLAGS="-DPREFIX=custom-prefix-" build
```

For example on my system, the compiler prefix is 'riscv32-unknown-elf-':

```bash
make CMAKE_FLAGS="-DPREFIX=riscv32-unknown-elf" build
```

## Deploying to the badge

After the build has run successfully, a file named 'main.bin' is placed in the project root.

The 'main.bin' file needs to be copied into the why2025-firmware-esp32p4 project.

```bash
cp ./main.bin < path to why2025-firmware-esp32p4 >/main/ch32_firmware.bin'
```

After this, build the P4 firmware again, and flash it to the badge.

When the badge reboots after the flash it'll flash the CH32 firmware as part of the boot sequence.

(In case instructions are needed to compile the P4 firmware, the README.md of the P4 firmware might help)

## License

This firmware, copyright 2024 Nicolai Electronics, is made available under the terms of the MIT license, see [LICENSE](LICENSE) for the full license text. The platform files come from the [ch32v003fun](https://github.com/cnlohr/ch32v003fun) project by Cnlohr and are also licensed under the terms of the MIT license. Please see the [CH32V003 LICENSE](src/platform/LICENSE) for a list of copyright holders.
