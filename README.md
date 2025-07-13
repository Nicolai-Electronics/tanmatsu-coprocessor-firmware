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

This firmware is compatible with all board revisions of Tanmatsu.

## How to build

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

## How to install

After the build has run successfully, a file named 'main.bin' is placed in the project root.

The file `build/application/coprocessor.bin` file needs to be copied into the launcher firmware project (`main/tanmatsu_coprocessor.bin`).

After this, build the launcher firmware again, and flash it to the device.

If the firmware version field has not been updated then the firmware won't automatically be flashed. You can force the flashing process to start by pressing the Fn + purple diamond keys.

## License

This firmware, copyright 2024-2025 Nicolai Electronics, is made available under the terms of the MIT license, see [LICENSE](LICENSE) for the full license text. The platform files come from the [ch32v003fun](https://github.com/cnlohr/ch32v003fun) project by Cnlohr and are also licensed under the terms of the MIT license. Please see the [CH32V003 LICENSE](src/platform/LICENSE) for a list of copyright holders.
