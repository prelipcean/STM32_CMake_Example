# STM32_CMake_Example

A bare-metal "catapult" platform for STM32 microcontrollers (currently the
STM32F429ZI), built with CMake. It gives a new project a layered structure,
register-level drivers, a cross-platform build and host unit tests, so you
don't start from scratch.

## Features

- **Layered architecture**: Application → VFB → Middleware/BSP → MCAL, each module
  a separate CMake target that only sees the headers it depends on
- **Clock configuration**: HSE + PLL, 168MHz (180MHz with Over-Drive, `-DSYSCLOCK_OVERDRIVE=ON`)
- **Drivers**: FPU, NVIC, SYSCFG, GPIO (incl. EXTI), USART (polling + RX interrupt)
- **Cross-platform build**: Windows, Linux and macOS with the same commands (CMake presets)
- **Unit tests** on the host with [Unity](https://github.com/ThrowTheSwitch/Unity)
- **CI**: GitHub Actions builds the firmware and runs the tests on all three OSes

## Hardware

- STM32F429I-DISC1 board (LEDs on PG13/PG14, user button on PA0, 8MHz HSE crystal)
- USART1 on PA9 (TX) / PA10 (RX), 9600 8N1 — RX bytes are echoed back. On the
  STM32F429I-DISC1 these pins are the ST-LINK/V2-B virtual COM port, so the echo
  works over the same USB cable (e.g. `/dev/cu.usbmodem*`, `/dev/ttyACM*`, `COMx`)
- ST-LINK V2/V3 for flashing and debugging

## Software Requirements

| Tool | Version | Notes |
|---|---|---|
| [Arm GNU Toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads) (`arm-none-eabi`) | 13.x / 14.x | found automatically, see below |
| CMake | ≥ 3.21 | |
| Ninja | any | |
| Host C compiler | GCC, Clang or MSVC | only for unit tests |
| OpenOCD | ≥ 0.11 | optional, flashing/debugging |

The toolchain is located in this order:

1. `-DARM_TOOLCHAIN_DIR=<path>` or the `ARM_TOOLCHAIN_DIR` environment variable
2. `arm-none-eabi-gcc` on `PATH`
3. Default install folders (newest first):
   - Windows: `C:/Program Files (x86)/Arm GNU Toolchain arm-none-eabi/*/bin`, STM32CubeIDE
   - macOS: `/Applications/ArmGNUToolchain/*/arm-none-eabi/bin`, Homebrew
   - Linux: `/opt/gcc-arm-none-eabi*/bin`, `/opt/arm-gnu-toolchain-*/bin`, `/usr/bin`

## Building

The same commands work on every OS:

```bash
# Firmware
cmake --preset debug              # or: release
cmake --build --preset debug

# Unit tests (host)
cmake --preset host-tests
cmake --build --preset host-tests
ctest --preset host-tests
```

### Build options

| CMake option | Default | Effect |
|---|---|---|
| `SYSCLOCK_OVERDRIVE` | `OFF` | `ON`: SYSCLK 180MHz (Voltage Scale 1 + Over-Drive), APB1/APB2 45/90MHz. The PLL48 clock (USB/SDIO) becomes 45MHz. `OFF`: 168MHz, APB1/APB2 42/84MHz, PLL48 = 48MHz. |
| `PLATFORM_BOARD` | `STM32F429ZI` | Board file in `cmake/boards/` |

Use the `debug-180mhz` preset, or add the option to any preset:

```bash
cmake --preset debug-180mhz && cmake --build --preset debug-180mhz
cmake --preset release -DSYSCLOCK_OVERDRIVE=ON
```

Outputs are in `build/<preset>/`:

- `STM32_F429ZI_Template.elf` - ELF file for debugging
- `STM32_F429ZI_Template.hex` / `.bin` - images for flashing
- `STM32_F429ZI_Template.map` - linker map

On Windows, run the unit tests from a "Developer PowerShell for VS" (MSVC) or
with MinGW/MSYS2 `gcc` on `PATH`.

## Programming

```bash
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
        -c "program build/debug/STM32_F429ZI_Template.elf verify reset exit"
```

## VS Code Tasks

`Ctrl/Cmd+Shift+P` → **Tasks: Run Task** (`Ctrl/Cmd+Shift+B` runs the default build). Tasks marked
*pick preset* ask for `debug`, `release`, `debug-180mhz` or `release-180mhz`.

| Group | Tasks |
|---|---|
| Build | Build (Debug) *(default)*, Build (Debug 180MHz), Build / Configure / Rebuild / Clean *(pick preset)*, Build All Presets, Clean All |
| Test | Unit Tests (Host) |
| Flash | Build + Flash (Debug), Build + Flash *(pick preset)*, Flash Only *(pick preset, no build)*, Reset Target, Mass Erase |
| Board | List ST-LINK probes, Identify target, Show clocks, Read variable *(by name, using the ELF of the flashed build)*, Backup flash (to `backups/`, git-ignored), UART echo test, UART baud scan |

Flash/board tasks need `openocd` on PATH; board tasks also need Python 3 (`tools/stm32_probe.py`,
standard library only; on Windows the UART tasks need `python -m pip install pyserial`). On Windows
the tasks run under `cmd.exe`, so `&&` chains work regardless of the default terminal.

## Debugging (VS Code)

Open the folder and install the recommended extensions (`.vscode/extensions.json`).
CMake Tools picks up the presets and provides IntelliSense. **F5** builds, flashes and starts
Cortex-Debug with OpenOCD. Launch configurations: *STM32 Debug (168 MHz)*, *STM32 Debug (180 MHz
Over-Drive)*, and *STM32 Attach (no flash, no reset)* for a target that is already running. Tools are taken from `PATH`; machine-specific paths
(`cortex-debug.armToolchainPath`, `cortex-debug.openocdPath`,
`clang-format.executable`) belong in your *user* settings. For the peripheral view
set `STM32_SVD_DIR` to the folder containing `STM32F429.svd`.

## Project Structure

```
├── CMakeLists.txt              # Top level: options, flags, firmware executable
├── CMakePresets.json           # debug / release / host-tests
├── cmake/
│   ├── arm-none-eabi-toolchain.cmake
│   ├── platform.cmake          # platform_add_module() helper
│   ├── boards/STM32F429ZI.cmake  # CPU flags, defines, startup, linker script
│   └── unit_tests.cmake        # Unity + add_unit_test()
├── Project/
│   ├── CMakeLists.txt          # Module list, layer by layer
│   ├── 01_Application/         # main.c, software components (SWC_*)
│   ├── 02_VFB/                 # Virtual function bus (SWC <-> BSP mapping)
│   ├── 03_Middleware/          # BSP (board, startup, linker), CDD, OS
│   ├── 04_MCAL/                # CMSIS + register-level drivers
│   ├── XX_Libraries/           # Third party libraries
│   └── YY_Utilities/           # std_types, platform_types, reg_util
├── Tests/UnitTest/             # Host unit tests and mocks
├── Templates/                  # .c/.h file templates
└── Documentation/
```

### Startup sequence

`Reset_Handler` → `SystemInit()` (FPU only; runs before `.data`/`.bss` are
initialized, so it must not write globals) → `.data`/`.bss` init → `main()` →
`Board_Init()` (clocks, LEDs, button) → `Controller_Init()` → main loop.

## Extending the Platform

**New module** - add the sources and register it in `Project/CMakeLists.txt`:

```cmake
platform_add_module(
  mcal_spi
  SOURCES 04_MCAL/SPI/Source/spi.c
  INCLUDES 04_MCAL/SPI/Include
  DEPENDS utilities cmsis          # needed by spi.h
  PRIVATE_DEPENDS mcal_gpio)       # needed only by spi.c
```

It is linked into the firmware automatically. Use the files in `Templates/` for
new sources/headers.

**New board** - add `cmake/boards/<BOARD>.cmake` (see `STM32F429ZI.cmake`) and
configure with `-DPLATFORM_BOARD=<BOARD>`.

**New unit test** - add `Tests/UnitTest/test_<name>.c` (see `test_std_types.c`) and
register it in `Tests/UnitTest/CMakeLists.txt`:

```cmake
add_unit_test(test_<name> ${CMAKE_CURRENT_SOURCE_DIR}/test_<name>.c
              <production sources under test>...)
```

## Code Style

Formatting is defined in `.clang-format` (see
`Documentation/Clang-Format_Configuration_Explained.txt`).

## Contributing

Feel free to submit issues and pull requests.

## License

This project is released under the MIT License. See the LICENSE file for details.
