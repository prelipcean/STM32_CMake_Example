# STM32_CMake_Example

A bare-metal example project for STM32F429ZI microcontroller using CMake as the build system. This project demonstrates how to set up a professional-grade embedded project with proper clock configuration, FPU usage, and GPIO control.

## Features

- **Clock Configuration**: System clock setup for 168MHz operation (180MHz with overdrive)
- **FPU Support**: Hardware floating-point unit configuration
- **GPIO Control**: LED blinking example using the onboard LEDs
- **CMake Build System**: Modern, cross-platform build configuration
- **CMSIS Integration**: Uses STM32F4xx CMSIS for hardware access

## Hardware Requirements

- STM32F429ZI Development Board
- ST-LINK V2/V3 programmer (integrated on most dev boards)
- USB cable for power and programming

## Software Requirements

- CMake (version 3.15 or higher)
- ARM GCC Toolchain (arm-none-eabi-gcc)
- Ninja or Make build system
- OpenOCD for programming (optional)

## Project Structure

```
STM32_CMake_Example/
├── Application/
│   ├── Include/        # Application headers
│   └── Source/         # Application source files
├── CMSIS/              # STM32F4xx CMSIS files
├── CMakeLists.txt      # Main CMake configuration
├── startup_stm32f429zitx.s  # Startup assembly
└── STM32F429ZITX_FLASH.ld   # Linker script
```

## Building the Project

1. Create a build directory:
   ```bash
   mkdir build
   cd build
   ```

2. Configure with CMake:
   ```bash
   # For Debug build
   cmake -DCMAKE_BUILD_TYPE=Debug -G "Ninja" ..

   # For Release build
   cmake -DCMAKE_BUILD_TYPE=Release -G "Ninja" ..
   ```

3. Build the project:
   ```bash
   ninja
   # or
   cmake --build .
   ```

The output files will be created in the build directory:
- `STM32_CMake_Example.elf` - ELF file for debugging
- `STM32_CMake_Example.hex` - HEX file for flashing
- `STM32_CMake_Example.bin` - Binary file for flashing

## Programming

Using OpenOCD:
```bash
openocd -f board/stm32f429discovery.cfg -c "program build/STM32_CMake_Example.elf verify reset exit"
```

## Features Demonstrated

1. **Clock Configuration**:
   - HSE as clock source
   - PLL configuration for 168MHz operation
   - APB1 at 42MHz, APB2 at 84MHz
   - Optional 180MHz with overdrive mode

2. **FPU Usage**:
   - Hardware FPU enabled
   - Proper initialization sequence

3. **GPIO Control**:
   - Green LED (PG13) blinking in normal operation
   - Red LED (PG14) flashing on clock error

## Debugging

The project can be debugged using:
- OpenOCD + GDB
- ST-Link + GDB
- Visual Studio Code with Cortex-Debug extension

## VS Code extensions installed

13xforever.language-x86-64-assembly
bbenoist.doxygen
cheshirekow.cmake-format
esbenp.prettier-vscode
github.copilot
github.copilot-chat
jeff-hykin.better-cpp-syntax
marus25.cortex-debug
mcu-debug.debug-tracker-vscode
mcu-debug.memory-view
mcu-debug.peripheral-viewer
mcu-debug.rtos-views
ms-vscode-remote.remote-containers
ms-vscode-remote.remote-ssh
ms-vscode-remote.remote-ssh-edit
ms-vscode-remote.remote-wsl
ms-vscode-remote.vscode-remote-extensionpack
ms-vscode.cmake-tools
ms-vscode.cpptools
ms-vscode.cpptools-extension-pack
ms-vscode.cpptools-themes
ms-vscode.hexeditor
ms-vscode.remote-explorer
ms-vscode.remote-server
sanaajani.taskrunnercode
usernamehw.errorlens
xaver.clang-format

## Clang

https://github.com/motine/cppstylelineup

.\clang-format.exe --dump-config --style=Microsoft > .clang-format


## Unit Testing

The project includes a unit testing framework using Unity. Tests are built using MinGW GCC to run on the host machine.

### Prerequisites

- MSYS2 with UCRT64 environment
- GCC and Make from MSYS2 UCRT64 (`C:/msys64/ucrt64/bin`)

### Building and Running Tests

```powershell
# Verify MinGW Make is available
Test-Path "C:/msys64/ucrt64/bin/mingw32-make.exe"

# Clean and create build directory
Remove-Item -Recurse -Force build_tests
mkdir build_tests
cd build_tests

# Configure CMake with explicit compiler paths
cmake -G "MinGW Makefiles" `
    -DCMAKE_C_COMPILER="C:/msys64/ucrt64/bin/gcc.exe" `
    -DCMAKE_CXX_COMPILER="C:/msys64/ucrt64/bin/g++.exe" `
    -DCMAKE_MAKE_PROGRAM="C:/msys64/ucrt64/bin/mingw32-make.exe" `
    -DBUILD_TESTS=ON ..

# Build the tests
cmake --build .

# Run the tests
cd tests
./test_example.exe
```

### Adding New Tests

1. Create a new test file in the `tests` directory
2. Include Unity and required headers:
   ```c
   #include <stdio.h>      // For Unity output functions
   #include "unity.h"      // Unity test framework
   #include "your_header.h" // Header to test
   ```
3. Implement test cases following Unity's pattern:
   ```c
   void setUp(void) {
       // Setup for each test
   }

   void tearDown(void) {
       // Cleanup after each test
   }

   void test_your_function(void) {
       // Your test code
       TEST_ASSERT_EQUAL(expected, actual);
   }

   int main(void) {
       UNITY_BEGIN();
       RUN_TEST(test_your_function);
       return UNITY_END();
   }
   ```
4. Add the test to `tests/CMakeLists.txt` using the `add_unit_test()` function

## Contributing

Feel free to submit issues and pull requests.

## License

This project is released under the MIT License. See the LICENSE file for details.
