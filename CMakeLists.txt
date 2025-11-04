# Minimum required version of CMake
cmake_minimum_required(VERSION 3.14.0)

# Check if we're building for tests
option(BUILD_TESTS "Build unit tests" OFF)

if(BUILD_TESTS)
  # Project settings for host build
  project(
    STM32_F429ZI_Tests
    VERSION 1.0.0
    DESCRIPTION "STM32F429ZI Unit Tests"
    LANGUAGES C CXX)

  # ===================================================================
  # Unit Tests Configuration
  # ===================================================================
  if(BUILD_TESTS)
    add_subdirectory(Tests/UnitTests)
  endif()

else()

  # Use ARM toolchain for target build
  set(CMAKE_TOOLCHAIN_FILE
      ${CMAKE_CURRENT_SOURCE_DIR}/cmake/arm-none-eabi-toolchain.cmake)

  # Project settings
  project(
    STM32_F429ZI_Template
    VERSION 1.0.0
    DESCRIPTION "STM32F429ZI Example Project with CMake"
    LANGUAGES C CXX ASM)

  # Settings for cross-compilation
  set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
  set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
  set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
  set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
  set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

  # ===================================================================
  # Project Executable and File Definitions
  # ===================================================================

  # Set the name of the final .elf file
  set(EXECUTABLE ${CMAKE_PROJECT_NAME})

  # MCU-specific files (relative to this CMakeLists.txt)
  set(STARTUP_FILE
      ${CMAKE_CURRENT_SOURCE_DIR}/Project/Board/STM32F429ZI/startup/startup_stm32f429zitx.s
  )
  set(LINKER_SCRIPT
      ${CMAKE_CURRENT_SOURCE_DIR}/Project/Board/STM32F429ZI/linker/STM32F429ZITX_FLASH.ld
  )

  # --- Define Your Source and Include Files Here ---
  # Add all your .c/.cpp source files here
  set(PROJECT_SOURCES
      Project/MCAL/FPU/Source/fpu.c
      Project/MCAL/NVIC/Source/nvic.c
      Project/MCAL/SystemClock/Source/sysclock.c
      Project/MCAL/SysCfgCtrl/Source/syscfgctrl.c
      Project/MCAL/GPIO/Source/gpio.c
      Project/MCAL/USART/Source/usart.c
      Project/Config/USART/Source/usart3_config.c
      Project/Application/SWC_Controller/Source/controller.c
      Project/Board/STM32F429ZI/board.c)

  # Add all your include directories here
  set(PROJECT_INCLUDE_DIRS
      Project/CMSIS/Device/ST/STM32F4xx/Include
      Project/CMSIS/Include
      Project/Utilities/Include
      Project/MCAL/FPU/Include
      Project/MCAL/NVIC/Include
      Project/MCAL/SystemClock/Include
      Project/MCAL/GPIO/Include
      Project/MCAL/USART/Include
      Project/Config/USART/Include
      Project/Application/SWC_Controller/Include
      Project/VFB/Generated
      Project/MCAL/SysCfgCtrl/Include)
  # --- End of File Definitions ---

  # Add the executable target
  add_executable(${EXECUTABLE} ${STARTUP_FILE} ${PROJECT_SOURCES})

  # Add include directories to the target
  target_include_directories(${EXECUTABLE} PRIVATE ${PROJECT_INCLUDE_DIRS})

  # ===================================================================
  # Compiler, Assembler, and Linker Flags
  # ===================================================================

  # --- Core CPU Flags ---
  set(CPU_FLAGS -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb)

  set(MEMORY_DEFS -DRAM_SIZE=192K -DFLASH_SIZE=2048K -DCCM_RAM_SIZE=64K)

  # --- Common Flags for all compilers ---
  set(COMMON_FLAGS
      -nostdlib # Don't use standard system startup files or libraries
      -nodefaultlibs # Don't use standard system libraries
      -fno-builtin # Don't recognize built-in functions
      -Wall # Enable all warnings
      -ffunction-sections # Place each function in its own section
      -fdata-sections # Place each data item in its own section
      -fstack-usage # Generate stack usage information
  )

  # --- Preprocessor Definitions ---
  target_compile_definitions(
    ${EXECUTABLE}
    PRIVATE STM32 STM32F4 STM32F429xx STM32F429ZITx
            $<$<CONFIG:Debug>:DEBUG> # Define DEBUG only for Debug builds
  )

  target_compile_definitions(${EXECUTABLE} PRIVATE USE_FPU)

  # --- Language Specific Flags ---
  target_compile_options(
    ${EXECUTABLE}
    PRIVATE # C specific flags
            $<$<COMPILE_LANGUAGE:C>:
            -std=gnu11 # Use GNU11 standard
            -O0 # No optimization in debug
            -g3 # Maximum debug information
            >
            # ASM specific flags
            $<$<COMPILE_LANGUAGE:ASM>:
            ${CPU_FLAGS}
            ${COMMON_FLAGS}
            -x
            assembler-with-cpp
            -Wa,-a,-ad,-alms=${CMAKE_PROJECT_NAME}.lst # Generate list file
            $<$<CONFIG:Debug>:-g3> # Debug level: Maximum
            $<$<CONFIG:Release>:-g0> # No debug info for Release
            >)

  # --- C Compiler Flags ---
  target_compile_options(
    ${EXECUTABLE}
    PRIVATE $<$<COMPILE_LANGUAGE:C>:
            ${CPU_FLAGS}
            ${COMMON_FLAGS}
            -std=gnu11 # Language standard
            -ffunction-sections # Place functions in their own sections
            -fdata-sections # Place data in their own sections
            -fstack-usage # Enable stack usage analysis
            $<$<CONFIG:Debug>:
            -O0 # No optimization
            -g3 # Debug level: Maximum
            >
            $<$<CONFIG:Release>:
            -Os # Optimize for size (common for release)
            -g0 # No debug info
            >
            >)

  # --- C++ Compiler Flags (for multipurpose use) ---
  target_compile_options(
    ${EXECUTABLE}
    PRIVATE $<$<COMPILE_LANGUAGE:CXX>:
            ${CPU_FLAGS}
            ${COMMON_FLAGS}
            -std=gnu++17 # C++ standard (adjust as needed)
            -fno-rtti
            -fno-exceptions
            -fno-threadsafe-statics
            -ffunction-sections
            -fdata-sections
            -fstack-usage
            $<$<CONFIG:Debug>:
            -O0
            -g3
            >
            $<$<CONFIG:Release>:
            -Os
            -g0
            >
            >)

  # --- Linker Flags ---
  target_link_options(
    ${EXECUTABLE}
    PRIVATE
    ${CPU_FLAGS}
    ${COMMON_FLAGS}
    -T${LINKER_SCRIPT} # Linker script
    -nostartfiles # Don't use standard system startup files
    -Wl,-Map=${EXECUTABLE}.map # Generate map file
    -Wl,--gc-sections # Discard unused sections
    -Wl,--print-memory-usage # Print memory usage summary
  )

  # ===================================================================
  # Post-Build Commands (Hex, Bin, Size)
  # ===================================================================

  add_custom_command(
    TARGET ${EXECUTABLE}
    POST_BUILD
    # Command 1: Show size information
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${EXECUTABLE}>
    COMMENT "Showing size information")

  add_custom_command(
    TARGET ${EXECUTABLE}
    POST_BUILD
    # Command 2: Convert to Intel Hex
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${EXECUTABLE}>
            ${EXECUTABLE}.hex
    COMMENT "Generating Intel Hex file (${EXECUTABLE}.hex)")

  add_custom_command(
    TARGET ${EXECUTABLE}
    POST_BUILD
    # Command 3: Convert to Binary
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${EXECUTABLE}>
            ${EXECUTABLE}.bin
    COMMENT "Generating binary file (${EXECUTABLE}.bin)")
endif()

# ===================================================================
# End of CMakeLists.txt
# ===================================================================
