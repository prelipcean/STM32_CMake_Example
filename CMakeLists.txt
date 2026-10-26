# Minimum required version of CMake (3.21 is needed for CMakePresets.json v3)
cmake_minimum_required(VERSION 3.21)

# BUILD_TESTS=ON builds the host unit tests, otherwise the target firmware
option(BUILD_TESTS "Build unit tests for the host instead of the firmware" OFF)

# Default to the ARM toolchain for the firmware build, unless the user
# (or a preset) already provided one
if(NOT BUILD_TESTS AND NOT CMAKE_TOOLCHAIN_FILE)
  set(CMAKE_TOOLCHAIN_FILE
      "${CMAKE_CURRENT_SOURCE_DIR}/cmake/arm-none-eabi-toolchain.cmake"
      CACHE FILEPATH "Toolchain file")
endif()

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE
      Debug
      CACHE STRING "Build type" FORCE)
endif()

if(BUILD_TESTS)
  # ===================================================================
  # Unit Tests Configuration (host build)
  # ===================================================================
  project(
    STM32_F429ZI_Tests
    VERSION 1.0.0
    DESCRIPTION "STM32F429ZI Unit Tests"
    LANGUAGES C)

  enable_testing()
  add_subdirectory(Tests/UnitTest)
  return()
endif()

# Project settings
project(
  STM32_F429ZI_Template
  VERSION 1.0.0
  DESCRIPTION "STM32F429ZI Example Project with CMake"
  LANGUAGES C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON) # gnu11
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON) # gnu++17
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ===================================================================
# Project Executable and File Definitions
# ===================================================================

# Set the name of the final .elf file
set(EXECUTABLE ${CMAKE_PROJECT_NAME})

# MCU-specific files (relative to this CMakeLists.txt)
set(STARTUP_FILE
    ${CMAKE_CURRENT_SOURCE_DIR}/Project/03_Middleware/BSP/STM32F429ZI/startup/startup_stm32f429zitx.s
)
set(LINKER_SCRIPT
    ${CMAKE_CURRENT_SOURCE_DIR}/Project/03_Middleware/BSP/STM32F429ZI/linker/STM32F429ZITX_FLASH.ld
)

# --- Define Your Source and Include Files Here ---
# Add all your .c/.cpp source files here
set(PROJECT_SOURCES
    # 01_Application Layer
    Project/01_Application/SWC_Controller/Source/swc_controller.c

    # 02_VFB Layer

    # 03_Middlewares / Services Layer
    Project/03_Middleware/BSP/STM32F429ZI/board.c
    Project/03_Middleware/BSP/USART/Source/usart3_bsp.c

    # 04_MCAL Layer
    Project/04_MCAL/FPU/Source/fpu.c
    Project/04_MCAL/NVIC/Source/nvic.c
    Project/04_MCAL/SystemClock/Source/sysclock.c
    Project/04_MCAL/SysCfgCtrl/Source/syscfgctrl.c
    Project/04_MCAL/GPIO/Source/gpio.c
    Project/04_MCAL/USART/Source/usart.c)

# Add all your include directories here
set(PROJECT_INCLUDE_DIRS
    # 01_Application Layer
    Project/01_Application/SWC_Controller/Include

    # 02_VFB Layer
    Project/02_VFB/Generated

    # 03_Middlewares / Services Layer
    Project/03_Middleware/BSP/USART/Include

    # 04_MCAL Layer
    Project/04_MCAL/CMSIS/Device/ST/STM32F4xx/Include
    Project/04_MCAL/CMSIS/Include
    Project/04_MCAL/FPU/Include
    Project/04_MCAL/NVIC/Include
    Project/04_MCAL/SystemClock/Include
    Project/04_MCAL/SysCfgCtrl/Include
    Project/04_MCAL/GPIO/Include
    Project/04_MCAL/USART/Include

    # Others
    Project/YY_Utilities/Include)
# --- End of File Definitions ---

# Add the executable target
add_executable(${EXECUTABLE} ${STARTUP_FILE} ${PROJECT_SOURCES})
set_target_properties(${EXECUTABLE} PROPERTIES SUFFIX ".elf")

# Add include directories to the target
target_include_directories(${EXECUTABLE} PRIVATE ${PROJECT_INCLUDE_DIRS})

# ===================================================================
# Compiler, Assembler, and Linker Flags
# ===================================================================

# --- Core CPU Flags (Cortex-M4F) ---
set(CPU_FLAGS -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb)

# --- Preprocessor Definitions ---
target_compile_definitions(
  ${EXECUTABLE}
  PRIVATE STM32 STM32F4 STM32F429xx STM32F429ZITx USE_FPU
          $<$<CONFIG:Debug>:DEBUG> # Define DEBUG only for Debug builds
)

# --- Flags for all languages (optimization/debug level comes from the
#     build type, see the toolchain file) ---
target_compile_options(
  ${EXECUTABLE}
  PRIVATE ${CPU_FLAGS}
          -ffunction-sections # Place each function in its own section
          -fdata-sections # Place each data item in its own section
          -fno-common
          -fmessage-length=0
          -fstack-usage # Generate stack usage information (.su)
          $<$<COMPILE_LANGUAGE:C,CXX>:-Wall
          -Wextra>
          # C++ specific flags
          $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti
          -fno-exceptions
          -fno-threadsafe-statics>
          # ASM specific flags
          $<$<COMPILE_LANGUAGE:ASM>:-x
          assembler-with-cpp
          -Wa,-a,-ad,-alms=${CMAKE_PROJECT_NAME}.lst # Generate list file
          >)

# --- Linker Flags ---
target_link_options(
  ${EXECUTABLE}
  PRIVATE
  ${CPU_FLAGS}
  -T${LINKER_SCRIPT} # Linker script
  -nostartfiles # Startup code is provided by the startup .s file
  --specs=nano.specs # newlib-nano (small memcpy/memset/printf)
  --specs=nosys.specs # Stub system calls (no OS)
  -Wl,-Map=${EXECUTABLE}.map # Generate map file
  -Wl,--gc-sections # Discard unused sections
  -Wl,--print-memory-usage # Print memory usage summary
)
set_target_properties(${EXECUTABLE} PROPERTIES LINK_DEPENDS ${LINKER_SCRIPT})

# ===================================================================
# Post-Build Commands (Hex, Bin, Size)
# ===================================================================

add_custom_command(
  TARGET ${EXECUTABLE}
  POST_BUILD
  COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${EXECUTABLE}>
  COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${EXECUTABLE}> ${EXECUTABLE}.hex
  COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${EXECUTABLE}> ${EXECUTABLE}.bin
  COMMENT "Showing size, generating ${EXECUTABLE}.hex and ${EXECUTABLE}.bin")

# ===================================================================
# End of CMakeLists.txt
# ===================================================================
