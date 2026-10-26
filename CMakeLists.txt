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
# Board selection
# ===================================================================
# Each board is described by cmake/boards/<PLATFORM_BOARD>.cmake
set(PLATFORM_BOARD
    STM32F429ZI
    CACHE STRING "Target board (file name in cmake/boards)")
include(${PROJECT_SOURCE_DIR}/cmake/boards/${PLATFORM_BOARD}.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/platform.cmake)

# ===================================================================
# Compiler, Assembler, and Linker Flags (applied to every module)
# ===================================================================
add_library(platform_options INTERFACE)

target_compile_definitions(
  platform_options INTERFACE ${BOARD_DEFINITIONS}
                             $<$<CONFIG:Debug>:DEBUG> # Debug builds only
)

# Optimization/debug level comes from the build type (see toolchain file)
target_compile_options(
  platform_options
  INTERFACE ${BOARD_CPU_FLAGS}
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
            assembler-with-cpp>)

target_link_options(
  platform_options
  INTERFACE
  ${BOARD_CPU_FLAGS}
  -T${BOARD_LINKER_SCRIPT} # Linker script
  -nostartfiles # Startup code is provided by the startup .s file
  --specs=nano.specs # newlib-nano (small memcpy/memset/printf)
  --specs=nosys.specs # Stub system calls (no OS)
  -Wl,--gc-sections # Discard unused sections
  -Wl,--print-memory-usage # Print memory usage summary
)

# ===================================================================
# Platform modules (Project/CMakeLists.txt)
# ===================================================================
add_subdirectory(Project)

# ===================================================================
# Firmware executable
# ===================================================================

# Set the name of the final .elf file
set(EXECUTABLE ${CMAKE_PROJECT_NAME})

add_executable(${EXECUTABLE} ${BOARD_STARTUP_FILE})
set_target_properties(${EXECUTABLE} PROPERTIES SUFFIX ".elf" LINK_DEPENDS
                                                             ${BOARD_LINKER_SCRIPT})
target_link_libraries(${EXECUTABLE} PRIVATE platform_options)
target_link_options(${EXECUTABLE} PRIVATE -Wl,-Map=${EXECUTABLE}.map)
platform_link_modules(${EXECUTABLE})

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
