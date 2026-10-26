# ===================================================================
# Arm GNU Toolchain (arm-none-eabi) - cross-platform toolchain file
# ===================================================================
# The compiler is located in this order:
#   1. -DARM_TOOLCHAIN_DIR=<dir> or the ARM_TOOLCHAIN_DIR environment variable
#      (either the toolchain root or its bin/ folder)
#   2. The PATH
#   3. The default install locations of the Arm GNU Toolchain on
#      Windows, macOS and Linux (newest version first)
#
# This file only selects the compiler and the build-type flags. The CPU /
# MCU specific flags live in the top level CMakeLists.txt.
# ===================================================================

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(TOOLCHAIN_PREFIX arm-none-eabi-)

if(NOT DEFINED ARM_TOOLCHAIN_DIR AND DEFINED ENV{ARM_TOOLCHAIN_DIR})
  set(ARM_TOOLCHAIN_DIR "$ENV{ARM_TOOLCHAIN_DIR}")
endif()

# Forward the hint to the try_compile() projects CMake creates internally
list(APPEND CMAKE_TRY_COMPILE_PLATFORM_VARIABLES ARM_TOOLCHAIN_DIR)

# --- Default install locations (globbed, newest version first) ---
if(CMAKE_HOST_WIN32)
  set(_arm_globs
      "C:/Program Files (x86)/Arm GNU Toolchain arm-none-eabi/*/bin"
      "C:/Program Files/Arm GNU Toolchain arm-none-eabi/*/bin"
      "C:/ST/STM32CubeIDE_*/STM32CubeIDE/plugins/*gnu-tools-for-stm32*/tools/bin")
  set(_host_exe_suffix ".exe")
elseif(CMAKE_HOST_APPLE)
  set(_arm_globs
      "/Applications/ArmGNUToolchain/*/arm-none-eabi/bin"
      "/opt/homebrew/bin"
      "/usr/local/bin")
  set(_host_exe_suffix "")
else()
  set(_arm_globs
      "/opt/gcc-arm-none-eabi/bin"
      "/opt/arm-gnu-toolchain-*/bin"
      "/opt/gcc-arm-none-eabi-*/bin"
      "/usr/bin")
  set(_host_exe_suffix "")
endif()

set(_arm_search_dirs "")
foreach(_glob IN LISTS _arm_globs)
  file(GLOB _found LIST_DIRECTORIES true "${_glob}")
  list(SORT _found ORDER DESCENDING)
  list(APPEND _arm_search_dirs ${_found})
endforeach()

find_program(
  ARM_GCC_EXECUTABLE
  NAMES ${TOOLCHAIN_PREFIX}gcc
  HINTS "${ARM_TOOLCHAIN_DIR}" "${ARM_TOOLCHAIN_DIR}/bin"
  PATHS ${_arm_search_dirs}
  NO_CMAKE_FIND_ROOT_PATH)

if(NOT ARM_GCC_EXECUTABLE)
  message(
    FATAL_ERROR
      "arm-none-eabi-gcc not found.\n"
      "Install the Arm GNU Toolchain and either add its bin/ folder to PATH or "
      "configure with -DARM_TOOLCHAIN_DIR=<path> (or set the ARM_TOOLCHAIN_DIR "
      "environment variable).")
endif()

get_filename_component(ARM_TOOLCHAIN_BIN_DIR "${ARM_GCC_EXECUTABLE}" DIRECTORY)
if(NOT CMAKE_IN_TRY_COMPILE)
  message(STATUS "Using Arm GNU Toolchain from: ${ARM_TOOLCHAIN_BIN_DIR}")
endif()

set(_tc "${ARM_TOOLCHAIN_BIN_DIR}/${TOOLCHAIN_PREFIX}")

set(CMAKE_C_COMPILER "${_tc}gcc${_host_exe_suffix}")
set(CMAKE_CXX_COMPILER "${_tc}g++${_host_exe_suffix}")
set(CMAKE_ASM_COMPILER "${_tc}gcc${_host_exe_suffix}")
set(CMAKE_AR "${_tc}ar${_host_exe_suffix}")
set(CMAKE_RANLIB "${_tc}ranlib${_host_exe_suffix}")
set(CMAKE_OBJCOPY "${_tc}objcopy${_host_exe_suffix}" CACHE FILEPATH "objcopy")
set(CMAKE_OBJDUMP "${_tc}objdump${_host_exe_suffix}" CACHE FILEPATH "objdump")
set(CMAKE_SIZE "${_tc}size${_host_exe_suffix}" CACHE FILEPATH "size")
set(CMAKE_NM "${_tc}nm${_host_exe_suffix}" CACHE FILEPATH "nm")

# Bare-metal: compiler checks must not try to link a host executable
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# Search programs on the host, libraries/headers only in the target sysroot
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# --- Build type flags (the *_INIT variables are plain variables, no CACHE) ---
foreach(_lang C CXX ASM)
  set(CMAKE_${_lang}_FLAGS_DEBUG_INIT "-O0 -g3")
  set(CMAKE_${_lang}_FLAGS_RELEASE_INIT "-Os -g0 -DNDEBUG")
  set(CMAKE_${_lang}_FLAGS_RELWITHDEBINFO_INIT "-Os -g3 -DNDEBUG")
  set(CMAKE_${_lang}_FLAGS_MINSIZEREL_INIT "-Os -g0 -DNDEBUG")
endforeach()
