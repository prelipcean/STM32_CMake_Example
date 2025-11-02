# Set system information
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Auto-detect toolchain path for Windows/Unix
if(UNIX)
  message(STATUS "Unix system detected. Searching for toolchain in /opt/")
  # You might need to adjust this path
  set(ARM_TOOLCHAIN_BIN_DIR /opt/gcc-arm-none-eabi/bin)
  set(TOOLCHAIN_PREFIX arm-none-eabi-)
  set(EXT "")
else()
  message(
    STATUS
      "Non-Unix system (assuming Windows). Searching for toolchain in Program Files"
  )
  set(ARM_TOOLCHAIN_BIN_DIR
      "C:/Program Files (x86)/Arm GNU Toolchain arm-none-eabi/14.3 rel1/bin")
  set(TOOLCHAIN_PREFIX arm-none-eabi-)
  set(EXT ".exe")
endif()

# Check if toolchain exists
if(NOT EXISTS ${ARM_TOOLCHAIN_BIN_DIR})
  message(
    FATAL_ERROR
      "ARM toolchain directory does not exist: ${ARM_TOOLCHAIN_BIN_DIR}")
endif()

# Set toolchain paths
set(CMAKE_C_COMPILER ${ARM_TOOLCHAIN_BIN_DIR}/${TOOLCHAIN_PREFIX}gcc${EXT})
set(CMAKE_CXX_COMPILER ${ARM_TOOLCHAIN_BIN_DIR}/${TOOLCHAIN_PREFIX}g++${EXT})
set(CMAKE_ASM_COMPILER ${ARM_TOOLCHAIN_BIN_DIR}/${TOOLCHAIN_PREFIX}gcc${EXT})
set(CMAKE_LINKER ${ARM_TOOLCHAIN_BIN_DIR}/${TOOLCHAIN_PREFIX}ld${EXT})
set(CMAKE_AR ${ARM_TOOLCHAIN_BIN_DIR}/${TOOLCHAIN_PREFIX}ar${EXT})
set(CMAKE_OBJCOPY ${ARM_TOOLCHAIN_BIN_DIR}/${TOOLCHAIN_PREFIX}objcopy${EXT})
set(CMAKE_OBJDUMP ${ARM_TOOLCHAIN_BIN_DIR}/${TOOLCHAIN_PREFIX}objdump${EXT})
set(CMAKE_SIZE ${ARM_TOOLCHAIN_BIN_DIR}/${TOOLCHAIN_PREFIX}size${EXT})
set(CMAKE_NM ${ARM_TOOLCHAIN_BIN_DIR}/${TOOLCHAIN_PREFIX}nm${EXT})
set(CMAKE_RANLIB ${ARM_TOOLCHAIN_BIN_DIR}/${TOOLCHAIN_PREFIX}ranlib${EXT})

# Essential compiler flags for Cortex-M4F
set(CPU_FLAGS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")
set(COMMON_FLAGS
    "${CPU_FLAGS} -ffunction-sections -fdata-sections -fno-common -fmessage-length=0"
)

set(CMAKE_C_FLAGS_INIT
    "${COMMON_FLAGS}"
    CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS_INIT
    "${COMMON_FLAGS}"
    CACHE STRING "" FORCE)
set(CMAKE_ASM_FLAGS_INIT
    "${COMMON_FLAGS}"
    CACHE STRING "" FORCE)
set(CMAKE_EXE_LINKER_FLAGS_INIT
    "-Wl,--gc-sections"
    CACHE STRING "" FORCE)

# Don't try to link with standard system libraries
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
