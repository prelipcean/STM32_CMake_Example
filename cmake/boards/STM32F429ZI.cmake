# ===================================================================
# Board: STM32F429ZI (Cortex-M4F, 2MB Flash, 192KB RAM + 64KB CCM)
# ===================================================================
# A board file provides:
#   BOARD_CPU_FLAGS     - CPU / FPU flags for compiler and linker
#   BOARD_DEFINITIONS   - device defines (CMSIS device selection)
#   BOARD_STARTUP_FILE  - startup assembly file
#   BOARD_LINKER_SCRIPT - linker script
#   BOARD_BSP_DIR       - folder of the board support sources
#   BOARD_SYSCLOCK_DEFINITIONS - defines for the system clock driver only
# ===================================================================

set(BOARD_BSP_DIR ${PROJECT_SOURCE_DIR}/Project/03_Middleware/BSP/STM32F429ZI)

set(BOARD_CPU_FLAGS -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard
                    -mthumb)

set(BOARD_DEFINITIONS STM32 STM32F4 STM32F429xx STM32F429ZITx USE_FPU)

set(BOARD_STARTUP_FILE ${BOARD_BSP_DIR}/startup/startup_stm32f429zitx.s)
set(BOARD_LINKER_SCRIPT ${BOARD_BSP_DIR}/linker/STM32F429ZITX_FLASH.ld)

# System clock: 168MHz by default, 180MHz with Over-Drive (F42x/43x only).
# Over-Drive needs VDD >= 2.1V; the PLL48 clock (USB/SDIO) is 45MHz at 180MHz.
option(SYSCLOCK_OVERDRIVE "Run SYSCLK at 180MHz using Over-Drive mode" OFF)
if(SYSCLOCK_OVERDRIVE)
  set(BOARD_SYSCLOCK_DEFINITIONS SYSCLOCK_OVERDRIVE)
  message(STATUS "System clock: 180 MHz (Over-Drive)")
else()
  set(BOARD_SYSCLOCK_DEFINITIONS "")
  message(STATUS "System clock: 168 MHz")
endif()
