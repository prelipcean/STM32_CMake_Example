# ===================================================================
# Board: STM32F429ZI (Cortex-M4F, 2MB Flash, 192KB RAM + 64KB CCM)
# ===================================================================
# A board file provides:
#   BOARD_CPU_FLAGS     - CPU / FPU flags for compiler and linker
#   BOARD_DEFINITIONS   - device defines (CMSIS device selection)
#   BOARD_STARTUP_FILE  - startup assembly file
#   BOARD_LINKER_SCRIPT - linker script
#   BOARD_BSP_DIR       - folder of the board support sources
# ===================================================================

set(BOARD_BSP_DIR ${PROJECT_SOURCE_DIR}/Project/03_Middleware/BSP/STM32F429ZI)

set(BOARD_CPU_FLAGS -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard
                    -mthumb)

set(BOARD_DEFINITIONS STM32 STM32F4 STM32F429xx STM32F429ZITx USE_FPU)

set(BOARD_STARTUP_FILE ${BOARD_BSP_DIR}/startup/startup_stm32f429zitx.s)
set(BOARD_LINKER_SCRIPT ${BOARD_BSP_DIR}/linker/STM32F429ZITX_FLASH.ld)
