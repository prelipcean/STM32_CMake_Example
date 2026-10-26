# ===================================================================
# Platform modules, top down by layer
# ===================================================================
# Add a new module with platform_add_module() (see cmake/platform.cmake).
# It is linked into the firmware automatically.

# ===================================================================
# 01_Application Layer
# ===================================================================
platform_add_module(
  app_main
  SOURCES 01_Application/Main/Source/main.c
  PRIVATE_DEPENDS bsp_board swc_controller)

platform_add_module(
  swc_controller
  SOURCES 01_Application/SWC_Controller/Source/swc_controller.c
  INCLUDES 01_Application/SWC_Controller/Include
  DEPENDS utilities
  PRIVATE_DEPENDS vfb_swc_controller)

# ===================================================================
# 02_VFB Layer
# ===================================================================
platform_add_module(
  vfb_swc_controller
  INCLUDES 02_VFB/Generated
  DEPENDS utilities bsp_usart1)

# ===================================================================
# 03_Middlewares / Services Layer
# ===================================================================
platform_add_module(
  bsp_board
  SOURCES ${BOARD_BSP_DIR}/board.c
  INCLUDES ${BOARD_BSP_DIR}
  DEPENDS utilities
  PRIVATE_DEPENDS cmsis mcal_fpu mcal_sysclock mcal_gpio mcal_systick)

platform_add_module(
  bsp_usart1
  SOURCES 03_Middleware/BSP/USART/Source/usart1_bsp.c
  INCLUDES 03_Middleware/BSP/USART/Include
  DEPENDS utilities
  PRIVATE_DEPENDS cmsis mcal_gpio mcal_usart)

# ===================================================================
# 04_MCAL Layer
# ===================================================================
platform_add_module(
  cmsis
  INCLUDES 04_MCAL/CMSIS/Device/ST/STM32F4xx/Include 04_MCAL/CMSIS/Include)
target_compile_definitions(cmsis INTERFACE ${BOARD_DEFINITIONS})

platform_add_module(
  mcal_fpu
  SOURCES 04_MCAL/FPU/Source/fpu.c
  INCLUDES 04_MCAL/FPU/Include
  DEPENDS utilities
  PRIVATE_DEPENDS cmsis)

platform_add_module(
  mcal_nvic
  SOURCES 04_MCAL/NVIC/Source/nvic.c
  INCLUDES 04_MCAL/NVIC/Include
  DEPENDS utilities
  PRIVATE_DEPENDS cmsis)

platform_add_module(
  mcal_sysclock
  SOURCES 04_MCAL/SystemClock/Source/sysclock.c
  INCLUDES 04_MCAL/SystemClock/Include
  DEPENDS utilities
  PRIVATE_DEPENDS cmsis)
target_compile_definitions(mcal_sysclock PRIVATE ${BOARD_SYSCLOCK_DEFINITIONS})

platform_add_module(
  mcal_systick
  SOURCES 04_MCAL/SysTick/Source/systick.c
  INCLUDES 04_MCAL/SysTick/Include
  DEPENDS utilities
  PRIVATE_DEPENDS cmsis)

platform_add_module(
  mcal_syscfgctrl
  SOURCES 04_MCAL/SysCfgCtrl/Source/syscfgctrl.c
  INCLUDES 04_MCAL/SysCfgCtrl/Include
  DEPENDS utilities cmsis)

platform_add_module(
  mcal_gpio
  SOURCES 04_MCAL/GPIO/Source/gpio.c
  INCLUDES 04_MCAL/GPIO/Include
  DEPENDS utilities cmsis
  PRIVATE_DEPENDS mcal_syscfgctrl mcal_nvic)

platform_add_module(
  mcal_usart
  SOURCES 04_MCAL/USART/Source/usart.c
  INCLUDES 04_MCAL/USART/Include
  DEPENDS utilities cmsis
  PRIVATE_DEPENDS mcal_sysclock mcal_nvic)

# ===================================================================
# Others
# ===================================================================
platform_add_module(utilities INCLUDES YY_Utilities/Include)
