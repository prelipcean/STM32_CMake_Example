/******************************************************************************
 * @file                systick.c
 * @brief               SysTick time base (millisecond tick).
 *
 * @details             The tick counter is 32 bit and wraps after ~49.7 days;
 *                      compare ticks with unsigned subtraction (now - start).
 ******************************************************************************/

/******************************************************************************
 * INCLUDES
 * List of header files required by this source file
 ******************************************************************************/
#include "stm32f4xx.h"
#include "systick.h"

/******************************************************************************
 * PRIVATE VARIABLES
 * File-scope variables (static)
 ******************************************************************************/
STATIC volatile uint32 g_SysTickCount_u32 = 0U;

/******************************************************************************
 * PUBLIC FUNCTIONS
 * Global function implementations
 ******************************************************************************/
/**
 * @brief       Start SysTick with a SYSTICK_TICK_HZ interrupt from the core clock.
 * @retval      E_OK on success, E_NOT_OK if the reload value does not fit (24 bit).
 */
Std_ReturnType SysTickDriver_Init(void)
{
  /* CMSIS: sets LOAD/VAL, lowest priority, enables counter and interrupt */
  return (0U == SysTick_Config(SystemCoreClock / SYSTICK_TICK_HZ)) ? E_OK : E_NOT_OK;
}

/**
 * @brief       Ticks (ms) since SysTickDriver_Init().
 */
uint32 SysTickDriver_GetTick(void)
{
  return g_SysTickCount_u32; /* 32 bit aligned read is atomic on Cortex-M */
}

void SysTick_Handler(void)
{
  g_SysTickCount_u32++;
}

/******************************************************************************
 * End of File                                                                *
 ******************************************************************************/
