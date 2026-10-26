/******************************************************************************
 * @file                systick.h
 * @brief               SysTick time base (millisecond tick).
 *
 * @details             Configures the Cortex-M SysTick timer from SystemCoreClock
 *                      and counts ticks in SysTick_Handler.
 *
 * @note                Call SysTickDriver_Init() after the system clock setup,
 *                      so SystemCoreClock holds the real core frequency.
 ******************************************************************************/

#ifndef SYSTICK_H_
#define SYSTICK_H_

#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************************************
 * INCLUDES
 * List of required header files
 ******************************************************************************/
#include "std_types.h"

/******************************************************************************
 * MACRO DEFINITIONS
 * Constants, configurations, and function-like macros
 ******************************************************************************/
#define SYSTICK_TICK_HZ (1000U) /* 1 tick = 1 ms */

/******************************************************************************
 * API FUNCTIONS
 * Public function prototypes
 ******************************************************************************/
Std_ReturnType SysTickDriver_Init(void);
uint32 SysTickDriver_GetTick(void);
void SysTick_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* SYSTICK_H_ */

/******************************************************************************
 * End of File                                                                *
 ******************************************************************************/
