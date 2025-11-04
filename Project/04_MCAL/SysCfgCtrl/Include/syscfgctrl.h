/******************************************************************************
 * @file                syscfgctrl.h
 * @brief
 *
 * @details
 *
 * @note
 ******************************************************************************/

#ifndef SYSCFGCTRL_H_
#define SYSCFGCTRL_H_

#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************************************
 * INCLUDES
 * List of required header files
 ******************************************************************************/
#include "std_types.h"
#include "stm32f4xx.h"

/******************************************************************************
 * MACRO DEFINITIONS
 * Constants, configurations, and function-like macros
 ******************************************************************************/
#define SYSCFGCTRL_CLOCK_ENABLE()  (RCC->APB2RSTR |= RCC_APB2RSTR_SYSCFGRST)
#define SYSCFGCTRL_CLOCK_DISABLE() (RCC->APB2RSTR &= ~RCC_APB2RSTR_SYSCFGRST)

// clang-format off
/**
 * @brief Converts a GPIO port base address (e.g., GPIOA) into the
 * 4-bit port selection code required for the SYSCFG External
 * Interrupt Configuration Register (EXTICR) fields.
 *
 * This code is used by the SYSCFG to select which GPIO port is connected
 * to a specific external interrupt line (EXTI line).
 *
 * @param GPIOx The base address of the GPIO Port (e.g., GPIOA, GPIOB, ...).
 * @return The corresponding 4-bit port code (0x0 for GPIOA, 0x1 for GPIOB,
 * up to 0xA for GPIOK). Returns 0x0 if the port address is unrecognized.
 */
#define SYSCFGCTRL_GET_EXTICR_PORT_CODE(GPIOx) \
( ((GPIOx == GPIOA) ? 0x0U : \
    (GPIOx == GPIOB) ? 0x1U : \
    (GPIOx == GPIOC) ? 0x2U : \
    (GPIOx == GPIOD) ? 0x3U : \
    (GPIOx == GPIOE) ? 0x4U : \
    (GPIOx == GPIOF) ? 0x5U : \
    (GPIOx == GPIOG) ? 0x6U : \
    (GPIOx == GPIOH) ? 0x7U : \
    (GPIOx == GPIOI) ? 0x8U : \
    (GPIOx == GPIOJ) ? 0x9U : \
    (GPIOx == GPIOK) ? 0xAU : 0x0U ) )
// clang-format on
/******************************************************************************
 * TYPE DEFINITIONS
 * Structures, enumerations, and type aliases
 ******************************************************************************/

/******************************************************************************
 * CONSTANTS
 * Exported constants and configuration parameters
 ******************************************************************************/

/******************************************************************************
 * EXPORTED VARIABLES
 * Variables visible to other modules (declared as extern here)
 ******************************************************************************/

/******************************************************************************
 * INLINE FUNCTIONS
 * Public inline functions
 ******************************************************************************/

/******************************************************************************
 * API FUNCTIONS
 * Public function prototypes
 ******************************************************************************/
void SysCfgCtrl_SetExtiSource(void *GPIOx, uint8_t exti_line);

#ifdef __cplusplus
}
#endif

#endif /* SYSCFGCTRL_H_ */

/******************************************************************************
 * End of File                                                                *
 ******************************************************************************/
