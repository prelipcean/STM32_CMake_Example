/******************************************************************************
 * @file                fpu.h
 * @brief               Header file for FPU (Floating Point Unit) configuration.
 *
 * @details             This module provides functions to configure and manage the
 *                      Floating Point Unit on STM32F4xx devices.
 *
 *                      Features of STM32F429ZI FPU:
 *                        - Single-precision floating-point operations
 *                        - IEEE 754 compliant
 *                        - Hardware support for float operations
 *                        - Automatic state preservation and restoration
 *
 * @note                The FPU must be enabled before any floating-point operations
 *                      are performed to prevent usage faults.
 ******************************************************************************/

#ifndef FPU_H_
#define FPU_H_

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
/**
 * @brief       Enable the Floating Point Unit
 *
 * @details     Enables the FPU by configuring CP10 and CP11 in the CPACR register.
 *              After enabling, the FPU can be used for single-precision floating
 *              point operations.
 *
 * @note        This function should be called early in the initialization sequence,
 *              typically before clock configuration and peripheral initialization.
 */
void Fpu_Enable(void);

#ifdef __cplusplus
}
#endif

#endif /* FPU_H_ */

/******************************************************************************
 * End of File                                                                *
 ******************************************************************************/
