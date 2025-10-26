/**
 * @file fpu.h
 * @brief Header file for FPU (Floating Point Unit) configuration.
 * @details This module provides functions to configure and manage the
 *          Floating Point Unit on STM32F4xx devices.
 *
 * Features of STM32F429ZI FPU:
 * - Single-precision floating-point operations
 * - IEEE 754 compliant
 * - Hardware support for float operations
 * - Automatic state preservation and restoration
 *
 * @note The FPU must be enabled before any floating-point operations
 *       are performed to prevent usage faults.
 */

#ifndef FPU_H_
#define FPU_H_

#include "stm32f4xx.h"

/**
 * @brief Enable the Floating Point Unit
 *
 * @details Enables the FPU by configuring CP10 and CP11 in the CPACR register.
 *          After enabling, the FPU can be used for single-precision floating
 *          point operations.
 *
 * @note This function should be called early in the initialization sequence,
 *       typically before clock configuration and peripheral initialization.
 */
void Fpu_Enable(void);

/**
 * @def FPU_CCR_ASPEN_Pos
 * @brief Position of Automatic State Preservation Enable bit in CCR
 */
#define FPU_CCR_ASPEN_Pos    (31U)

/**
 * @def FPU_CCR_LSPEN_Pos
 * @brief Position of Lazy State Preservation Enable bit in CCR
 */
#define FPU_CCR_LSPEN_Pos    (30U)

#endif /* FPU_H_ */
