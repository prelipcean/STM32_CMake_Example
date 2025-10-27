/**
 * @file fpu.c
 * @brief Floating Point Unit (FPU) configuration for STM32F4xx devices.
 * @details This module handles the configuration of the Cortex-M4 FPU.
 *          The STM32F429ZI features a single-precision FPU that supports ARM v7 architecture.
 */

#include "stm32f4xx.h"
#include "core_cm4.h"     /* For SCB access */
#include "fpu.h"

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

/**
 * @brief Enable the Floating Point Unit
 * @details This function enables the FPU by setting CP10 and CP11 to full access in the
 *          Coprocessor Access Control Register (CPACR). Once enabled, the FPU will be
 *          available for single-precision floating point operations.
 * 
 * @note The FPU should be enabled early in the initialization sequence, before any
 *       floating point operations are performed. The FPU state is preserved across
 *       sleep modes.
 * 
 * Register configuration:
 * - CPACR bits [20:21] = CP10 access rights
 * - CPACR bits [22:23] = CP11 access rights
 * Value of 0b11 grants full access
 */
void Fpu_Enable(void)
{
  /* Enable floating point unit: Enable CP10 and CP11 full access */
  SCB->CPACR |= (1u << 20u);  /* CP10 full access */
  SCB->CPACR |= (1u << 21u);
  SCB->CPACR |= (1u << 22u);  /* CP11 full access */
  SCB->CPACR |= (1u << 23u);
}
