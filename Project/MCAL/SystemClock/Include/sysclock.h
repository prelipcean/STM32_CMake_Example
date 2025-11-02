/**
 * @file sysclock.h
 * @brief System Clock Configuration Interface for STM32F429ZI
 * @details This module provides functions to configure and manage the system clock
 *          and bus frequencies for the STM32F429ZI microcontroller.
 *
 * Clock Features:
 * - System Clock (SYSCLK): Up to 180 MHz with over-drive mode
 * - AHB Clock (HCLK): Up to 180 MHz
 * - APB1 Clock (PCLK1): Up to 45 MHz
 * - APB2 Clock (PCLK2): Up to 90 MHz
 * - PLL source selectable between HSI (16 MHz) and HSE
 * - MCO1/MCO2 pins for clock output monitoring
 *
 * @note This implementation assumes the use of an external crystal (HSE)
 *       for the main PLL source to achieve maximum accuracy.
 */

#ifndef SYSCLOCK_H_
#define SYSCLOCK_H_

#include "std_types.h"

/**
 * @brief Configures the system clock source, PLL, and bus prescalers.
 * @return E_OK (0) if configuration succeeds, E_NOT_OK (1) if it fails (e.g., oscillator timeout)
 */
uint8_t SysClock_Setup(void);

/**
 * @brief Configures MCO1 (on PA8) to output the main PLL clock.
 */
void SysClock_OutputMCO1(void);

/**
 * @brief Configures MCO2 (on PC9) to output the SYSCLK.
 */
void SysClock_OutputMCO2(void);

/**
 * @brief Updates the CMSIS SystemCoreClock variable.
 * @details This function calculates the current SYSCLK frequency based on
 *          clock register values and updates the global SystemCoreClock
 *          variable used by the CMSIS layer.
 */
void SysClock_UpdateSystemCoreClock(void);

/**
 * @brief Get the current SYSCLK frequency in Hz
 * @return Current SYSCLK frequency in Hz
 */
uint32_t SysClock_GetSYSCLK(void);

/**
 * @brief Get the current HCLK frequency in Hz
 * @return Current HCLK frequency in Hz
 */
uint32_t SysClock_GetHCLK(void);

/**
 * @brief Get the current PCLK1 (APB1) frequency in Hz
 * @return Current PCLK1 frequency in Hz
 */
uint32_t SysClock_GetPCLK1(void);

/**
 * @brief Get the current PCLK2 (APB2) frequency in Hz
 * @return Current PCLK2 frequency in Hz
 */
uint32_t SysClock_GetPCLK2(void);

#endif /* SYSCLOCK_H_ */
