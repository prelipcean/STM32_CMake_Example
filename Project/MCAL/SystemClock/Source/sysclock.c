/**
 * @file sysclock.c
 * @brief System clock configuration for STM32F42x/43x devices.
 * @note This file assumes the use of an external crystal (HSE).
 * The PLL math is based on the HSE_VALUE defined below.
 */

#include "stm32f4xx.h"
#include "reg_util.h"
#include "sysclock.h"

// Add define for HSI if not already defined
#ifndef HSI_VALUE
#define HSI_VALUE 16000000UL
#endif

// ============================================================================
// CONFIGURATION
// ============================================================================

/**
 * @brief Uncomment this line to configure the system clock for 180MHz.
 * This requires Voltage Scale 1 and Over-Drive mode.
 * If commented, the clock will be configured for 168MHz (Voltage Scale 1).
 */
//#define OVERDRIVE

/**
 * @brief Define the external crystal (HSE) frequency in Hz.
 * @note This is typically 8MHz (e.g., ST-LINK MCO) or 25MHz on
 * different boards.
 */
#define HSE_VALUE 8000000UL

/**
 * @brief VCO input frequency in MHz (HSE_VALUE / PLLM divider).
 * Calculated here as HSE_VALUE / 1, assuming PLLM is set to 8 in the PLL configuration
 * to get a 1MHz input to the VCO (VCO_IN = HSE_VALUE / PLLM).
 * This value is used for PLL mathematical calculations (e.g., PLL_VCO = VCO_IN * PLLN).
 */
#define VCO_IN (HSE_VALUE / 1000000UL)

/**
 * @brief Timeout for oscillator startup (in loop iterations).
 * Adjust this based on your startup time requirements.
 */
#define OSC_STARTUP_TIMEOUT 0x5000U

/**
 * @brief Lock flag to prevent clock reconfiguration during critical operations
 */
static volatile uint8 clockConfigLock = 0;

// ============================================================================
// CLOCK LIMITS (STM32F42x/43x)
// ============================================================================
/*
 * Max System Clock (SYSCLK): 180 MHz (with Over-Drive)
 * Max HCLK (AHB bus clock): 180 MHz
 * Max PCLK1 (APB1 bus clock): 45 MHz
 * Max PCLK2 (APB2 bus clock): 90 MHz
 */

/**
 * @brief Get the current SYSCLK frequency in Hz
 * @return Current SYSCLK frequency in Hz
 */
uint32 SysClock_GetSYSCLK(void)
{
  uint32 sysclk = 0;
  uint32 source = RCC->CFGR & RCC_CFGR_SWS;
  
  switch (source)
  {
    case RCC_CFGR_SWS_HSI:
      sysclk = HSI_VALUE;
      break;
    case RCC_CFGR_SWS_HSE:
      sysclk = HSE_VALUE;
      break;
    case RCC_CFGR_SWS_PLL:
      // TODO: Calculate PLL frequency based on source and multipliers
      sysclk = 168000000; // Default to 168MHz
      break;
    default:
      sysclk = HSI_VALUE;
      break;
  }

  return sysclk;
}

/**
 * @brief Get the current HCLK frequency in Hz
 * @return Current HCLK frequency in Hz
 */
uint32 SysClock_GetHCLK(void)
{
  uint32 presc = (RCC->CFGR & RCC_CFGR_HPRE) >> 4;
  uint32 hclk = SysClock_GetSYSCLK();
  
  if (presc & 0x8)
  {
    presc = (0x10 << (presc & 0x7)) >> 1;
    hclk /= presc;
  }
  
  return hclk;
}

/**
 * @brief Get the current PCLK1 (APB1) frequency in Hz
 * @return Current PCLK1 frequency in Hz
 */
uint32 SysClock_GetPCLK1(void)
{
  uint32 presc = (RCC->CFGR & RCC_CFGR_PPRE1) >> 10;
  uint32 pclk1 = SysClock_GetHCLK();
  
  if (presc & 0x4)
  {
    presc = (0x8 << (presc & 0x3)) >> 1;
    pclk1 /= presc;
  }
  
  return pclk1;
}

/**
 * @brief Get the current PCLK2 (APB2) frequency in Hz
 * @return Current PCLK2 frequency in Hz
 */
uint32 SysClock_GetPCLK2(void)
{
  uint32 presc = (RCC->CFGR & RCC_CFGR_PPRE2) >> 13;
  uint32 pclk2 = SysClock_GetHCLK();
  
  if (presc & 0x4)
  {
    presc = (0x8 << (presc & 0x3)) >> 1;
    pclk2 /= presc;
  }
  
  return pclk2;
}
// ============================================================================

/*
 * CMSIS standard global variable.
 * This variable is used by SysTick_Config() and other HAL/LL functions.
 * It MUST be updated whenever the core clock changes.
 */
uint32 SystemCoreClock = HSE_VALUE;

static volatile uint32 timeout = OSC_STARTUP_TIMEOUT;

/**
 * @brief Configures the system clock source, PLL, and bus prescalers.
 * @return uint8 E_OK (0) if configuration succeeds, E_NOT_OK (1) if it fails (e.g., oscillator timeout)
 */
uint8 SysClock_Setup(void)
{
  /* 1. Define peripheral pointers */
  RCC_TypeDef *pRCC = RCC;
  FLASH_TypeDef *pFlash = FLASH;
  PWR_TypeDef *pPWR = PWR;

  /*
   * 2. Reset RCC to default state
   * - HSI ON (16MHz)
   * - HSI selected as SYSCLK
   * - HSE, PLL, PLLSAI OFF
   * - All prescalers = 1
   */
  REG_WRITE(pRCC->CR, 0x00000083U); // Set HSIOM + default trim
  REG_WRITE(pRCC->CFGR, 0x00000000U); // Reset prescalers and clock selection
  // Reset HSEON, CSSON, PLLON, PLLI2SON, PLLSAION bits
  REG_CLR_BIT(pRCC->CR, 16);
  REG_CLR_BIT(pRCC->CR, 19);
  REG_CLR_BIT(pRCC->CR, 24);
  REG_CLR_BIT(pRCC->CR, 26);
  REG_CLR_BIT(pRCC->CR, 28);
  REG_WRITE(pRCC->PLLCFGR, 0x24003010U); // Reset PLL to default
  REG_WRITE(pRCC->CIR, 0x00000000U); // Disable all interrupts

  /*
   * 3. Enable PWR Clock
   * Access to the PWR registers (like VOS) is required.
   */
  REG_SET_BIT(pRCC->APB1ENR, RCC_APB1ENR_PWREN_Pos);

  /*
   * 4. Configure Voltage Scaling and Over-Drive
   * This MUST be done before increasing the clock speed.
   * Scale 1 (VOS=0b11) is required for > 168MHz.
   * Over-Drive is required for > 168MHz (i.e., 180MHz).
   */
#ifdef OVERDRIVE
  // --- 180MHz Configuration ---
  // Set VOS to 0b11 (Scale 1 mode)
  REG_SET_VAL(pPWR->CR, 0x03U, 0x03, PWR_CR_VOS_Pos);
  // Activate over-drive mode
  REG_SET_BIT(pPWR->CR, PWR_CR_ODEN_Pos);
  // Wait for over-drive ready with timeout
  timeout = OSC_STARTUP_TIMEOUT;
  while (REG_READ_BIT(pPWR->CSR, PWR_CSR_ODRDY_Pos) == 0)
  {
    if (timeout-- == 0) return E_NOT_OK; // Failure
  }
  // Over-drive switch enable
  REG_SET_BIT(pPWR->CR, PWR_CR_ODSWEN_Pos);
  // Wait for over-drive switch to be ready
  timeout = OSC_STARTUP_TIMEOUT;
  while (REG_READ_BIT(pPWR->CSR, PWR_CSR_ODSWRDY_Pos) == 0)
  {
    if (timeout-- == 0) return E_NOT_OK; // Failure
  }
#else
  // --- 168MHz Configuration ---
  // Set VOS to 0b11 (Scale 1 mode).
  // While Scale 2 is *technically* ok for 168MHz, Scale 1 gives
  // more margin and is required if 180MHz is ever used.
  REG_SET_VAL(pPWR->CR, 0x03U, 0x03, PWR_CR_VOS_Pos);
#endif

  /*
   * 5. Program FLASH Wait States (Latency)
   * This must be set based on the *target* HCLK and Vcore.
   *
   * From RM0090 (Table 10):
   * VOS Scale 1 (2.7V-3.6V):
   * - 150 < HCLK <= 180 MHz : 5 Wait States (WS)
   * - 120 < HCLK <= 150 MHz : 4 WS
   * ...
   * We are targeting 168MHz or 180MHz, so 5 WS is required.
   */
  REG_SET_VAL(pFlash->ACR, 5u, 0xFu, FLASH_ACR_LATENCY_Pos);

  /*
   * 5b. (CRITICAL FOR PERFORMANCE) Enable FLASH Caches
   * Without these, the CPU will stall constantly fetching instructions.
   */
  REG_SET_BIT(pFlash->ACR, FLASH_ACR_PRFTEN_Pos);  // Prefetch Enable
  REG_SET_BIT(pFlash->ACR, FLASH_ACR_ICEN_Pos);    // Instruction Cache Enable
  REG_SET_BIT(pFlash->ACR, FLASH_ACR_DCEN_Pos);    // Data Cache Enable

  /*
   * 6. Enable High-Speed External Clock (HSE)
   */
  REG_SET_BIT(pRCC->CR, RCC_CR_HSEON_Pos);
  /* Wait for HSE to be stable */
  timeout = OSC_STARTUP_TIMEOUT;
  while (REG_READ_BIT(pRCC->CR, RCC_CR_HSERDY_Pos) == 0)
  {
    if (timeout-- == 0) return E_NOT_OK; // Failure
  }

  /*
   * 7. Configure Bus Prescalers
   * - HPRE: AHB Prescaler (HCLK)
   * - PPRE1: APB1 Prescaler (PCLK1)
   * - PPRE2: APB2 Prescaler (PCLK2)
   */
#ifdef OVERDRIVE
  // Target: SYSCLK = 180MHz
  REG_SET_VAL(pRCC->CFGR, 0u, 0xFu, RCC_CFGR_HPRE_Pos);  // AHB prescaler = 1. HCLK = 180MHz
  REG_SET_VAL(pRCC->CFGR, 5u, 0x7u, RCC_CFGR_PPRE1_Pos); // APB1 prescaler = 4. PCLK1 = 45MHz (max 45)
  REG_SET_VAL(pRCC->CFGR, 4u, 0x7u, RCC_CFGR_PPRE2_Pos); // APB2 prescaler = 2. PCLK2 = 90MHz (max 90)
#else
  // Target: SYSCLK = 168MHz
  REG_SET_VAL(pRCC->CFGR, 0u, 0xFu, RCC_CFGR_HPRE_Pos);  // AHB prescaler = 1. HCLK = 168MHz
  REG_SET_VAL(pRCC->CFGR, 5u, 0x7u, RCC_CFGR_PPRE1_Pos); // APB1 prescaler = 4. PCLK1 = 42MHz (max 45)
  REG_SET_VAL(pRCC->CFGR, 4u, 0x7u, RCC_CFGR_PPRE2_Pos); // APB2 prescaler = 2. PCLK2 = 84MHz (max 90)
#endif

  /*
   * 8. Configure Main PLL (PLL)
   * VCO_IN = HSE / PLLM
   * VCO_OUT = VCO_IN * PLLN
   * SYSCLK = VCO_OUT / PLLP
   * CLK_48MHz = VCO_OUT / PLLQ
   *
   * Target VCO_IN: 1-2MHz (Using 1MHz for 8MHz HSE, 2MHz for 25MHz HSE)
   * Target VCO_OUT: 100-432MHz
   */

  // Set PLL Source to HSE
  REG_SET_BIT(pRCC->PLLCFGR, RCC_PLLCFGR_PLLSRC_Pos);

  // Set PLLM (Input Divider)
  // VCO_IN = 8MHz / 8 = 1MHz
  REG_SET_VAL(pRCC->PLLCFGR, VCO_IN, 0x3Fu, RCC_PLLCFGR_PLLM_Pos);

#ifdef OVERDRIVE
  // --- 180MHz PLL Configuration ---
  // VCO_OUT = 1MHz * 360 = 360MHz
  // SYSCLK = 360MHz / 2 = 180MHz
  REG_SET_VAL(pRCC->PLLCFGR, 360u, 0x1FFu, RCC_PLLCFGR_PLLN_Pos); // Set PLLN = 360
  REG_SET_VAL(pRCC->PLLCFGR, 0u, 0x3u, RCC_PLLCFGR_PLLP_Pos);   // Set PLLP = 2 (0b00)
  // CLK_48MHz = 360MHz / 7.5 -> Not possible. Use 8 for 45MHz.
  // Use 8 for 45MHz, which is OK for USB (if host) or SDIO.
  // If 48MHz is critical, PLL setup must be different.
  REG_SET_VAL(pRCC->PLLCFGR, 8u, 0xFu, RCC_PLLCFGR_PLLQ_Pos);   // Set PLLQ = 8 (for 45MHz)
#else
  // --- 168MHz PLL Configuration ---
  // VCO_OUT = 1MHz * 336 = 336MHz
  // SYSCLK = 336MHz / 2 = 168MHz
  REG_SET_VAL(pRCC->PLLCFGR, 336u, 0x1FFu, RCC_PLLCFGR_PLLN_Pos); // Set PLLN = 336
  REG_SET_VAL(pRCC->PLLCFGR, 0u, 0x3u, RCC_PLLCFGR_PLLP_Pos);   // Set PLLP = 2 (0b00)
  // CLK_48MHz = 336MHz / 7 = 48MHz (Perfect for USB)
  REG_SET_VAL(pRCC->PLLCFGR, 7u, 0xFu, RCC_PLLCFGR_PLLQ_Pos);   // Set PLLQ = 7
#endif

  /*
   * 9. Enable Main PLL and wait for it to be ready
   */
  REG_SET_BIT(pRCC->CR, RCC_CR_PLLON_Pos);
  timeout = OSC_STARTUP_TIMEOUT;
  while (REG_READ_BIT(pRCC->CR, RCC_CR_PLLRDY_Pos) == 0)
  {
    if (timeout-- == 0) return E_NOT_OK; // Failure
  }

  /*
   * 10. Switch System Clock (SYSCLK) to PLL
   */
  REG_SET_VAL(pRCC->CFGR, 2u, 0x3u, RCC_CFGR_SW_Pos);
  /* Wait for switch status to confirm PLL is system clock */
  while (!(REG_READ_VAL(pRCC->CFGR, 0x3u, RCC_CFGR_SWS_Pos) == 0x02u));

  /*
   * 11. Configure and Enable PLLSAI (for LCD)
   * VCO_IN = HSE / PLLM = 1MHz (shared with Main PLL)
   * VCO_SAI_OUT = VCO_IN * PLLSAIN
   * LCD_CLK_RAW = VCO_SAI_OUT / PLLSAIR
   * LCD_CLK = LCD_CLK_RAW / PLLSAIDIVR
   */

  // --- Target: LCD_CLK = 6.25MHz ---
  // VCO_SAI_OUT = 1MHz * 100 = 100MHz
  REG_SET_VAL(pRCC->PLLSAICFGR, 100u, 0x1FFu, RCC_PLLSAICFGR_PLLSAIN_Pos);
  // LCD_CLK_RAW = 100MHz / 2 = 50MHz
  REG_SET_VAL(pRCC->PLLSAICFGR, 2u, 0x7u, RCC_PLLSAICFGR_PLLSAIR_Pos);
  // LCD_CLK = 50MHz / 8 = 6.25MHz (PLLSAIDIVR=0b10 means Div 8)
  REG_SET_VAL(pRCC->DCKCFGR, 2u, 0x3u, RCC_DCKCFGR_PLLSAIDIVR_Pos);

  // SAI_CLK (for I2S, etc.) = 100MHz / 4 = 25MHz
  REG_SET_VAL(pRCC->PLLSAICFGR, 4u, 0xFu, RCC_PLLSAICFGR_PLLSAIQ_Pos);

  /* Turn on PLLSAI for LCD */
  REG_SET_BIT(pRCC->CR, RCC_CR_PLLSAION_Pos);
  /* Wait until PLLSAI clock ready bit is set */
  timeout = OSC_STARTUP_TIMEOUT;
  while (REG_READ_BIT(pRCC->CR, RCC_CR_PLLSAIRDY_Pos) == 0)
  {
    if (timeout-- == 0) return E_NOT_OK; // Failure
  }

  /*
   * System clock setup is complete.
   * Update CMSIS SystemCoreClock variable
   */
  SysClock_UpdateSystemCoreClock();

  return E_OK; // Success
}

/**
 * @brief Updates the CMSIS SystemCoreClock variable.
 * @details This function calculates the current SYSCLK frequency based on
 *          clock register values and updates the global SystemCoreClock
 *          variable used by the CMSIS layer.
 * @note This implementation is based on the standard CMSIS function but
 *       renamed to follow module naming conventions.
 */
void SysClock_UpdateSystemCoreClock(void)
{
  uint32 pllm = 0, plln = 0, pllp = 0;
  uint32 sysclk = 0;
  uint32 src = 0;

  /* Get SYSCLK source */
  src = RCC->CFGR & RCC_CFGR_SWS;

  switch (src) {
    case 0x00:  // HSI used as system clock source
      sysclk = HSI_VALUE;
      break;
    case 0x04:  // HSE used as system clock source
      sysclk = HSE_VALUE;
      break;
    case 0x08:  // PLL used as system clock source
      /* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLLM) * PLLN
         SYSCLK = PLL_VCO / PLLP */
      pllm = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;
      plln = (RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6;
      pllp = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >> 16) + 1) * 2;
      
      if (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) {
        /* HSE used as PLL clock source */
        sysclk = (HSE_VALUE / pllm) * plln / pllp;
      } else {
        /* HSI used as PLL clock source */
        sysclk = (HSI_VALUE / pllm) * plln / pllp;
      }
      break;
    default:
      sysclk = HSI_VALUE;
      break;
  }

  /* HCLK = SYSCLK / AHB prescaler */
  uint32 ahbpre = (RCC->CFGR & RCC_CFGR_HPRE) >> 4;
  if (ahbpre & 0x08) {
    ahbpre = (0x10U << (ahbpre & 0x07)) >> 1;
    sysclk /= ahbpre;
  }

  /* Update SystemCoreClock global variable */
  SystemCoreClock = sysclk;
}

/**
 * @brief Configures MCO1 (on PA8) to output the main PLL clock.
 * @note Call this function *after* SysClock_Setup() has successfully run.
 * * MCO1 Pin: PA8
 * MCO1 Source: Main PLL clock
 * MCO1 Prescaler: /4
 * * Output Freq:
 * - 180MHz / 4 = 45.0 MHz
 * - 168MHz / 4 = 42.0 MHz
 */
void SysClock_OutputMCO1(void)
{
  /* 1. Enable GPIOA Clock */
  // MCO1 is on pin PA8
  REG_SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN_Pos);

  /* 2. Configure MCO1 Clock Source and Prescaler */
  REG_SET_VAL(RCC->CFGR, 3u, 0x3u, RCC_CFGR_MCO1_Pos);
  REG_SET_VAL(RCC->CFGR, 6u, 0x7u, RCC_CFGR_MCO1PRE_Pos);

  /* 3. Configure PA8 to Alternate Function (AF0) */
  // Set MODER to Alternate Function (0b10)
  REG_MODIFY(GPIOA->MODER, GPIO_MODER_MODER8, GPIO_MODER_MODER8_1);

  // Set OSPEEDR to High Speed (0b11) for fast clock output
  REG_SET_VAL(GPIOA->OSPEEDR, 3u, 0x3u, GPIO_OSPEEDR_OSPEED8_Pos);

  // Set AFR[1] (High Register) for PA8 to AF0 (MCO)
  // AF0 is 0b0000.
  REG_MODIFY(GPIOA->AFR[1], GPIO_AFRH_AFSEL8, (0x00U << GPIO_AFRH_AFSEL8_Pos));
}

/**
 * @brief Configures MCO2 (on PC9) to output the SYSCLK.
 * @note Call this function *after* SysClock_Setup() has successfully run.
 * * MCO2 Pin: PC9
 * MCO2 Source: SYSCLK
 * MCO2 Prescaler: /5 (to keep it under 50MHz for many logic analyzers)
 * * Output Freq:
 * - 180MHz / 5 = 36.0 MHz
 * - 168MHz / 5 = 33.6 MHz
 */
void SysClock_OutputMCO2(void)
{
  /* 1. Enable GPIOC Clock */
  // MCO2 is on pin PC9
  REG_SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN_Pos);

  /* 2. Configure MCO2 Clock Source and Prescaler */
  REG_SET_VAL(RCC->CFGR, 3u, 0x3u, RCC_CFGR_MCO2_Pos);
  REG_SET_VAL(RCC->CFGR, 7u, 0x7u, RCC_CFGR_MCO2PRE_Pos);

  /* 3. Configure PC9 to Alternate Function (AF0) */
  // Set MODER to Alternate Function (0b10)
  REG_MODIFY(GPIOC->MODER, GPIO_MODER_MODER9, GPIO_MODER_MODER9_1);

  // Set OSPEEDR to High Speed (0b11)
  REG_SET_VAL(GPIOC->OSPEEDR, 3u, 0x3u, GPIO_OSPEEDR_OSPEED9_Pos);

  // Set AFR[1] (High Register) for PC9 to AF0 (MCO)
  // AF0 is 0b0000.
  REG_MODIFY(GPIOC->AFR[1], GPIO_AFRH_AFSEL9, (0x00U << GPIO_AFRH_AFSEL9_Pos));
}
