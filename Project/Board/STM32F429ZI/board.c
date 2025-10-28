#include "stm32f4xx.h"
#include "fpu.h"
#include "sysclock.h"

/*
 * Two user LEDs: LD3 (green), LD4 (red)
 * The green LED is a user LED connected to the I/O PG13 of the STM32F429ZIT6.
 * The red LED is a user LED connected to the I/O PG14 of the STM32F429ZIT6.
 * 
 * Initialization order:
 * 1. FPU - Enable floating point unit first (core feature)
 * 2. Clock - Configure system clock and bus clocks
 * 3. GPIO - Configure LED pins for status indication
 * */
#define GPIOGEN                 (1U << 6U)
#define PIN13                   (1U << 13U)
#define PIN14                   (1U << 14U)
#define GPIOG_GREEN_LED_MODER0  (26U)
#define GPIOG_GREEN_LED_MODER1  (27U)

#define GREEN_LED_PIN           PIN13

int Board_Init(void)
{
  /* Optional: Output the clock to a pin for verification */
  SysClock_OutputMCO1(); // Output PLL/4 on PA8
  SysClock_OutputMCO2(); // Output SYSCLK/5 on PC9

  /* Configure green LED (PG13) */
  RCC->AHB1ENR |= GPIOGEN;           // Enable GPIOG clock
  GPIOG->MODER |= (1U << 26);        // Set PG13 as output (bits 27:26 = 01)
  GPIOG->OTYPER &= ~(1U << 13);      // Push-pull output (default, but being explicit)
  GPIOG->OSPEEDR |= (1U << 26);      // Medium speed (bits 27:26 = 01)
  GPIOG->PUPDR &= ~(3U << 26);       // No pull-up, no pull-down (bits 27:26 = 00)

  while(1)
  {
    //TODO: Application code

    /* Toggle green LED to indicate normal operation */
    GPIOG->ODR ^= GREEN_LED_PIN;
    
    /* Simple delay - about 0.5 second at 168MHz */
    for(volatile uint32_t i = 0; i < 5000000; i++);
  }
}

/**
  * @brief  Setup the microcontroller system
  *         Initialize the FPU setting, vector table location and External memory 
  *         configuration.
  * @param  None
  * @retval None
  */
void SystemInit(void)
{
  /* Enable FPU first as it's a core feature and might be needed during clock setup */
  Fpu_Enable();

  /* Configure system clock to HSE and 168MHz using PLL */
  if (SysClock_Setup() != E_OK)
  {
    /* Clock setup failed - flash LED to indicate error */
    while (1)
    {
      /* Enable GPIOG clock */
      RCC->AHB1ENR |= GPIOGEN;
      /* Configure red LED (PG14) as output */
      GPIOG->MODER |= (1U << 28);  // Set PG14 as output
      /* Toggle LED to indicate clock error */
      GPIOG->ODR ^= PIN14;
      /* Simple delay */
      for(volatile uint32_t i = 0; i < 100000; i++);
    }
  }

  /* Configure the Vector Table location -------------------------------------*/
// #if defined(USER_VECT_TAB_ADDRESS)
//   SCB->VTOR = VECT_TAB_BASE_ADDRESS | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal SRAM */
// #endif /* USER_VECT_TAB_ADDRESS */
}

/**
   * @brief  Update SystemCoreClock variable according to Clock Register Values.
  *         The SystemCoreClock variable contains the core clock (HCLK), it can
  *         be used by the user application to setup the SysTick timer or configure
  *         other parameters.
  *           
  * @note   Each time the core clock (HCLK) changes, this function must be called
  *         to update SystemCoreClock variable value. Otherwise, any configuration
  *         based on this variable will be incorrect.         
  *     
  * @note   - The system frequency computed by this function is not the real 
  *           frequency in the chip. It is calculated based on the predefined 
  *           constant and the selected clock source:
  *             
  *           - If SYSCLK source is HSI, SystemCoreClock will contain the HSI_VALUE(*)
  *                                              
  *           - If SYSCLK source is HSE, SystemCoreClock will contain the HSE_VALUE(**)
  *                          
  *           - If SYSCLK source is PLL, SystemCoreClock will contain the HSE_VALUE(**) 
  *             or HSI_VALUE(*) multiplied/divided by the PLL factors.
  *         
  *         (*) HSI_VALUE is a constant defined in stm32f4xx_hal_conf.h file (default value
  *             16 MHz) but the real value may vary depending on the variations
  *             in voltage and temperature.   
  *    
  *         (**) HSE_VALUE is a constant defined in stm32f4xx_hal_conf.h file (its value
  *              depends on the application requirements), user has to ensure that HSE_VALUE
  *              is same as the real frequency of the crystal used. Otherwise, this function
  *              may have wrong result.
  *                
  *         - The result of this function could be not correct when using fractional
  *           value for HSE crystal.
  *     
  * @param  None
  * @retval None
  */
void SystemCoreClockUpdate(void)
{
  SysClock_UpdateSystemCoreClock();
}
