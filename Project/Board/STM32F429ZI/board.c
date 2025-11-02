#include "stm32f4xx.h"
#include "fpu.h"
#include "sysclock.h"
#include "gpio.h"

/*
 * Two user LEDs: LD3 (green), LD4 (red)
 * The green LED is a user LED connected to the I/O PG13 of the STM32F429ZIT6.
 * The red LED is a user LED connected to the I/O PG14 of the STM32F429ZIT6.
 *
 * One user button: B1 (blue)
 * The blue button is connected to the I/O PA0 of the STM32F429ZIT6.
 *
 * Initialization order:
 * 1. FPU - Enable floating point unit first (core feature)
 * 2. Clock - Configure system clock and bus clocks
 * 3. GPIO - Configure LED pins for status indication
 * */
#define GREEN_LED_PIN   GPIO_PIN_NUM_13
#define RED_LED_PIN     GPIO_PIN_NUM_14
#define BLUE_BUTTON_PIN GPIO_PIN_NUM_0

STATIC const GPIO_PinConfig_T greenLedConfig   = {.pinNumber = GPIO_PIN_NUM_13,
                                                  .mode      = GPIO_MODE_OUTPUT,
                                                  .type      = GPIO_OUTPUT_TYPE_PP,
                                                  .speed     = GPIO_SPEED_MEDIUM,
                                                  .pull      = GPIO_NO_PULL,
                                                  .alternate = GPIO_AF0};

STATIC const GPIO_PinConfig_T redLedConfig     = {.pinNumber = GPIO_PIN_NUM_14,
                                                  .mode      = GPIO_MODE_OUTPUT,
                                                  .type      = GPIO_OUTPUT_TYPE_PP,
                                                  .speed     = GPIO_SPEED_MEDIUM,
                                                  .pull      = GPIO_NO_PULL,
                                                  .alternate = GPIO_AF0};

STATIC const GPIO_PinConfig_T blueButtonConfig = {.pinNumber = GPIO_PIN_NUM_0,
                                                  .mode      = GPIO_MODE_INPUT,
                                                  .type      = GPIO_OUTPUT_TYPE_PP,
                                                  .speed     = GPIO_SPEED_LOW,
                                                  .pull =
                                                    GPIO_NO_PULL, /* Already pull down resistor present on board */
                                                  .alternate = GPIO_AF0};

int Board_Init(void)
{
  /* Optional: Output the clock to a pin for verification */
  SysClock_OutputMCO1(); // Output PLL/4 on PA8
  SysClock_OutputMCO2(); // Output SYSCLK/5 on PC9

  /* Configure green LED (PG13) */
  GPIOG_CLOCK_ENABLE();
  GPIO_Init(GPIOG, &greenLedConfig);

  /* Configure blue user button (PA0) */
  GPIOA_CLOCK_ENABLE();
  GPIO_Init(GPIOA, &blueButtonConfig);

  while (1)
  {
    // TODO: Application code

    /* Toggle green LED to indicate normal operation */
    GPIO_TogglePin(GPIOG, GREEN_LED_PIN);

    if (1u == GPIO_ReadPin(GPIOA, BLUE_BUTTON_PIN))
    {
      /* If button is pressed, turn on red LED */
      GPIO_WritePin(GPIOG, RED_LED_PIN, GPIO_PIN_HIGH);
    }
    else
    {
      /* If button is not pressed, turn off red LED */
      GPIO_WritePin(GPIOG, RED_LED_PIN, GPIO_PIN_LOW);
    }

    /* Simple delay - about 0.5 second at 168MHz */
    for (volatile uint32_t i = 0; i < 5000000; i++);
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
  /* Enable FPU first as it's a core feature and might be needed during clock
   * setup */
  Fpu_Enable();

  /* Configure system clock to HSE and 168MHz using PLL */
  if (SysClock_Setup() != E_OK)
  {
    /* Clock setup failed - flash LED to indicate error */
    while (1)
    {
      /* Enable GPIOG clock */
      GPIOG_CLOCK_ENABLE();
      GPIO_Init(GPIOG, &redLedConfig);
      GPIO_TogglePin(GPIOG, RED_LED_PIN);

      /* Simple delay */
      for (volatile uint32_t i = 0; i < 100000; i++);
    }
  }
  else
  {
    /* Clock setup successful - proceed normally */
    /* Enable GPIOG clock for other usage */
    GPIOG_CLOCK_ENABLE();
    GPIO_Init(GPIOG, &redLedConfig);
  }

  /* Configure the Vector Table location -------------------------------------*/
  // #if defined(USER_VECT_TAB_ADDRESS)
  //   SCB->VTOR = VECT_TAB_BASE_ADDRESS | VECT_TAB_OFFSET; /* Vector Table
  //   Relocation in Internal SRAM */
  // #endif /* USER_VECT_TAB_ADDRESS */
}

/**
 * @brief   Update SystemCoreClock variable according to Clock Register Values.
 *          The SystemCoreClock variable contains the core clock (HCLK), it can
 *          be used by the user application to setup the SysTick timer or
 *          configure other parameters.
 *
 * @note    Each time the core clock (HCLK) changes, this function must be called
 *          to update SystemCoreClock variable value. Otherwise, any
 *          configuration based on this variable will be incorrect.
 *
 * @note    - The system frequency computed by this function is not the real
 *          frequency in the chip. It is calculated based on the predefined
 *          constant and the selected clock source:
 *
 *          - If SYSCLK source is HSI, SystemCoreClock will contain the HSI_VALUE(*)
 *
 *          - If SYSCLK source is HSE, SystemCoreClock will contain the HSE_VALUE(**)
 *
 *          - If SYSCLK source is PLL, SystemCoreClock will contain the
 *          HSE_VALUE(**) or HSI_VALUE(*) multiplied/divided by the PLL factors.
 *
 *          (*) HSI_VALUE is a constant defined in stm32f4xx_hal_conf.h file
 *          (default value 16 MHz) but the real value may vary depending on the
 *          variations in voltage and temperature.
 *
 *          (**) HSE_VALUE is a constant defined in stm32f4xx_hal_conf.h file
 *          (its value depends on the application requirements), user has to ensure that
 *          HSE_VALUE is same as the real frequency of the crystal used. Otherwise, this
 *          function may have wrong result.
 *
 *          - The result of this function could be not correct when using
 *          fractional value for HSE crystal.
 *
 * @param  None
 * @retval None
 */
void SystemCoreClockUpdate(void)
{
  SysClock_UpdateSystemCoreClock();
}
