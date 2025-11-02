/******************************************************************************
 * @file                gpio.c
 * @brief               GPIO (General Purpose Input/Output) driver implementation for STM32F4xx.
 *
 * @details             This driver provides functions to configure and control GPIO pins
 *                      for the STM32F4xx microcontroller series.
 ******************************************************************************/

/******************************************************************************
 * INCLUDES
 * List of header files required by this source file
 ******************************************************************************/
#include "stm32f4xx.h"
#include "gpio.h"
#include "reg_util.h"

/******************************************************************************
 * MACRO DEFINITIONS
 * Constants, bit masks, register configurations, etc.
 ******************************************************************************/
/**
 * @def GPIO_MODER_MASK(pin)
 * @brief Mask for 2 bits per pin in MODER register
 */
#define GPIO_MODER_MASK(pin)   (0x03U << ((pin) * 2))

/**
 * @def GPIO_OSPEEDR_MASK(pin)
 * @brief Mask for 2 bits per pin in OSPEEDR register
 */
#define GPIO_OSPEEDR_MASK(pin) (0x03U << ((pin) * 2))

/**
 * @def GPIO_PUPDR_MASK(pin)
 * @brief Mask for 2 bits per pin in PUPDR register
 */
#define GPIO_PUPDR_MASK(pin)   (0x03U << ((pin) * 2))

/**
 * @def GPIO_AFR_MASK(pin)
 * @brief Mask for 4 bits per pin in AFRL/H register
 */
#define GPIO_AFR_MASK          (0x0FU)

/******************************************************************************
 * TYPE DEFINITIONS
 * Structures, enums, typedefs, etc.
 ******************************************************************************/

/******************************************************************************
 * PRIVATE VARIABLES
 * File-scope variables (static)
 ******************************************************************************/

/******************************************************************************
 * PRIVATE CONSTANTS
 * File-scope constants (static const)
 ******************************************************************************/

/******************************************************************************
 * PUBLIC VARIABLES
 * Global variables (extern)
 ******************************************************************************/

/******************************************************************************
 * PUBLIC CONSTANTS
 * Global constants
 ******************************************************************************/

/******************************************************************************
 * PRIVATE FUNCTION PROTOTYPES
 * File-scope function declarations
 ******************************************************************************/

/******************************************************************************
 * PRIVATE FUNCTIONS
 * File-scope function implementations
 ******************************************************************************/

/******************************************************************************
 * PUBLIC FUNCTIONS
 * Global function implementations
 ******************************************************************************/

/**
 * @brief       Initializes a single GPIO pin with the specified configuration.
 * @note        This function configures the pin's mode, output type, speed,
 *              pull-up/pull-down resistor, and alternate function (if applicable).
 * @param[in]   GPIOx Pointer to the GPIO peripheral base address (e.g., GPIOA, GPIOB).
 * @param[in]   pinConfig Pointer to a GPIO_PinConfig_T structure that contains
 *              the configuration information for the specified pin.
 * @retval None
 */
void GPIO_Init(GPIO_TypeDef *GPIOx, const GPIO_PinConfig_T *pinConfig)
{
  uint8 l_afrIndex_u8;    /* In dex 0 or 1 which indicates AFRL or AFRH */
  uint8 l_afrPosition_u8; /* Position of the alternate function bits */

  /* Configure the pin mode */
  GPIOx->MODER &= ~GPIO_MODER_MASK(pinConfig->pinNumber);          // Clear mode bits
  GPIOx->MODER |= (pinConfig->mode << (pinConfig->pinNumber * 2)); // Set mode bits

  if ((GPIO_MODE_OUTPUT == pinConfig->mode) || (GPIO_MODE_AF == pinConfig->mode))
  {
    /* Configure the output type */
    if (pinConfig->type == 0) // Push-pull
    {
      GPIOx->OTYPER &= ~(1U << pinConfig->pinNumber);
    }
    else // Open-drain
    {
      GPIOx->OTYPER |= (1U << pinConfig->pinNumber);
    }

    /* Configure the output speed */
    GPIOx->OSPEEDR &= ~GPIO_OSPEEDR_MASK(pinConfig->pinNumber);         // Clear speed bits
    GPIOx->OSPEEDR |= (pinConfig->speed << (pinConfig->pinNumber * 2)); // Set speed bits
  }

  /* Configure the pull-up/pull-down resistors */
  GPIOx->PUPDR &= ~GPIO_PUPDR_MASK(pinConfig->pinNumber);          // Clear pull bits
  GPIOx->PUPDR |= (pinConfig->pull << (pinConfig->pinNumber * 2)); // Set pull bits

  /* Configure the alternate function if applicable */
  if (GPIO_MODE_AF == pinConfig->mode)
  {
    /* Note:
     * value 8 number of pins in a register
     * value 4 for number of bits for a pin AF
     */
    l_afrIndex_u8    = pinConfig->pinNumber / 8;
    l_afrPosition_u8 = (pinConfig->pinNumber % 8) * 4;
    GPIOx->AFR[l_afrIndex_u8] &= ~(GPIO_AFR_MASK << l_afrPosition_u8);       // Clear AF bits
    GPIOx->AFR[l_afrIndex_u8] |= (pinConfig->alternate << l_afrPosition_u8); // Set AF bits
  }
}

/**
 * @brief     De-initializes all GPIO ports (A-K) to their default reset values.
 * @note      This function mimics the hardware power-on reset state for the GPIO
 *            peripherals. The reset values for Port A and Port B are different
 *            from other ports (which are all 0).
 * @param     None
 * @retval    None
 */
void GPIO_DeInit(void)
{
  /* Array of ports C through K, which share the same 0x0 reset values */
  GPIO_TypeDef *common_ports[] = {GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH, GPIOI, GPIOJ, GPIOK};

  /* 1. Reset Port A (Special Case) */
  REG_WRITE(GPIOA->MODER, 0xA8000000U);
  REG_WRITE(GPIOA->OTYPER, 0x00000000U);
  REG_WRITE(GPIOA->OSPEEDR, 0x0C000000U);
  REG_WRITE(GPIOA->PUPDR, 0x64000000U);
  REG_WRITE(GPIOA->AFR[0], 0x00000000U); // AFRL
  REG_WRITE(GPIOA->AFR[1], 0x00000000U); // AFRH

  /* 2. Reset Port B (Special Case) */
  REG_WRITE(GPIOB->MODER, 0x00000280U);
  REG_WRITE(GPIOB->OTYPER, 0x00000000U);
  REG_WRITE(GPIOB->OSPEEDR, 0x000000C0U);
  REG_WRITE(GPIOB->PUPDR, 0x00000100U);
  REG_WRITE(GPIOB->AFR[0], 0x00000000U); // AFRL
  REG_WRITE(GPIOB->AFR[1], 0x00000000U); // AFRH

  /* 3. Reset all other ports (C through K) in a loop */
  for (int i = 0; i < (sizeof(common_ports) / sizeof(common_ports[0])); i++)
  {
    REG_WRITE(common_ports[i]->MODER, 0x00000000U);
    REG_WRITE(common_ports[i]->OTYPER, 0x00000000U);
    REG_WRITE(common_ports[i]->OSPEEDR, 0x00000000U);
    REG_WRITE(common_ports[i]->PUPDR, 0x00000000U);
    REG_WRITE(common_ports[i]->AFR[0], 0x00000000U); // AFRL
    REG_WRITE(common_ports[i]->AFR[1], 0x00000000U); // AFRH
  }
}

/**
 * @brief       Sets or clears a specific output pin.
 * @note        This function uses the BSRR register for atomic bit set/reset operations.
 *              This avoids read-modify-write issues in interrupt-driven or
 *              multi-threaded environments.
 * @param[in]   GPIOx Pointer to the GPIO peripheral base address (e.g., GPIOA).
 * @param[in]   pinNumber The pin number to write to (0-15).
 * @param[in]   value The state to set the pin to (e.g., GPIO_PIN_LOW or GPIO_PIN_HIGH).
 * @retval      None
 */
void GPIO_WritePin(GPIO_TypeDef *GPIOx, uint8 pinNumber, GPIO_PinState_T value)
{
  if (GPIO_PIN_LOW == value)
  {
    /* Clear the output pin */
    GPIOx->BSRR = (1U << (pinNumber + 16U));
    /* Note: Writing to the upper half of BSRR register resets the pin */
    /* Another method is to use ODR register */
    // GPIOx->ODR &= ~(1U << pinNumber);
  }
  else
  {
    /* Set the output pin */
    GPIOx->BSRR = (1U << pinNumber);
    /* Note: Writing to the lower half of BSRR register sets the pin */
    /* Another method is to use ODR register */
    // GPIOx->ODR |= (1U << pinNumber);
  }
}

/**
 * @brief       Reads the current state of a specific GPIO input pin.
 * @note        This function reads the pin's state directly from the Input Data Register (IDR).
 * @param[in]   GPIOx Pointer to the GPIO peripheral base address (e.g., GPIOA).
 * @param[in]   pinNumber The pin number to read from (0-15).
 * @retval      uint8 The state of the pin (0 for low, 1 for high).
 */
uint8 GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint8 pinNumber)
{
  uint8 l_pinState_u8;

  l_pinState_u8 = (uint8)((GPIOx->IDR >> pinNumber) & 0x01U);

  return l_pinState_u8;
}

/**
 * @brief       Toggles (inverts) the current state of a specific output pin.
 * @note        This function performs a read-modify-write (RMW) operation on the
 *              Output Data Register (ODR). This operation is **not atomic** and
 *              can lead to race conditions if the same port is modified by an
 *              interrupt service routine (ISR) or another thread concurrently.
 * @param[in]   GPIOx Pointer to the GPIO peripheral base address (e.g., GPIOA).
 * @param[in]   pinNumber The pin number to toggle (0-15).
 * @retval      None
 */
void GPIO_TogglePin(GPIO_TypeDef *GPIOx, uint8 pinNumber)
{
  GPIOx->ODR ^= (1U << pinNumber);
}

/******************************************************************************
 * End of File                                                                *
 ******************************************************************************/
