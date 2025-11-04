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
#include "reg_util.h"
#include "gpio.h"
#include "syscfgctrl.h"
#include "nvic.h"

/******************************************************************************
 * MACRO DEFINITIONS
 * Constants, bit masks, register configurations, etc.
 ******************************************************************************/
#define GPIO_CLK_ENABLE(GPIOx) \
( (GPIOx == GPIOA) ? GPIOA_CLOCK_ENABLE() : \
  (GPIOx == GPIOB) ? GPIOB_CLOCK_ENABLE() : \
  (GPIOx == GPIOC) ? GPIOC_CLOCK_ENABLE() : \
  (GPIOx == GPIOD) ? GPIOD_CLOCK_ENABLE() : \
  (GPIOx == GPIOE) ? GPIOE_CLOCK_ENABLE() : \
  (GPIOx == GPIOF) ? GPIOF_CLOCK_ENABLE() : \
  (GPIOx == GPIOG) ? GPIOG_CLOCK_ENABLE() : \
  (GPIOx == GPIOH) ? GPIOH_CLOCK_ENABLE() : \
  (GPIOx == GPIOI) ? GPIOI_CLOCK_ENABLE() : \
  (GPIOx == GPIOJ) ? GPIOJ_CLOCK_ENABLE() : \
  (GPIOx == GPIOK) ? GPIOK_CLOCK_ENABLE() : (void)0 )

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

#define GPIO_PIN_TO_IRQN(pin)                                                                                          \
  ((pin == 0)                 ? EXTI0_IRQn                                                                             \
   : (pin == 1)               ? EXTI1_IRQn                                                                             \
   : (pin == 2)               ? EXTI2_IRQn                                                                             \
   : (pin == 3)               ? EXTI3_IRQn                                                                             \
   : (pin == 4)               ? EXTI4_IRQn                                                                             \
   : (pin >= 5 && pin <= 9)   ? EXTI9_5_IRQn                                                                           \
   : (pin >= 10 && pin <= 15) ? EXTI15_10_IRQn                                                                         \
                              : 0xFEU)

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
  uint8 l_afrIndex_u8;    /* Index 0 or 1 which indicates AFRL or AFRH */
  uint8 l_afrPosition_u8; /* Position of the alternate function bits */

  /* Enable the GPIO port clock */
  GPIO_CLK_ENABLE(GPIOx);

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

/**
 * @brief       Locks the configuration of a specific GPIO pin.
 * @note        Once a pin is locked, its configuration cannot be changed until
 *              the next system reset. This function follows the lock key
 *              sequence as specified in the STM32F4xx reference manual.
 * @param[in]   GPIOx Pointer to the GPIO peripheral base address (e.g., GPIOA).
 * @param[in]   pinNumber The pin number to lock (0-15).
 * @retval      None
 */
void GPIO_LockPin(GPIO_TypeDef *GPIOx, uint8 pinNumber)
{
  uint32 l_lockKey_u32;

  /* Prepare the lock key sequence */
  l_lockKey_u32 = (1U << pinNumber) | (1U << 16U); // Set LCKK bit along with pin bit

  /* Write the lock key sequence to LCKR register */
  GPIOx->LCKR   = l_lockKey_u32;     // Step 1: Write 1 to LCKK and pin bit
  GPIOx->LCKR   = (1U << pinNumber); // Step 2: Write 0 to LCKK, keep pin bit
  GPIOx->LCKR   = l_lockKey_u32;     // Step 3: Write 1 to LCKK and pin bit again

  /* Read back LCKR to confirm the lock is active */
  (void)GPIOx->LCKR; // Step 4: Read LCKR register (dummy read)
}

/**
 * @brief       Configures a GPIO pin to generate an interrupt on a specified event.
 * @note        This function sets up the EXTI line for the specified pin and
 *              configures the NVIC for the corresponding interrupt.
 * @param[in]   GPIOx Pointer to the GPIO peripheral base address (e.g., GPIOA).
 * @param[in]   pinConfig Pointer to a GPIO_PinConfig_T structure that contains
 *              the configuration information for the specified pin.
 * @param[in]   Priority The priority level for the interrupt.
 * @retval      None
 */
void GPIO_SetPinInterrupt(GPIO_TypeDef *GPIOx, const GPIO_PinConfig_T *pinConfig, uint8 Priority)
{
  uint8 l_extiLine_u8;

  /* 1. Configure SYSCFG */
  /* 1.1 Enable the clock for SYSCFG */
  SYSCFGCTRL_CLOCK_ENABLE();
  /* 1.2 Select the source (GPIO pin) for the respective EXTI line */
  SysCfgCtrl_SetExtiSource(GPIOx, pinConfig->pinNumber);

  /* 2. Configure EXTI */
  // ToDo move EXTI configuration to EXTI driver
  /* 2.1 Select the edge trigger for the interrupt */
  switch (pinConfig->edgeTrigger)
  {
  case GPIO_EXTI_RISING_EDGE:
    EXTI->RTSR |= (1U << pinConfig->pinNumber);  /* Enable rising edge trigger */
    EXTI->FTSR &= ~(1U << pinConfig->pinNumber); /* Disable falling edge trigger */
    break;

  case GPIO_EXTI_FALLING_EDGE:
    EXTI->FTSR |= (1U << pinConfig->pinNumber);  /* Enable falling edge trigger */
    EXTI->RTSR &= ~(1U << pinConfig->pinNumber); /* Disable rising edge trigger */
    break;

  case GPIO_EXTI_RISING_FALLING:
    EXTI->RTSR |= (1U << pinConfig->pinNumber); /* Enable rising edge trigger */
    EXTI->FTSR |= (1U << pinConfig->pinNumber); /* Enable falling edge trigger */
    break;

  default:
    /* Invalid edge trigger configuration - handle error as needed */
    return;
  }
  /* 2.2 Enable the EXTI line interrupt */
  EXTI->IMR |= (1U << pinConfig->pinNumber);

  /* 3. Configure NVIC */
  l_extiLine_u8 = GPIO_PIN_TO_IRQN(pinConfig->pinNumber);

  if (0xFE != l_extiLine_u8)
  {
    /* 3.1 Set the priority for the EXTI line interrupt */
    NVICDriver_SetPriority(l_extiLine_u8, Priority);
    /* 3.2 Enable the EXTI line interrupt in NVIC */
    NVICDriver_EnableIRQ(l_extiLine_u8);
  }
}
/******************************************************************************
 * End of File                                                                *
 ******************************************************************************/
