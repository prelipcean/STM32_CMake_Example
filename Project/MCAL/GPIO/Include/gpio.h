/******************************************************************************
 * @file                gpio.h
 * @brief               GPIO (General Purpose Input/Output) driver interface for STM32F4xx.
 *
 * @details             This header provides the public interface for GPIO configuration and control.
 *                      It includes pin configuration, read/write operations, and interrupt setup.
 ******************************************************************************/

#ifndef GPIO_H_
#define GPIO_H_

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
/**
 * @defgroup GPIO_Pin_Numbers GPIO Pin Numbers
 * @brief Pin numbers for GPIO configuration
 * @{
 */
#define GPIO_PIN_NUM_0        (0U)
#define GPIO_PIN_NUM_1        (1U)
#define GPIO_PIN_NUM_2        (2U)
#define GPIO_PIN_NUM_3        (3U)
#define GPIO_PIN_NUM_4        (4U)
#define GPIO_PIN_NUM_5        (5U)
#define GPIO_PIN_NUM_6        (6U)
#define GPIO_PIN_NUM_7        (7U)
#define GPIO_PIN_NUM_8        (8U)
#define GPIO_PIN_NUM_9        (9U)
#define GPIO_PIN_NUM_10       (10U)
#define GPIO_PIN_NUM_11       (11U)
#define GPIO_PIN_NUM_12       (12U)
#define GPIO_PIN_NUM_13       (13U)
#define GPIO_PIN_NUM_14       (14U)
#define GPIO_PIN_NUM_15       (15U)
/** @} */ // end of GPIO_Pin_Numbers group

/**
 * @defgroup GPIO_Modes GPIO Modes
 * @brief GPIO mode definitions.
 * @{
 */
#define GPIO_MODE_INPUT       (0U)
#define GPIO_MODE_OUTPUT      (1U)
#define GPIO_MODE_AF          (2U)
#define GPIO_MODE_ANALOG      (3U)
/** @} */ // end of GPIO_Modes group

/**
 * @defgroup GPIO_Output_Type GPIO Output Types
 * @brief GPIO output type definitions.
 * @{
 */
#define GPIO_OUTPUT_TYPE_PP   (0U) /**< Push-Pull */
#define GPIO_OUTPUT_TYPE_OD   (1U) /**< Open-Drain */
/** @} */                          // end of GPIO_Output_Type group

/**
 * @defgroup GPIO_Speed GPIO Speed Configurations
 * @brief GPIO speed configurations.
 * @{
 */
#define GPIO_SPEED_LOW        (0U)
#define GPIO_SPEED_MEDIUM     (1U)
#define GPIO_SPEED_HIGH       (2U)
#define GPIO_SPEED_VERY_HIGH  (3U)
/** @} */ // end of GPIO_Speed group

/**
 * @defgroup GPIO_Pull GPIO Pull-Up/Pull-Down Configurations
 * @brief GPIO pull-up/pull-down configurations.
 * @{
 */
#define GPIO_NO_PULL          (0U)
#define GPIO_PULL_UP          (1U)
#define GPIO_PULL_DOWN        (2U)
/** @} */ // end of GPIO_Pull group

/**
 * @defgroup GPIO_Alternate_Function GPIO Alternate Functions
 * @brief GPIO alternate function mappings.
 * @{
 */
#define GPIO_AF0              (0U)
#define GPIO_AF1              (1U)
#define GPIO_AF2              (2U)
#define GPIO_AF3              (3U)
#define GPIO_AF4              (4U)
#define GPIO_AF5              (5U)
#define GPIO_AF6              (6U)
#define GPIO_AF7              (7U)
#define GPIO_AF8              (8U)
#define GPIO_AF9              (9U)
#define GPIO_AF10             (10U)
#define GPIO_AF11             (11U)
#define GPIO_AF12             (12U)
#define GPIO_AF13             (13U)
#define GPIO_AF14             (14U)
#define GPIO_AF15             (15U)
/** @} */ // end of GPIO_Alternate_Function group

#define GPIOA_CLOCK_ENABLE()  (RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN)
#define GPIOB_CLOCK_ENABLE()  (RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN)
#define GPIOC_CLOCK_ENABLE()  (RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN)
#define GPIOD_CLOCK_ENABLE()  (RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN)
#define GPIOE_CLOCK_ENABLE()  (RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN)
#define GPIOF_CLOCK_ENABLE()  (RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN)
#define GPIOG_CLOCK_ENABLE()  (RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN)
#define GPIOH_CLOCK_ENABLE()  (RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN)
#define GPIOI_CLOCK_ENABLE()  (RCC->AHB1ENR |= RCC_AHB1ENR_GPIOIEN)
#define GPIOJ_CLOCK_ENABLE()  (RCC->AHB1ENR |= RCC_AHB1ENR_GPIOJEN)
#define GPIOK_CLOCK_ENABLE()  (RCC->AHB1ENR |= RCC_AHB1ENR_GPIOKEN)

#define GPIOA_CLOCK_DISABLE() (RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOAEN)
#define GPIOB_CLOCK_DISABLE() (RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOBEN)
#define GPIOC_CLOCK_DISABLE() (RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOCEN)
#define GPIOD_CLOCK_DISABLE() (RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIODEN)
#define GPIOE_CLOCK_DISABLE() (RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOEEN)
#define GPIOF_CLOCK_DISABLE() (RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOFEN)
#define GPIOG_CLOCK_DISABLE() (RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOGEN)
#define GPIOH_CLOCK_DISABLE() (RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOHEN)
#define GPIOI_CLOCK_DISABLE() (RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOIEN)
#define GPIOJ_CLOCK_DISABLE() (RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOJEN)
#define GPIOK_CLOCK_DISABLE() (RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOKEN)

/******************************************************************************
 * TYPE DEFINITIONS
 * Structures, enumerations, and type aliases
 ******************************************************************************/

/**
 * @brief GPIO Pin State Enumeration
 * @details Enumeration for GPIO pin states (SET or RESET)
 */
typedef enum
{
  GPIO_PIN_LOW  = 0, /**< GPIO_PIN_LOW (Reset state) */
  GPIO_PIN_HIGH = 1  /**< GPIO_PIN_HIGH (Set state) */
} GPIO_PinState_T;

/**
 * @brief GPIO EXTI Trigger Enumeration
 * @details Enumeration for GPIO external interrupt trigger types
 */
typedef enum
{
  GPIO_EXTI_RISING_EDGE    = 0, /**< Trigger on rising edge */
  GPIO_EXTI_FALLING_EDGE   = 1, /**< Trigger on falling edge */
  GPIO_EXTI_RISING_FALLING = 2  /**< Trigger on both edges */
} GPIO_EXTI_Trigger_T;

/**
 * @brief GPIO Pin Configuration Structure
 * @details Structure containing all configuration parameters for a GPIO pin
 */
typedef struct
{
  uint8 pinNumber; /**< GPIO pin number (0-15) @see GPIO_Pin_Numbers */
  uint8 mode;      /**< GPIO mode (input, output, alternate function, analog) @see GPIO_Mode */
  uint8 type;      /**< GPIO type (push-pull, open-drain) @see GPIO_Output_Type */
  uint8 speed;     /**< GPIO speed (low, medium, high, very high) @see GPIO_Speed */
  uint8 pull;      /**< GPIO pull-up/pull-down configuration @see GPIO_Pull */
  uint8 alternate; /**< GPIO alternate function (if applicable) @see GPIO_Alternate_Function */
  uint8 edgeTrigger; /**< Edge trigger configuration for interrupts (rising, falling, both) */
} GPIO_PinConfig_T;

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
void GPIO_Init(GPIO_TypeDef *GPIOx, const GPIO_PinConfig_T *pinConfig);
void GPIO_DeInit();
void GPIO_WritePin(GPIO_TypeDef *GPIOx, uint8 pinNumber, GPIO_PinState_T value);
uint8 GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint8 pinNumber);
void GPIO_TogglePin(GPIO_TypeDef *GPIOx, uint8 pinNumber);
void GPIO_LockPin(GPIO_TypeDef *GPIOx, uint8 pinNumber);
void GPIO_SetPinInterrupt(GPIO_TypeDef *GPIOx, const GPIO_PinConfig_T *pinConfig, uint8 Priority);

#ifdef __cplusplus
}
#endif

#endif /* GPIO_H_ */

/******************************************************************************
 * End of File                                                                *
 ******************************************************************************/
