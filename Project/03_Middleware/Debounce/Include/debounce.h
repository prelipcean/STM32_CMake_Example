/******************************************************************************
 * @file                debounce.h
 * @brief               Time-based debounce for a single digital input.
 *
 * @details             Filters the raw level of one GPIO pin so a noisy
 *                      mechanical switch (button, limit switch, ...) reads as
 *                      a single clean transition instead of a burst of
 *                      bounces. Call Debounce_Update() periodically (e.g. once
 *                      per main loop iteration or SysTick tick) with the
 *                      current time; a raw level change only becomes the
 *                      reported (debounced) level once it has held steady for
 *                      Debounce_Config_T::stableTime_u32 milliseconds.
 *
 *                      The module reads exactly one hardware input per
 *                      Debounce_Update() call (GPIO_ReadPin()); everything
 *                      else is pure, deterministic logic over the instance
 *                      state and the caller-supplied time, which is what
 *                      makes it host-unit-testable without any board (see
 *                      Tests/UnitTest/test_debounce.c).
 *
 * @note                Debounce_Init() seeds the instance with one raw read
 *                      so the very first Debounce_Update() call cannot be
 *                      mistaken for a transition. Each Debounce_T instance is
 *                      independent, so one pin per instance - multiple
 *                      buttons need multiple instances.
 ******************************************************************************/

#ifndef DEBOUNCE_H_
#define DEBOUNCE_H_

#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************************************
 * INCLUDES
 * List of required header files
 ******************************************************************************/
#include "std_types.h"
#include "gpio.h"

/******************************************************************************
 * MACRO DEFINITIONS
 * Constants, configurations, and function-like macros
 ******************************************************************************/

/** @defgroup Debounce_Macros Debounce Macros
 * @{
 */
#define DEBOUNCE_DEFAULT_STABLE_TIME_MS (20U) /**< Typical mechanical-switch settle time */
/** @} */                                     /* End of Debounce_Macros group */

/******************************************************************************
 * TYPE DEFINITIONS
 * Structures, enumerations, and type aliases
 ******************************************************************************/

/** @defgroup Debounce_Types Debounce Types
 * @{
 */

/**
 * @brief       Static configuration for one debounced input.
 */
typedef struct
{
  GPIO_TypeDef *port_pst; /**< GPIO port of the monitored pin */
  uint8 pin_u8;           /**< Pin number on that port @see GPIO_Pin_Numbers */
  uint8 activeLevel_u8;   /**< Raw level (GPIO_PIN_LOW/GPIO_PIN_HIGH) considered "active" */
  uint32 stableTime_u32;  /**< Raw level must hold this long [ms] before it is accepted */
} Debounce_Config_T;

/**
 * @brief       Runtime state for one debounced input.
 * @note        Treat as opaque; read it only through Debounce_IsActive().
 */
typedef struct
{
  const Debounce_Config_T *config_pcst; /**< Configuration this instance was initialized with */
  uint8 debouncedLevel_u8;              /**< Last accepted (debounced) raw level */
  uint8 rawLevel_u8;                    /**< Raw level read on the previous Debounce_Update() */
  uint32 rawChangeTime_u32;             /**< Timestamp [ms] the raw level last changed */
} Debounce_T;

/** @} */ /* End of Debounce_Types group */

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

/** @defgroup Debounce_API Debounce API
 * @{
 */

/**
 * @brief       Initialize an instance and seed it with the current raw level.
 * @param[out]  instance_pst    Instance to initialize.
 * @param[in]   config_pcst     Configuration to use (must outlive the instance).
 * @param[in]   currentTime_u32 Current time [ms], same clock Debounce_Update() will use.
 */
void Debounce_Init(Debounce_T *instance_pst, const Debounce_Config_T *config_pcst, uint32 currentTime_u32);

/**
 * @brief       Read the pin once and advance the debounce state machine.
 * @param[in,out] instance_pst  Instance to update.
 * @param[in]     currentTime_u32 Current time [ms]; must be non-decreasing between
 *                                calls except for one wraparound of the whole range.
 */
void Debounce_Update(Debounce_T *instance_pst, uint32 currentTime_u32);

/**
 * @brief       Current debounced state.
 * @param[in]   instance_pst    Instance to read.
 * @retval      1 if the debounced level equals Debounce_Config_T::activeLevel_u8, else 0.
 */
uint8 Debounce_IsActive(const Debounce_T *instance_pst);

/** @} */ /* End of Debounce_API group */

#ifdef __cplusplus
}
#endif

#endif /* DEBOUNCE_H_ */

/******************************************************************************
 * End of File                                                                *
 ******************************************************************************/
