/******************************************************************************
 * @file                swc_controller.h
 * @brief               Application: UART command controller (vim-style H/J/K/L demo).
 *
 * @details             Reads bytes received on USART1 (via the VFB indirection
 *                      layer) and replies with LEFT/DOWN/UP/RIGHT to the H/J/K/L
 *                      commands (case-insensitive); any other received byte is
 *                      echoed back unchanged. Driven from the main loop by
 *                      calling Controller_Runnable() each iteration.
 *
 * @note                Call Controller_Init() once (enables the USART1 RX
 *                      interrupt via the VFB layer) before the first call to
 *                      Controller_Runnable().
 ******************************************************************************/

#ifndef SWC_CONTROLLER_H_
#define SWC_CONTROLLER_H_

#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************************************
 * INCLUDES
 * List of required header files
 ******************************************************************************/

/******************************************************************************
 * MACRO DEFINITIONS
 * Constants, configurations, and function-like macros
 ******************************************************************************/

/******************************************************************************
 * TYPE DEFINITIONS
 * Structures, enumerations, and type aliases
 ******************************************************************************/

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
void Controller_Init(void);
void Controller_Runnable(void);

#ifdef __cplusplus
}
#endif

#endif /* SWC_CONTROLLER_H_ */

/******************************************************************************
 * End of File                                                                *
 ******************************************************************************/
