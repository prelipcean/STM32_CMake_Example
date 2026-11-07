/******************************************************************************
 * @file                vfb_swc_controller.h
 * @brief               Virtual Function Bus (VFB) binding SWC_Controller to USART1.
 *
 * @details             Indirection layer between the Application layer
 *                      (SWC_Controller) and the Middleware/BSP layer
 *                      (usart1_bsp). The application calls the hardware-agnostic
 *                      VFB_USART1_* macros; today they forward 1:1 to the
 *                      USART1_* BSP functions, but a different board/BSP
 *                      binding could redefine them without the application
 *                      changing - that indirection is this file's only job.
 *
 * @note                This is the generated binding for SWC_Controller; see
 *                      Project/02_VFB/Generated for the full set of bindings.
 ******************************************************************************/

#ifndef VFB_CONTROLLER_H_
#define VFB_CONTROLLER_H_

#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************************************
 * INCLUDES
 * List of required header files
 ******************************************************************************/
#include "std_types.h"
#include "usart1_bsp.h"

/******************************************************************************
 * MACRO DEFINITIONS
 * Constants, configurations, and function-like macros
 ******************************************************************************/
#define VFB_USART1_Init()                   USART1_Init()
#define VFB_USART1_It_Init()                USART1_It_Init()

#define VFB_USART1_TransmitData(data, size) USART1_TransmitData(data, size)
#define VFB_USART1_ReceiveData(data, size)  USART1_ReceiveData(data, size)
#define VFB_USART1_ReadByte(data)           USART1_ReadByte(data)

#define VFB_USART1_PollingDemo()            USART1_PollingDemo()

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

#ifdef __cplusplus
}
#endif

#endif /* VFB_CONTROLLER_H_ */

/******************************************************************************
 * End of File                                                                *
 ******************************************************************************/
