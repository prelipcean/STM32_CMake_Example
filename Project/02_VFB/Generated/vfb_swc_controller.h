/******************************************************************************
 * @file                vfb_controller.h
 * @brief
 *
 * @details
 *
 * @note
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
#include "usart3_bsp.h"

/******************************************************************************
 * MACRO DEFINITIONS
 * Constants, configurations, and function-like macros
 ******************************************************************************/
#define VFB_USART3_Init()                   USART3_Init()
#define VFB_USART3_It_Init()                USART3_It_Init()

#define VFB_USART3_TransmitData(data, size) USART3_TransmitData(data, size)
#define VFB_USART3_ReceiveData(data, size)  USART3_ReceiveData(data, size)

#define VFB_USART3_PollingDemo()            USART3_PollingDemo()


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
