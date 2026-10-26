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
