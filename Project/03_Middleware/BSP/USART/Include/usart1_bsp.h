/******************************************************************************
 * @file                usart1_bsp.h
 * @brief               
 *
 * @details             
 *
 * @note                
 ******************************************************************************/

#ifndef USART1_BSP_H_
#define USART1_BSP_H_

#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************************************
 * INCLUDES
 * List of required header files
 ******************************************************************************/
#include "std_types.h"

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
void USART1_Init(void);
void USART1_It_Init(void);
void USART1_TransmitData(const uint8 *data, uint8 size);
void USART1_ReceiveData(uint8 *data, uint8 size);
/* Interrupt mode (USART1_It_Init): next received byte, E_NOT_OK if none (non-blocking) */
Std_ReturnType USART1_ReadByte(uint8 *data);

void USART1_PollingDemo(void);

#ifdef __cplusplus
}
#endif

#endif /* USART1_BSP_H_ */

/******************************************************************************
 * End of File                                                                *
 ******************************************************************************/
