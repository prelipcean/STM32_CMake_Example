/******************************************************************************
 * @file                usart1_bsp.h
 * @brief               BSP: USART1 on PA9 (Tx)/PA10 (Rx), AF7 - the ST-LINK/V2-B
 *                      virtual COM port on the STM32F429I-DISC1.
 *
 * @details             Binds the generic MCAL USART and GPIO drivers to this
 *                      specific peripheral instance and pin assignment.
 *                      USART1_Init()/USART1_It_Init() configure the pins and
 *                      peripheral for polling or interrupt-driven reception;
 *                      USART1_ReadByte() pops one byte from the ISR-filled ring
 *                      buffer (non-blocking); USART1_TransmitData()/
 *                      USART1_ReceiveData() wrap the blocking MCAL calls;
 *                      USART1_PollingDemo() is a minimal one-byte echo demo.
 *
 * @note                Use either the polling pair (USART1_Init +
 *                      USART1_ReceiveData/TransmitData) or the interrupt pair
 *                      (USART1_It_Init + USART1_ReadByte) - not both; the RX
 *                      ring buffer is only fed once USART1_It_Init() has
 *                      enabled the interrupt.
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
