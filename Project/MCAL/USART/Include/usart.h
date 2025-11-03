/******************************************************************************
 * @file                usart.h
 * @brief
 *
 * @details
 *
 * @note
 ******************************************************************************/

#ifndef USART_H_
#define USART_H_

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
#define USART1_CLOCK_ENABLE()    (RCC->APB2ENR |= RCC_APB2ENR_USART1EN)
#define USART2_CLOCK_ENABLE()    (RCC->APB1ENR |= RCC_APB1ENR_USART2EN)
#define USART3_CLOCK_ENABLE()    (RCC->APB1ENR |= RCC_APB1ENR_USART3EN)
#define UART4_CLOCK_ENABLE()     (RCC->APB1ENR |= RCC_APB1ENR_UART4EN)
#define UART5_CLOCK_ENABLE()     (RCC->APB1ENR |= RCC_APB1ENR_UART5EN)
#define USART6_CLOCK_ENABLE()    (RCC->APB2ENR |= RCC_APB2ENR_USART6EN)
#define UART7_CLOCK_ENABLE()     (RCC->APB1ENR |= RCC_APB1ENR_UART7EN)
#define UART8_CLOCK_ENABLE()     (RCC->APB1ENR |= RCC_APB1ENR_UART8EN)

#define USART1_CLOCK_DISABLE()   (RCC->APB2ENR &= ~RCC_APB2ENR_USART1EN)
#define USART2_CLOCK_DISABLE()   (RCC->APB1ENR &= ~RCC_APB1ENR_USART2EN)
#define USART3_CLOCK_DISABLE()   (RCC->APB1ENR &= ~RCC_APB1ENR_USART3EN)
#define UART4_CLOCK_DISABLE()    (RCC->APB1ENR &= ~RCC_APB1ENR_UART4EN)
#define UART5_CLOCK_DISABLE()    (RCC->APB1ENR &= ~RCC_APB1ENR_UART5EN)
#define USART6_CLOCK_DISABLE()   (RCC->APB2ENR &= ~RCC_APB2ENR_USART6EN)
#define UART7_CLOCK_DISABLE()    (RCC->APB1ENR &= ~RCC_APB1ENR_UART7EN)
#define UART8_CLOCK_DISABLE()    (RCC->APB1ENR &= ~RCC_APB1ENR_UART8EN)

#define USART_MODE_TX            (0x01U) /* Transmitter mode */
#define USART_MODE_RX            (0x02U) /* Receiver mode */
#define USART_MODE_TX_RX         (0x03U) /* Transmitter and Receiver mode */

#define USART_WORD_LENGTH_8B     (0x00U) /* 8 data bits */
#define USART_WORD_LENGTH_9B     (0x01U) /* 9 data bits */

#define USART_STOPBITS_1         (0x00U) /* 1 stop bit */
#define USART_STOPBITS_0_5       (0x01U) /* 0.5 stop bit */
#define USART_STOPBITS_2         (0x02U) /* 2 stop bits */
#define USART_STOPBITS_1_5       (0x03U) /* 1.5 stop bits */

#define USART_PARITY_NONE        (0x00U) /* No parity */
#define USART_PARITY_EVEN        (0x01U) /* Even parity */
#define USART_PARITY_ODD         (0x02U) /* Odd parity */

#define USART_HW_FLOWCTL_NONE    (0x00U) /* No hardware flow control */
#define USART_HW_FLOWCTL_RTS     (0x01U) /* RTS hardware flow control */
#define USART_HW_FLOWCTL_CTS     (0x02U) /* CTS hardware flow control */
#define USART_HW_FLOWCTL_RTS_CTS (0x03U) /* RTS and CTS hardware flow control */

#define USART_OVERSAMPLING_16    (0x00U) /* Oversampling by 16 */
#define USART_OVERSAMPLING_8     (0x01U) /* Oversampling by 8 */

#define USART_BAUDRATE_9600      (9600U)   /* 9600 baud */
#define USART_BAUDRATE_19200     (19200U)  /* 19200 baud */
#define USART_BAUDRATE_38400     (38400U)  /* 38400 baud */
#define USART_BAUDRATE_57600     (57600U)  /* 57600 baud */
#define USART_BAUDRATE_115200    (115200U) /* 115200 baud */
#define USART_BAUDRATE_230400    (230400U) /* 230400 baud */
#define USART_BAUDRATE_460800    (460800U) /* 460800 baud */
#define USART_BAUDRATE_921600    (921600U) /* 921600 baud */

/******************************************************************************
 * TYPE DEFINITIONS
 * Structures, enumerations, and type aliases
 ******************************************************************************/
typedef struct
{
  uint32 BaudRate;    /**< Communication Baud Rate */
  uint8 Mode;         /**< USART Mode: TX, RX, or TX_RX */
  uint8 WordLength;   /**< Number of data bits */
  uint8 StopBits;     /**< Number of stop bits */
  uint8 Parity;       /**< Parity configuration */
  uint8 HwFlowCtl;    /**< Hardware flow control configuration */
  uint8 OverSampling; /**< Oversampling mode */
} USART_Config_T;

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
void USART_Init(USART_TypeDef *USARTx, const USART_Config_T *config);
void USART_TransmitData(USART_TypeDef *USARTx, uint8 *data, uint8 size);
void USART_ReceiveData(USART_TypeDef *USARTx, uint8 *data, uint8 size);

#ifdef __cplusplus
}
#endif

#endif /* USART_H_ */

/******************************************************************************
 * End of File                                                                *
 ******************************************************************************/
