/******************************************************************************
 * @file                usart1_bsp.c
 * @brief
 *
 * @details
 ******************************************************************************/

/******************************************************************************
 * INCLUDES
 * List of header files required by this source file
 ******************************************************************************/
#include "usart1_bsp.h"
#include "gpio.h"
#include "usart.h"

/******************************************************************************
 * MACRO DEFINITIONS
 * Constants, bit masks, register configurations, etc.
 ******************************************************************************/

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
/* USART1 configuration structure */
STATIC const USART_Config_T usart1Config   = {.BaudRate        = USART_BAUDRATE_9600,
                                              .Mode            = USART_MODE_TX_RX,
                                              .WordLength      = USART_WORD_LENGTH_8B,
                                              .StopBits        = USART_STOPBITS_1,
                                              .Parity          = USART_PARITY_NONE,
                                              .HwFlowCtl       = USART_HW_FLOWCTL_NONE,
                                              .OverSampling    = USART_OVERSAMPLING_16,
                                              .InterruptEnable = 0};

STATIC const USART_Config_T usart1ItConfig = {.BaudRate        = USART_BAUDRATE_9600,
                                              .Mode            = USART_MODE_TX_RX,
                                              .WordLength      = USART_WORD_LENGTH_8B,
                                              .StopBits        = USART_STOPBITS_1,
                                              .Parity          = USART_PARITY_NONE,
                                              .HwFlowCtl       = USART_HW_FLOWCTL_NONE,
                                              .OverSampling    = USART_OVERSAMPLING_16,
                                              .InterruptEnable = 1};

/* Use PA9 Tx AF7 and PA10 Rx AF7: wired to the ST-LINK/V2-B virtual COM port
 * on the STM32F429I-DISC1 (PD8/PD9 are SDRAM data lines on this board) */
STATIC const GPIO_PinConfig_T uart1Tx      = {.pinNumber = GPIO_PIN_NUM_9,
                                              .mode      = GPIO_MODE_AF,
                                              .type      = GPIO_OUTPUT_TYPE_PP,
                                              .speed     = GPIO_SPEED_VERY_HIGH,
                                              .pull      = GPIO_NO_PULL,
                                              .alternate = GPIO_AF7};

STATIC const GPIO_PinConfig_T uart1Rx      = {.pinNumber = GPIO_PIN_NUM_10,
                                              .mode      = GPIO_MODE_AF,
                                              .type      = GPIO_OUTPUT_TYPE_PP,
                                              .speed     = GPIO_SPEED_VERY_HIGH,
                                              .pull      = GPIO_NO_PULL,
                                              .alternate = GPIO_AF7};
static uint8 ReceivedDataBuffer[1];
static uint8 ReceivedDataSize = 1;

/******************************************************************************
 * PUBLIC VARIABLES
 * Global variables (extern)
 ******************************************************************************/

/******************************************************************************
 * PUBLIC CONSTANTS
 * Global constants
 *****************************************************************************/

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
void USART1_Init(void)
{
  /* Initialize GPIO pins for USART1 Tx and Rx */
  GPIO_Init(GPIOA, &uart1Tx);
  GPIO_Init(GPIOA, &uart1Rx);

  /* Initialize USART1 peripheral */
  USART_Init(USART1, &usart1Config);
}

void USART1_It_Init(void)
{
  /* Initialize GPIO pins for USART1 Tx and Rx */
  GPIO_Init(GPIOA, &uart1Tx);
  GPIO_Init(GPIOA, &uart1Rx);

  /* Initialize USART1 peripheral */
  USART_Init(USART1, &usart1ItConfig);
}

void USART1_TransmitData(uint8 *data, uint8 size)
{
  USART_TransmitData(USART1, data, size);
}

void USART1_ReceiveData(uint8 *data, uint8 size)
{
  USART_ReceiveData(USART1, data, size);
}

void USART1_PollingDemo(void)
{
  /* Receive data */
  USART_ReceiveData(USART1, ReceivedDataBuffer, ReceivedDataSize);
  /* Echo back received data over USART1 */
  USART_TransmitData(USART1, ReceivedDataBuffer, ReceivedDataSize);
}

/* Interrupt setup */
void USART1_IRQHandler(void)
{
  /* Check if RXNE flag is set */
  if (USART1->SR & USART_SR_RXNE)
  {
    /* Read received data */
    uint8 receivedByte = (uint8)(USART1->DR & 0xFFU);
    /* Echo back received data */
    USART_TransmitData(USART1, &receivedByte, 1);
  }
}

/******************************************************************************
 * End of File                                                                *
 ******************************************************************************/
