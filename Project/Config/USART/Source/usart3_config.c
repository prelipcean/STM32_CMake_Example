/******************************************************************************
 * @file                usart3_config.c
 * @brief               
 *
 * @details             
 ******************************************************************************/

/******************************************************************************
 * INCLUDES
 * List of header files required by this source file
 ******************************************************************************/
#include "usart3_config.h"
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
/* USART3 configuration structure */
STATIC const USART_Config_T usart3Config       = {.BaudRate     = USART_BAUDRATE_9600,
                                                  .Mode         = USART_MODE_TX_RX,
                                                  .WordLength   = USART_WORD_LENGTH_8B,
                                                  .StopBits     = USART_STOPBITS_1,
                                                  .Parity       = USART_PARITY_NONE,
                                                  .HwFlowCtl    = USART_HW_FLOWCTL_NONE,
                                                  .OverSampling = USART_OVERSAMPLING_16};
/* Use PD8 Tx AF7 and PD9 Rx AF7 */
STATIC const GPIO_PinConfig_T uart3Tx          = {.pinNumber = GPIO_PIN_NUM_8,
                                                  .mode      = GPIO_MODE_AF,
                                                  .type      = GPIO_OUTPUT_TYPE_PP,
                                                  .speed     = GPIO_SPEED_VERY_HIGH,
                                                  .pull      = GPIO_NO_PULL,
                                                  .alternate = GPIO_AF7};

STATIC const GPIO_PinConfig_T uart3Rx          = {.pinNumber = GPIO_PIN_NUM_9,
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
void USART3_Init(void)
{
  /* Initialize GPIO pins for USART3 Tx and Rx */
  GPIO_Init(GPIOD, &uart3Tx);
  GPIO_Init(GPIOD, &uart3Rx);

  /* Initialize USART3 peripheral */
  USART_Init(USART3, &usart3Config);
}

void USART3_PollingDemo(void)
{
    /* Receive data */
    USART_ReceiveData(USART3, ReceivedDataBuffer, ReceivedDataSize);
    /* Echo back received data over USART3 */
    USART_TransmitData(USART3, ReceivedDataBuffer, ReceivedDataSize);
}

/******************************************************************************
 * End of File                                                                *
 ******************************************************************************/
