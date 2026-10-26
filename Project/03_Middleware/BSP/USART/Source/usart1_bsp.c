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
/* RX ring buffer for interrupt mode, size must be a power of 2 (max 256) */
#define USART1_RX_BUFFER_SIZE (64U)
#define USART1_RX_BUFFER_MASK (USART1_RX_BUFFER_SIZE - 1U)

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

/* Single producer (ISR) / single consumer (USART1_ReadByte): no locking needed,
 * the ISR only writes the head, the reader only writes the tail */
STATIC volatile uint8 ga_Usart1RxBuffer_u8[USART1_RX_BUFFER_SIZE];
STATIC volatile uint8 g_Usart1RxHead_u8          = 0U;
STATIC volatile uint8 g_Usart1RxTail_u8          = 0U;
STATIC volatile uint32 g_Usart1RxOverflowCount_u32 = 0U; /* bytes dropped, buffer full */

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

void USART1_TransmitData(const uint8 *data, uint8 size)
{
  USART_TransmitData(USART1, data, size);
}

void USART1_ReceiveData(uint8 *data, uint8 size)
{
  USART_ReceiveData(USART1, data, size);
}

Std_ReturnType USART1_ReadByte(uint8 *data)
{
  Std_ReturnType l_ret = E_NOT_OK;
  uint8 l_tail_u8      = g_Usart1RxTail_u8;

  if (l_tail_u8 != g_Usart1RxHead_u8)
  {
    *data             = ga_Usart1RxBuffer_u8[l_tail_u8];
    g_Usart1RxTail_u8 = (uint8)((l_tail_u8 + 1U) & USART1_RX_BUFFER_MASK);
    l_ret             = E_OK;
  }

  return l_ret;
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
  uint8 l_head_u8;
  uint8 l_next_u8;

  /* Check if RXNE flag is set (also set on overrun) */
  if (USART1->SR & USART_SR_RXNE)
  {
    /* Reading DR (after SR) clears RXNE and ORE */
    uint8 receivedByte = (uint8)(USART1->DR & 0xFFU);

    /* Queue it for the application (USART1_ReadByte), no work in the ISR */
    l_head_u8          = g_Usart1RxHead_u8;
    l_next_u8          = (uint8)((l_head_u8 + 1U) & USART1_RX_BUFFER_MASK);
    if (l_next_u8 != g_Usart1RxTail_u8)
    {
      ga_Usart1RxBuffer_u8[l_head_u8] = receivedByte;
      g_Usart1RxHead_u8               = l_next_u8;
    }
    else
    {
      g_Usart1RxOverflowCount_u32++;
    }
  }
}

/******************************************************************************
 * End of File                                                                *
 ******************************************************************************/
