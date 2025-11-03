/******************************************************************************
 * @file                usart.c
 * @brief
 *
 * @details
 ******************************************************************************/

/******************************************************************************
 * INCLUDES
 * List of header files required by this source file
 ******************************************************************************/
#include "stm32f4xx.h"
#include "usart.h"
#include "sysclock.h"

/******************************************************************************
 * MACRO DEFINITIONS
 * Constants, bit masks, register configurations, etc.
 ******************************************************************************/
#define USARTX_CLK_ENABLE(USARTx)                                                                                      \
  ((USARTx == USART1)   ? USART1_CLOCK_ENABLE()                                                                        \
   : (USARTx == USART2) ? USART2_CLOCK_ENABLE()                                                                        \
   : (USARTx == USART3) ? USART3_CLOCK_ENABLE()                                                                        \
   : (USARTx == UART4)  ? UART4_CLOCK_ENABLE()                                                                         \
   : (USARTx == UART5)  ? UART5_CLOCK_ENABLE()                                                                         \
   : (USARTx == USART6) ? USART6_CLOCK_ENABLE()                                                                        \
   : (USARTx == UART7)  ? UART7_CLOCK_ENABLE()                                                                         \
   : (USARTx == UART8)  ? UART8_CLOCK_ENABLE()                                                                         \
                        : (void)0)

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
void USART_Init(USART_TypeDef *USARTx, const USART_Config_T *config)
{
  uint32 l_Mantissa;
  uint32 l_Fraction;
  uint32 l_Remainder;
  uint32 l_ScalingFactor;
  uint32 l_USARTxCLK;

  /* Step 1. Enable clock for USART */
  USARTX_CLK_ENABLE(USARTx);

  /* Step 2. Set USART mode in CR1 register */
  USARTx->CR1 &= ~(USART_CR1_TE | USART_CR1_RE); /* Clear TE and RE bits */
  if (USART_MODE_TX == config->Mode)
  {
    USARTx->CR1 |= USART_CR1_TE; /* Enable Transmitter */
  }
  else if (USART_MODE_RX == config->Mode)
  {
    USARTx->CR1 |= USART_CR1_RE; /* Enable Receiver */
  }
  else if (USART_MODE_TX_RX == config->Mode)
  {
    USARTx->CR1 |= (USART_CR1_TE | USART_CR1_RE); /* Enable both */
  }

  /* Step 3. Set wordlength in CR1 register */
  if (USART_WORD_LENGTH_9B == config->WordLength)
  {
    USARTx->CR1 |= USART_CR1_M; /* 9 data bits */
  }
  else /* Default to 8 bits */
  {
    USARTx->CR1 &= ~USART_CR1_M; /* 8 data bits */
  }

  /* Step 4. Set stop bits in CR2 register */
  USARTx->CR2 &= ~USART_CR2_STOP; /* Clear STOP bits */
  switch (config->StopBits)
  {
  case USART_STOPBITS_0_5:
    USARTx->CR2 |= (0x1U << USART_CR2_STOP_Pos); /* 0.5 stop bit */
    break;
  case USART_STOPBITS_2:
    USARTx->CR2 |= (0x2U << USART_CR2_STOP_Pos); /* 2 stop bits */
    break;
  case USART_STOPBITS_1_5:
    USARTx->CR2 |= (0x3U << USART_CR2_STOP_Pos); /* 1.5 stop bits */
    break;
  case USART_STOPBITS_1:
  default:
    /* 1 stop bit - default */
    break;
  }

  /* Step 5. Set parity in CR1 register */
  USARTx->CR1 &= ~USART_CR1_PCE; /* Clear PCE bit */
  switch (config->Parity)
  {
  case USART_PARITY_EVEN:
    USARTx->CR1 |= USART_CR1_PCE; /* Enable parity control */
    USARTx->CR1 &= ~USART_CR1_PS; /* Even parity */
    break;
  case USART_PARITY_ODD:
    USARTx->CR1 |= USART_CR1_PCE; /* Enable parity control */
    USARTx->CR1 |= USART_CR1_PS;  /* Odd parity */
    break;
  case USART_PARITY_NONE:
  default:
    /* No parity - default */
    break;
  }

  /* Step 6. Set hardware flow control in CR3 register */
  USARTx->CR3 &= ~(USART_CR3_CTSE | USART_CR3_RTSE); /* Clear CTSE and RTSE bits */
  switch (config->HwFlowCtl)
  {
  case USART_HW_FLOWCTL_RTS:
    USARTx->CR3 |= USART_CR3_RTSE; /* Enable RTS */
    break;
  case USART_HW_FLOWCTL_CTS:
    USARTx->CR3 |= USART_CR3_CTSE; /* Enable CTS */
    break;
  case USART_HW_FLOWCTL_RTS_CTS:
    USARTx->CR3 |= (USART_CR3_CTSE | USART_CR3_RTSE); /* Enable both */
    break;
  case USART_HW_FLOWCTL_NONE:
  default:
    /* No hardware flow control - default */
    break;
  }

  /* Step 7. Set oversampling in CR1 register */
  if (USART_OVERSAMPLING_8 == config->OverSampling)
  {
    USARTx->CR1 |= USART_CR1_OVER8; /* Oversampling by 8 */
  }
  else /* Default to oversampling by 16 */
  {
    USARTx->CR1 &= ~USART_CR1_OVER8; /* Oversampling by 16 */
  }

  /* Step 8. Set baud rate in BRR register */
  /* Get USARTx clock frequency */
  if (USARTx == USART1 || USARTx == USART6)
  {
    /* APB2 peripheral clock */
    l_USARTxCLK = SysClock_GetPCLK2();
  }
  else
  {
    /* APB1 peripheral clock */
    l_USARTxCLK = SysClock_GetPCLK1();
  }

  /* Calculate USARTDIV value for BRR register */
  l_ScalingFactor = 8 * (2 - config->OverSampling);
  l_Mantissa      = (l_USARTxCLK / (l_ScalingFactor * config->BaudRate));
  l_Remainder     = (l_USARTxCLK % (l_ScalingFactor * config->BaudRate));
  l_Fraction      = ((l_Remainder) + (config->BaudRate / 2)) / config->BaudRate; /* Rounding */
  /* Overflow handling */
  if (l_Fraction >= l_ScalingFactor)
  {
    l_Mantissa++;
    l_Fraction = 0;
  }

  USARTx->BRR = 0; /* Clear BRR register */
  USARTx->BRR = (((l_Mantissa << USART_BRR_DIV_Mantissa_Pos) & USART_BRR_DIV_Mantissa_Msk) |
                 ((l_Fraction << USART_BRR_DIV_Fraction_Pos) & USART_BRR_DIV_Fraction_Msk));

  /* Step 9. Enable USART peripheral in CR1 register */
  USARTx->CR1 |= USART_CR1_UE; /* Enable USART */
}

void USART_TransmitData(USART_TypeDef *USARTx, uint8 *data, uint8 size)
{
  uint8 *lp_data8Bit;
  uint16 *lp_data9Bit;
  uint8 l_txBuffCounter = 0;

  l_txBuffCounter       = size;

  /* Check wordlength to use 8 bit pointer or 9 bit pointer */
  if ((USART_WORD_LENGTH_9B == ((USARTx->CR1 >> USART_CR1_M_Pos) & 0x1u)) &&
      USART_PARITY_NONE == ((USARTx->CR1 >> USART_CR1_PS_Pos) & 0x1u))
  {
    lp_data8Bit = NULL_PTR;
    lp_data9Bit = (uint16 *)data;
  }
  else
  {
    lp_data9Bit = NULL_PTR;
    lp_data8Bit = data;
  }

  /* Loop through all characters in the data */
  while (l_txBuffCounter > 0)
  {
    /* Wait until TXE (Transmit Data Register Empty) flag is set */
    while (!(USARTx->SR & USART_SR_TXE))
    {
      /* Wait */
    }

    /* Transmit data based on word length */
    if (lp_data9Bit != NULL_PTR)
    {
      USARTx->DR = (*lp_data9Bit & 0x1FFU); /* Transmit 9 bits */
      lp_data9Bit++;                        /* Move to next 9-bit data */
    }
    else
    {
      USARTx->DR = (*lp_data8Bit & 0xFFU); /* Transmit 8 bits */
      lp_data8Bit++;                       /* Move to next 8-bit data */
    }
    /* Decrement counter buffer by 1 */
    l_txBuffCounter--;
  }
}

void USART_ReceiveData(USART_TypeDef *USARTx, uint8 *data, uint8 size)
{
  uint8 *lp_data8Bit;
  uint16 *lp_data9Bit;
  uint8 l_rxBuffCounter = 0;

  l_rxBuffCounter       = size;

  /* Check wordlength to use 8 bit pointer or 9 bit pointer */
  if ((USART_WORD_LENGTH_9B == ((USARTx->CR1 >> USART_CR1_M_Pos) & 0x1u)) &&
      USART_PARITY_NONE == ((USARTx->CR1 >> USART_CR1_PS_Pos) & 0x1u))
  {
    lp_data8Bit = NULL_PTR;
    lp_data9Bit = (uint16 *)data;
  }
  else
  {
    lp_data9Bit = NULL_PTR;
    lp_data8Bit = data;
  }

  /* Loop through all characters in the data */
  while (l_rxBuffCounter > 0)
  {
    /* Wait until RXNE */
    while (!(USARTx->SR & USART_SR_RXNE))
    {
      /* Wait */
    }

    /* Receive data based on word length */
    if (lp_data9Bit != NULL_PTR)
    {
      *lp_data9Bit = (uint16)(USARTx->DR & 0x1FFU); /* Receive 9 bits */
      lp_data9Bit++;                                /* Move to next 9-bit data */
    }
    else
    {
      if ((USART_WORD_LENGTH_9B == ((USARTx->CR1 >> USART_CR1_M_Pos) & 0x1u)) ||
          ((USART_WORD_LENGTH_8B == ((USARTx->CR1 >> USART_CR1_M_Pos) & 0x1u)) &&
           (USART_PARITY_NONE == ((USARTx->CR1 >> USART_CR1_PS_Pos) & 0x1u))))
      {
        *lp_data8Bit = (uint8)(USARTx->DR & 0xFFU); /* Receive 8 bits */
        lp_data8Bit++;                              /* Move to next 8-bit data */
      }
      else
      {
        *lp_data8Bit = (uint8)(USARTx->DR & 0x7FU); /* Receive 7 bits (parity used) */
        lp_data8Bit++;                              /* Move to next 8-bit data */
      }
    }

    /* Decrement counter buffer by 1 */
    l_rxBuffCounter--;
  }
}

/******************************************************************************
 * End of File                                                                *
 ******************************************************************************/
