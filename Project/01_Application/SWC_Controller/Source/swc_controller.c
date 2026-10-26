/******************************************************************************
 * @file                swc_controller.c
 * @brief
 *
 * @details
 ******************************************************************************/

/******************************************************************************
 * INCLUDES
 * List of header files required by this source file
 ******************************************************************************/
#include "swc_controller.h"
#include "vfb_swc_controller.h"

/******************************************************************************
 * MACRO DEFINITIONS
 * Constants, bit masks, register configurations, etc.
 ******************************************************************************/
/* Reply table entry from a string literal (length without the terminating NUL) */
#define CONTROLLER_REPLY(cmd, text)                                                                                    \
  {                                                                                                                    \
    (cmd), (const uint8 *)(text), (uint8)(sizeof(text) - 1U)                                                          \
  }

/******************************************************************************
 * TYPE DEFINITIONS
 * Structures, enums, typedefs, etc.
 ******************************************************************************/
typedef struct
{
  uint8 command_u8;        /**< Received command character (upper case) */
  const uint8 *reply_pu8;  /**< Reply sent back over the UART */
  uint8 replyLength_u8;    /**< Reply length in bytes */
} Controller_Reply_T;

/******************************************************************************
 * PRIVATE VARIABLES
 * File-scope variables (static)
 ******************************************************************************/

/******************************************************************************
 * PRIVATE CONSTANTS
 * File-scope constants (static const)
 ******************************************************************************/
/* Commands are accepted in upper and lower case (vim style h/j/k/l) */
STATIC const Controller_Reply_T ga_ControllerReplies[] = {
    CONTROLLER_REPLY('H', "LEFT\r\n"),
    CONTROLLER_REPLY('J', "DOWN\r\n"),
    CONTROLLER_REPLY('K', "UP\r\n"),
    CONTROLLER_REPLY('L', "RIGHT\r\n"),
};

#define CONTROLLER_REPLY_COUNT (sizeof(ga_ControllerReplies) / sizeof(ga_ControllerReplies[0]))

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
STATIC const Controller_Reply_T *Controller_FindReply(uint8 command);

/******************************************************************************
 * PRIVATE FUNCTIONS
 * File-scope function implementations
 ******************************************************************************/
/**
 * @brief       Look up the reply for a command character (case insensitive).
 * @retval      Reply entry, or NULL_PTR if the character is not a command.
 */
STATIC const Controller_Reply_T *Controller_FindReply(uint8 command)
{
  const Controller_Reply_T *lp_reply = NULL_PTR;
  uint8 l_index_u8;

  if ((command >= (uint8)'a') && (command <= (uint8)'z'))
  {
    command = (uint8)(command - ((uint8)'a' - (uint8)'A'));
  }

  for (l_index_u8 = 0U; l_index_u8 < CONTROLLER_REPLY_COUNT; l_index_u8++)
  {
    if (ga_ControllerReplies[l_index_u8].command_u8 == command)
    {
      lp_reply = &ga_ControllerReplies[l_index_u8];
      break;
    }
  }

  return lp_reply;
}

/******************************************************************************
 * PUBLIC FUNCTIONS
 * Global function implementations
 ******************************************************************************/
void Controller_Init(void)
{
  //VFB_USART1_Init();
  VFB_USART1_It_Init();
}

/**
 * @brief       Handle all received characters: reply to H/J/K/L commands,
 *              echo anything else back unchanged.
 * @note        Non-blocking for reception; replies are sent with polling TX.
 */
void Controller_Runnable(void)
{
  //VFB_USART1_PollingDemo();
  uint8 l_rxByte_u8;
  const Controller_Reply_T *lp_reply;

  while (E_OK == VFB_USART1_ReadByte(&l_rxByte_u8))
  {
    lp_reply = Controller_FindReply(l_rxByte_u8);

    if (NULL_PTR != lp_reply)
    {
      VFB_USART1_TransmitData(lp_reply->reply_pu8, lp_reply->replyLength_u8);
    }
    else
    {
      VFB_USART1_TransmitData(&l_rxByte_u8, 1U);
    }
  }
}

/******************************************************************************
 * End of File                                                                *
 ******************************************************************************/
