/******************************************************************************
 * @file                controller.c
 * @brief
 *
 * @details
 ******************************************************************************/

/******************************************************************************
 * INCLUDES
 * List of header files required by this source file
 ******************************************************************************/
#include "controller.h"
#include "vfb_controller.h"

/******************************************************************************
 * MACRO DEFINITIONS
 * Constants, bit masks, register configurations, etc.
 ******************************************************************************/
#define CONTROLLER_RX_MESSAGE_SIZE 3U
#define CONTROLLER_TX_MESSAGE_SIZE 3U

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
STATIC volatile uint8 ga_ReceivedMessage_u8[CONTROLLER_RX_MESSAGE_SIZE];
STATIC volatile uint8 ga_SendDownCmd_u8[CONTROLLER_TX_MESSAGE_SIZE]   = "J\n";
STATIC volatile uint8 ga_SendUpCmd_u8[CONTROLLER_TX_MESSAGE_SIZE]     = "K\n";
STATIC volatile uint8 ga_SendRightCmd_u8[CONTROLLER_TX_MESSAGE_SIZE]  = "L\n";
STATIC volatile uint8 ga_SendLeftCmd_u8[CONTROLLER_TX_MESSAGE_SIZE]   = "H\n";
STATIC volatile uint8 ga_SendActionCmd_u8[CONTROLLER_TX_MESSAGE_SIZE] = "E\n";

STATIC volatile uint8 g_TxMessageSize_u8                              = CONTROLLER_TX_MESSAGE_SIZE;
STATIC volatile uint8 g_RxIndex_u8                                    = 0U;
STATIC volatile uint8 g_RxData_u8                                     = 0u;
STATIC volatile boolean g_IsRxAvailable_b                             = FALSE;

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
void Controller_Init(void)
{
  VFB_USART3_Init();
}

void Controller_Runnable(void)
{
  VFB_USART3_PollingDemo();
}

/******************************************************************************
 * End of File                                                                *
 ******************************************************************************/
