/******************************************************************************
 * @file                syscfgctrl.c
 * @brief
 *
 * @details
 ******************************************************************************/

/******************************************************************************
 * INCLUDES
 * List of header files required by this source file
 ******************************************************************************/
#include "stm32f4xx.h"
#include "syscfgctrl.h"

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
/**
 * @brief     Configures the source GPIO Port for a specific External Interrupt (EXTI) line.
 *
 *            The SYSCFG_EXTICR registers determine which GPIO pin (e.g., PA0, PB0, PC0)
 *            will trigger the EXTI line (EXTI0).
 *
 * @param     GPIOx The base address of the GPIO Port to use (e.g., GPIOA).
 * @param     exti_line The specific EXTI line number to configure (0 to 15).
 * @return    void
 */
void SysCfgCtrl_SetExtiSource(void *GPIOx, uint8 exti_line)
{
  uint8 l_extiPortCode_u8;
  uint8 l_exticrIndex_u8;
  uint8 l_exticrPosition_u8;

  // 1. Calculate the 4-bit port code using the macro
  l_extiPortCode_u8   = SYSCFGCTRL_GET_EXTICR_PORT_CODE(GPIOx);

  // 2. Calculate the register array index (0-3) and the bit position (0, 4, 8, 12)
  //    Each EXTICR register handles 4 EXTI lines, and each line uses a 4-bit field.
  l_exticrIndex_u8    = exti_line / 4U;
  l_exticrPosition_u8 = (exti_line % 4U) * 4U; // 0, 4, 8, or 12

  // Safety check for valid EXTI line index (optional, but good practice)
  if (l_exticrIndex_u8 >= 4U)
  {
    // Handle error, e.g., return an error code or print a debug message
    return;
  }

  // 3. Perform the Read-Modify-Write operation on the SYSCFG EXTICR register

  // Clear the existing 4-bit field (0x0F = 1111 binary)
  SYSCFG->EXTICR[l_exticrIndex_u8] &= ~(0x0FU << l_exticrPosition_u8); // Clear bits

  // Set the new 4-bit port code
  SYSCFG->EXTICR[l_exticrIndex_u8] |= (l_extiPortCode_u8 << l_exticrPosition_u8); // Set bits
}

/******************************************************************************
 * End of File                                                                *
 ******************************************************************************/
