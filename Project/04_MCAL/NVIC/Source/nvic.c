/******************************************************************************
 * @file                nvic.c
 * @brief               
 *
 * @details             
 ******************************************************************************/

/******************************************************************************
 * INCLUDES
 * List of header files required by this source file
 ******************************************************************************/
#include "stm32f4xx.h"
#include "core_cm4.h"
#include "nvic.h"

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
void NVICDriver_SetPriority(uint8 IRQn, uint8 priority)
{
  /* Set the priority for the specified IRQ number */
  if (IRQn < 240) // Valid IRQn range for STM32F4
  {
    // Function from CMSIS to set priority
    // Interrupt Priority Register (8Bit wide)
    NVIC_SetPriority((IRQn_Type)IRQn, priority);
  }
}

void NVICDriver_EnableIRQ(uint8 IRQn)
{
  /* Enable the specified IRQ number */
  if (IRQn < 240) // Valid IRQn range for STM32F4
  {
    // Function from CMSIS to enable IRQ
    NVIC_EnableIRQ((IRQn_Type)IRQn);
  }
}

void NVICDriver_DisableIRQ(uint8 IRQn)
{
  /* Disable the specified IRQ number */
  if (IRQn < 240) // Valid IRQn range for STM32F4
  {
    // Function from CMSIS to disable IRQ
    NVIC_DisableIRQ((IRQn_Type)IRQn);
  }
}

/******************************************************************************
 * End of File                                                                *
 ******************************************************************************/
