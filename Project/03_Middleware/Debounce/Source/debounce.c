/******************************************************************************
 * @file                debounce.c
 * @brief               Time-based debounce for a single digital input.
 *
 * @details             See debounce.h for the algorithm description.
 ******************************************************************************/

/******************************************************************************
 * INCLUDES
 * List of header files required by this source file
 ******************************************************************************/
#include "debounce.h"

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
void Debounce_Init(Debounce_T *instance_pst, const Debounce_Config_T *config_pcst, uint32 currentTime_u32)
{
  uint8 l_rawLevel_u8             = GPIO_ReadPin(config_pcst->port_pst, config_pcst->pin_u8);

  instance_pst->config_pcst       = config_pcst;
  instance_pst->debouncedLevel_u8 = l_rawLevel_u8;
  instance_pst->rawLevel_u8       = l_rawLevel_u8;
  instance_pst->rawChangeTime_u32 = currentTime_u32;
}

void Debounce_Update(Debounce_T *instance_pst, uint32 currentTime_u32)
{
  uint8 l_rawLevel_u8 = GPIO_ReadPin(instance_pst->config_pcst->port_pst, instance_pst->config_pcst->pin_u8);

  if (l_rawLevel_u8 != instance_pst->rawLevel_u8)
  {
    /* Raw level just changed: (re)start the stability timer */
    instance_pst->rawLevel_u8       = l_rawLevel_u8;
    instance_pst->rawChangeTime_u32 = currentTime_u32;
  }
  else if ((l_rawLevel_u8 != instance_pst->debouncedLevel_u8) &&
           ((uint32)(currentTime_u32 - instance_pst->rawChangeTime_u32) >= instance_pst->config_pcst->stableTime_u32))
  {
    /* Raw level has held steady for long enough: accept it */
    instance_pst->debouncedLevel_u8 = l_rawLevel_u8;
  }
}

uint8 Debounce_IsActive(const Debounce_T *instance_pst)
{
  return (instance_pst->debouncedLevel_u8 == instance_pst->config_pcst->activeLevel_u8) ? 1U : 0U;
}

/******************************************************************************
 * End of File                                                                *
 ******************************************************************************/
