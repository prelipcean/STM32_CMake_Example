/******************************************************************************
 * @file                board.h
 * @brief               Board support for the STM32F429ZI board.
 *
 * @details             Clock tree, status LEDs and user button setup.
 *
 * @note                Board_Init() must be called from main(), i.e. after the
 *                      startup code initialized .data and .bss.
 ******************************************************************************/

#ifndef BOARD_H_
#define BOARD_H_

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
 * API FUNCTIONS
 * Public function prototypes
 ******************************************************************************/
void Board_Init(void);
void Board_Heartbeat(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_H_ */

/******************************************************************************
 * End of File                                                                *
 ******************************************************************************/
