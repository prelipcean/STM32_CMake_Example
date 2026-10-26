/******************************************************************************
 * @file                main.c
 * @brief               Application entry point.
 *
 * @details             Called by Reset_Handler after SystemInit() and the
 *                      .data/.bss initialization. Initializes the board, then
 *                      the software components, and runs the main loop.
 ******************************************************************************/

/******************************************************************************
 * INCLUDES
 * List of header files required by this source file
 ******************************************************************************/
#include "board.h"
#include "swc_controller.h"

/******************************************************************************
 * PUBLIC FUNCTIONS
 * Global function implementations
 ******************************************************************************/
int main(void)
{
  /* Board initialization: clocks, LEDs, button */
  Board_Init();

  /* SWC initialization */
  Controller_Init();

  while (1)
  {
    /* SWC runnables */
    Controller_Runnable();

    /* Status LED + simple delay */
    Board_Heartbeat();
  }
}

/******************************************************************************
 * End of File                                                                *
 ******************************************************************************/
