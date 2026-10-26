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
#include "runtime.h"
#include "swc_controller.h"

/******************************************************************************
 * PUBLIC FUNCTIONS
 * Global function implementations
 ******************************************************************************/
int main(void)
{
  /* Board initialization: clocks, LEDs, button */
  Board_Init();

  /* Runtime measurement: cycle counter, CPU load window */
  Runtime_Init();

  /* SWC initialization */
  Controller_Init();

  while (1)
  {
    /* SWC runnables */
    Runtime_Start(RUNTIME_ID_CONTROLLER);
    Controller_Runnable();
    Runtime_Stop(RUNTIME_ID_CONTROLLER);

    /* Status LED (non-blocking) */
    Runtime_Start(RUNTIME_ID_HEARTBEAT);
    Board_Heartbeat();
    Runtime_Stop(RUNTIME_ID_HEARTBEAT);

    /* Sleep until the next interrupt (SysTick or UART RX); measures CPU load */
    Runtime_Idle();
  }
}

/******************************************************************************
 * End of File                                                                *
 ******************************************************************************/
