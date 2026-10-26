/******************************************************************************
 * @file                runtime.h
 * @brief               Runtime measurement: execution time and CPU load.
 *
 * @details             Uses the Cortex-M DWT cycle counter (CYCCNT, 1 count per
 *                      core clock cycle, no peripheral timer and no interrupt).
 *                      - Runtime_Start()/Runtime_Stop() around a runnable record
 *                        its last/min/max execution time in cycles.
 *                      - Runtime_Idle() at the end of the main loop sleeps until
 *                        the next interrupt (WFI). Everything outside it, ISRs
 *                        included, counts as busy. The CPU load of the last
 *                        RUNTIME_WINDOW_MS window is in g_RuntimeCpuLoad_u32.
 *
 *                      Read the results on target by name, e.g.
 *                      tools/stm32_probe.py read --elf <elf> --symbol
 *                      g_RuntimeCpuLoad_u32 g_RuntimeCpuLoadMax_u32
 *
 * @note                Call Runtime_Init() after Board_Init() (needs the SysTick
 *                      time base and the final SystemCoreClock).
 *                      Cycles to microseconds: cycles / (SystemCoreClock / 1e6).
 ******************************************************************************/

#ifndef RUNTIME_H_
#define RUNTIME_H_

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
 * MACRO DEFINITIONS
 * Constants, configurations, and function-like macros
 ******************************************************************************/
#define RUNTIME_WINDOW_MS     (1000U)  /* CPU load averaging window */
#define RUNTIME_LOAD_FULL     (10000U) /* CPU load unit: 0.01 % (10000 = 100 %) */

/******************************************************************************
 * TYPE DEFINITIONS
 * Structures, enums, typedefs, etc.
 ******************************************************************************/
/* Measured sections; add an ID per runnable or ISR you want to time */
typedef enum
{
  RUNTIME_ID_CONTROLLER = 0,
  RUNTIME_ID_HEARTBEAT,
  RUNTIME_ID_COUNT
} Runtime_Id_T;

typedef struct
{
  uint32 start_u32; /**< CYCCNT at Runtime_Start() */
  uint32 last_u32;  /**< Last execution time [cycles] */
  uint32 min_u32;   /**< Shortest execution time [cycles] */
  uint32 max_u32;   /**< Longest execution time [cycles] */
  uint32 count_u32; /**< Number of measurements */
} Runtime_Stats_T;

/******************************************************************************
 * API FUNCTIONS
 * Public function prototypes
 ******************************************************************************/
void Runtime_Init(void);
void Runtime_Start(Runtime_Id_T id);
void Runtime_Stop(Runtime_Id_T id);
void Runtime_Idle(void);
uint32 Runtime_GetCpuLoad(void);
const Runtime_Stats_T *Runtime_GetStats(Runtime_Id_T id);

/**
 * @brief       CPU load in 0.01 % from busy and total cycles of a window.
 * @retval      0 .. RUNTIME_LOAD_FULL (clamped), 0 for an empty window.
 */
LOCAL_INLINE uint32 Runtime_CalcLoad(uint32 busyCycles, uint32 windowCycles)
{
  uint32 l_cyclesPerUnit_u32 = windowCycles / RUNTIME_LOAD_FULL;
  uint32 l_load_u32          = 0U;

  if (0U != l_cyclesPerUnit_u32)
  {
    l_load_u32 = busyCycles / l_cyclesPerUnit_u32;
    if (l_load_u32 > RUNTIME_LOAD_FULL)
    {
      l_load_u32 = RUNTIME_LOAD_FULL;
    }
  }

  return l_load_u32;
}

#ifdef __cplusplus
}
#endif

#endif /* RUNTIME_H_ */

/******************************************************************************
 * End of File                                                                *
 ******************************************************************************/
