/******************************************************************************
 * @file                runtime.c
 * @brief               Runtime measurement: execution time and CPU load.
 *
 * @details             Busy time is summed from the moment the core wakes up
 *                      until it goes to sleep again. The wake-up timestamp is
 *                      taken with interrupts masked (PRIMASK), so the ISR that
 *                      woke the core runs after it and is counted as busy.
 *                      This works whether or not CYCCNT counts during sleep.
 *                      The window length comes from the SysTick (wall clock).
 ******************************************************************************/

/******************************************************************************
 * INCLUDES
 * List of header files required by this source file
 ******************************************************************************/
#include "stm32f4xx.h"
#include "systick.h"
#include "runtime.h"

/******************************************************************************
 * PRIVATE VARIABLES
 * File-scope variables (static)
 ******************************************************************************/
STATIC Runtime_Stats_T ga_RuntimeStats[RUNTIME_ID_COUNT];

STATIC uint32 g_RuntimeAwakeStart_u32  = 0U; /* CYCCNT when the core woke up */
STATIC uint32 g_RuntimeBusyCycles_u32  = 0U; /* Busy cycles in this window */
STATIC uint32 g_RuntimeWindowStart_u32 = 0U; /* SysTick at window start [ms] */

/******************************************************************************
 * PUBLIC VARIABLES
 * Global variables (extern)
 ******************************************************************************/
/* Volatile: read by the debugger / tools/stm32_probe.py while running */
volatile uint32 g_RuntimeCpuLoad_u32    = 0U; /* Last window [0.01 %] */
volatile uint32 g_RuntimeCpuLoadMax_u32 = 0U; /* Highest window since reset [0.01 %] */

/******************************************************************************
 * PUBLIC FUNCTIONS
 * Global function implementations
 ******************************************************************************/
/**
 * @brief       Enable the DWT cycle counter and start the first load window.
 */
void Runtime_Init(void)
{
  uint8 l_id_u8;

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; /* Enable DWT/ITM */
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  for (l_id_u8 = 0U; l_id_u8 < (uint8)RUNTIME_ID_COUNT; l_id_u8++)
  {
    ga_RuntimeStats[l_id_u8].min_u32 = 0xFFFFFFFFU;
  }

  g_RuntimeWindowStart_u32 = SysTickDriver_GetTick();
  g_RuntimeAwakeStart_u32  = DWT->CYCCNT;
}

void Runtime_Start(Runtime_Id_T id)
{
  ga_RuntimeStats[id].start_u32 = DWT->CYCCNT;
}

void Runtime_Stop(Runtime_Id_T id)
{
  Runtime_Stats_T *lp_stats = &ga_RuntimeStats[id];
  uint32 l_cycles_u32       = DWT->CYCCNT - lp_stats->start_u32; /* Wraps safely */

  lp_stats->last_u32 = l_cycles_u32;
  if (l_cycles_u32 < lp_stats->min_u32)
  {
    lp_stats->min_u32 = l_cycles_u32;
  }
  if (l_cycles_u32 > lp_stats->max_u32)
  {
    lp_stats->max_u32 = l_cycles_u32;
  }
  lp_stats->count_u32++;
}

/**
 * @brief       Sleep until the next interrupt and account the busy time.
 * @note        Call once at the end of every main loop iteration. Wakes up at
 *              least every SysTick (1 ms) and on every enabled interrupt.
 */
void Runtime_Idle(void)
{
  uint32 l_now_u32;
  uint32 l_elapsedMs_u32;

  __disable_irq();
  g_RuntimeBusyCycles_u32 += DWT->CYCCNT - g_RuntimeAwakeStart_u32;
  __DSB();
  __WFI(); /* Wakes on a pending interrupt even with PRIMASK set */
  g_RuntimeAwakeStart_u32 = DWT->CYCCNT;
  __enable_irq(); /* The waking ISR runs here, counted as busy */
  __ISB();

  l_now_u32       = SysTickDriver_GetTick();
  l_elapsedMs_u32 = l_now_u32 - g_RuntimeWindowStart_u32;
  if (l_elapsedMs_u32 >= RUNTIME_WINDOW_MS)
  {
    g_RuntimeCpuLoad_u32 = Runtime_CalcLoad(g_RuntimeBusyCycles_u32, l_elapsedMs_u32 * (SystemCoreClock / 1000U));
    if (g_RuntimeCpuLoad_u32 > g_RuntimeCpuLoadMax_u32)
    {
      g_RuntimeCpuLoadMax_u32 = g_RuntimeCpuLoad_u32;
    }
    g_RuntimeBusyCycles_u32  = 0U;
    g_RuntimeWindowStart_u32 = l_now_u32;
  }
}

/**
 * @brief       CPU load of the last window in 0.01 % (e.g. 125 = 1.25 %).
 */
uint32 Runtime_GetCpuLoad(void)
{
  return g_RuntimeCpuLoad_u32;
}

const Runtime_Stats_T *Runtime_GetStats(Runtime_Id_T id)
{
  return &ga_RuntimeStats[id];
}

/******************************************************************************
 * End of File                                                                *
 ******************************************************************************/
