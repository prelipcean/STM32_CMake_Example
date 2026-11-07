#ifndef MOCK_STM32F4XX_H
#define MOCK_STM32F4XX_H

#include <stdint.h>

// Mock definitions for testing
#define __FPU_USED 1

typedef struct
{
  volatile uint32_t CPACR;
} SCB_Type;

extern SCB_Type SCB;

/* Opaque GPIO port type: just enough for gpio.h to compile on host. The
 * register layout doesn't matter here because every test that needs one
 * fakes GPIO_ReadPin()/GPIO_WritePin() directly instead of touching real
 * registers (see Tests/UnitTest/test_debounce.c) - the mock port is never
 * dereferenced, only passed around as a pointer. */
typedef struct
{
  volatile uint32_t DUMMY;
} GPIO_TypeDef;

#endif /* MOCK_STM32F4XX_H */
