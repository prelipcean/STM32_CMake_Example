#ifndef MOCK_STM32F4XX_H
#define MOCK_STM32F4XX_H

// Mock definitions for testing
#define __FPU_USED 1

typedef struct {
    volatile uint32_t CPACR;
} SCB_Type;

extern SCB_Type SCB;

#endif /* MOCK_STM32F4XX_H */