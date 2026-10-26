#include <stdio.h>
#include "unity.h"
#include "runtime.h"

#define CORE_HZ (168000000U) /* One 1 s window at 168 MHz */

void setUp(void)
{
}

void tearDown(void)
{
}

void test_load_idle_and_full(void)
{
    TEST_ASSERT_EQUAL_UINT32(0U, Runtime_CalcLoad(0U, CORE_HZ));
    TEST_ASSERT_EQUAL_UINT32(RUNTIME_LOAD_FULL, Runtime_CalcLoad(CORE_HZ, CORE_HZ));
}

void test_load_resolution_is_0_01_percent(void)
{
    TEST_ASSERT_EQUAL_UINT32(5000U, Runtime_CalcLoad(CORE_HZ / 2U, CORE_HZ));  /* 50.00 % */
    TEST_ASSERT_EQUAL_UINT32(1U, Runtime_CalcLoad(CORE_HZ / 10000U, CORE_HZ)); /*  0.01 % */
}

void test_load_clamped_when_busy_exceeds_window(void)
{
    TEST_ASSERT_EQUAL_UINT32(RUNTIME_LOAD_FULL, Runtime_CalcLoad(2U * CORE_HZ, CORE_HZ));
}

void test_load_empty_window(void)
{
    TEST_ASSERT_EQUAL_UINT32(0U, Runtime_CalcLoad(1234U, 0U));
    TEST_ASSERT_EQUAL_UINT32(0U, Runtime_CalcLoad(1234U, RUNTIME_LOAD_FULL - 1U));
}

/* 180 MHz over-drive, 1.5 s late window: no 32 bit overflow */
void test_load_long_window_at_180mhz(void)
{
    TEST_ASSERT_EQUAL_UINT32(2500U, Runtime_CalcLoad(67500000U, 270000000U));
}

void unity_output_char(int c)
{
    putchar(c);
}

void unity_output_flush(void)
{
    fflush(stdout);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_load_idle_and_full);
    RUN_TEST(test_load_resolution_is_0_01_percent);
    RUN_TEST(test_load_clamped_when_busy_exceeds_window);
    RUN_TEST(test_load_empty_window);
    RUN_TEST(test_load_long_window_at_180mhz);
    return UNITY_END();
}
