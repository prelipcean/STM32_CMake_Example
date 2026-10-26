#include <stdio.h>
#include "unity.h"
#include "std_types.h"
#include "reg_util.h"

void setUp(void)
{
}

void tearDown(void)
{
}

/* The platform types must have the same size on the target and on every host */
void test_platform_type_sizes(void)
{
    TEST_ASSERT_EQUAL(1, sizeof(uint8));
    TEST_ASSERT_EQUAL(1, sizeof(sint8));
    TEST_ASSERT_EQUAL(2, sizeof(uint16));
    TEST_ASSERT_EQUAL(2, sizeof(sint16));
    TEST_ASSERT_EQUAL(4, sizeof(uint32));
    TEST_ASSERT_EQUAL(4, sizeof(sint32));
    TEST_ASSERT_EQUAL(8, sizeof(uint64));
    TEST_ASSERT_EQUAL(8, sizeof(sint64));
    TEST_ASSERT_EQUAL(4, sizeof(float32));
}

void test_reg_set_val_only_touches_field(void)
{
    volatile uint32 reg = 0xFFFFFFFFU;

    REG_SET_VAL(reg, 0x5U, 0xFU, 8U);

    TEST_ASSERT_EQUAL_HEX32(0xFFFFF5FFU, reg);
}

void test_reg_bit_macros(void)
{
    volatile uint32 reg = 0U;

    REG_SET_BIT(reg, 31U);
    TEST_ASSERT_EQUAL_HEX32(0x80000000U, reg);
    TEST_ASSERT_EQUAL(1U, REG_READ_BIT(reg, 31U));

    REG_CLR_BIT(reg, 31U);
    TEST_ASSERT_EQUAL_HEX32(0x00000000U, reg);
}

void test_reg_modify(void)
{
    volatile uint32 reg = 0x000000F0U;

    REG_MODIFY(reg, 0x000000F0U, 0x00000003U);

    TEST_ASSERT_EQUAL_HEX32(0x00000003U, reg);
}

// Unity output function implementations
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
    RUN_TEST(test_platform_type_sizes);
    RUN_TEST(test_reg_set_val_only_touches_field);
    RUN_TEST(test_reg_bit_macros);
    RUN_TEST(test_reg_modify);
    return UNITY_END();
}
