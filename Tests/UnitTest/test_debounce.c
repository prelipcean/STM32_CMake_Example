#include <stdio.h>
#include "unity.h"
#include "debounce.h"

/* ---------------------------------------------------------------------------
 * Fake for the MCAL GPIO read used by Debounce (see mock/stm32f4xx.h for why
 * GPIO_TypeDef is just an opaque stub here)
 * ------------------------------------------------------------------------- */
static GPIO_TypeDef fake_port;
static uint8 fake_rawLevel_u8;

uint8 GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint8 pinNumber)
{
  (void)GPIOx;
  (void)pinNumber;
  return fake_rawLevel_u8;
}

/* Active-low button: pressed = GPIO_PIN_LOW, released = GPIO_PIN_HIGH */
static const Debounce_Config_T config = {
  .port_pst       = &fake_port,
  .pin_u8         = 0U,
  .activeLevel_u8 = GPIO_PIN_LOW,
  .stableTime_u32 = 20U,
};

static Debounce_T instance;

void setUp(void)
{
  fake_rawLevel_u8 = GPIO_PIN_HIGH; /* idle/released */
  Debounce_Init(&instance, &config, 0U);
}

void tearDown(void)
{
}

/* ---------------------------------------------------------------------------
 * Tests
 * ------------------------------------------------------------------------- */
void test_init_seeds_state_without_a_false_trigger(void)
{
  TEST_ASSERT_FALSE(Debounce_IsActive(&instance));
}

void test_short_glitch_is_ignored(void)
{
  fake_rawLevel_u8 = GPIO_PIN_LOW;
  Debounce_Update(&instance, 5U); /* raw changes, timer restarts at t=5 */
  TEST_ASSERT_FALSE(Debounce_IsActive(&instance));

  fake_rawLevel_u8 = GPIO_PIN_HIGH; /* bounces back before the 20ms settle time */
  Debounce_Update(&instance, 10U);
  TEST_ASSERT_FALSE(Debounce_IsActive(&instance));

  Debounce_Update(&instance, 40U); /* HIGH (released) held stable well past 20ms */
  TEST_ASSERT_FALSE(Debounce_IsActive(&instance));
}

void test_press_is_accepted_only_after_the_stable_time(void)
{
  fake_rawLevel_u8 = GPIO_PIN_LOW;
  Debounce_Update(&instance, 5U);
  TEST_ASSERT_FALSE(Debounce_IsActive(&instance));

  Debounce_Update(&instance, 24U); /* 19ms elapsed - one short of the 20ms */
  TEST_ASSERT_FALSE(Debounce_IsActive(&instance));

  Debounce_Update(&instance, 25U); /* 20ms elapsed - now accepted */
  TEST_ASSERT_TRUE(Debounce_IsActive(&instance));
}

void test_release_after_a_press_is_also_debounced(void)
{
  fake_rawLevel_u8 = GPIO_PIN_LOW;
  Debounce_Update(&instance, 0U);
  Debounce_Update(&instance, 20U);
  TEST_ASSERT_TRUE(Debounce_IsActive(&instance));

  fake_rawLevel_u8 = GPIO_PIN_HIGH;
  Debounce_Update(&instance, 20U);                /* raw changes, timer restarts */
  TEST_ASSERT_TRUE(Debounce_IsActive(&instance)); /* still debounced-active */

  Debounce_Update(&instance, 39U); /* 19ms - not yet */
  TEST_ASSERT_TRUE(Debounce_IsActive(&instance));

  Debounce_Update(&instance, 40U); /* 20ms - release accepted */
  TEST_ASSERT_FALSE(Debounce_IsActive(&instance));
}

void test_time_wraparound_does_not_break_debounce(void)
{
  fake_rawLevel_u8 = GPIO_PIN_LOW;
  Debounce_Update(&instance, 0xFFFFFFF0U); /* raw changes just before the millis counter wraps */
  TEST_ASSERT_FALSE(Debounce_IsActive(&instance));

  Debounce_Update(&instance, 0x0000000AU); /* wrapped past 0; 26ms later (mod 2^32) */
  TEST_ASSERT_TRUE(Debounce_IsActive(&instance));
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
  RUN_TEST(test_init_seeds_state_without_a_false_trigger);
  RUN_TEST(test_short_glitch_is_ignored);
  RUN_TEST(test_press_is_accepted_only_after_the_stable_time);
  RUN_TEST(test_release_after_a_press_is_also_debounced);
  RUN_TEST(test_time_wraparound_does_not_break_debounce);
  return UNITY_END();
}
