#include <stdio.h>
#include <string.h>
#include "unity.h"
#include "swc_controller.h"
#include "usart1_bsp.h"

/* ---------------------------------------------------------------------------
 * Fakes for the USART1 BSP used by the controller (through the VFB)
 * ------------------------------------------------------------------------- */
static uint8 fake_rx[64];
static uint32 fake_rx_len;
static uint32 fake_rx_pos;
static uint8 fake_tx[256];
static uint32 fake_tx_len;
static uint32 fake_it_init_calls;

void USART1_Init(void) {}
void USART1_It_Init(void) { fake_it_init_calls++; }
void USART1_ReceiveData(uint8 *data, uint8 size) { (void)data; (void)size; }
void USART1_PollingDemo(void) {}

Std_ReturnType USART1_ReadByte(uint8 *data)
{
    if (fake_rx_pos < fake_rx_len)
    {
        *data = fake_rx[fake_rx_pos++];
        return E_OK;
    }
    return E_NOT_OK;
}

void USART1_TransmitData(const uint8 *data, uint8 size)
{
    TEST_ASSERT_TRUE_MESSAGE(fake_tx_len + size <= sizeof(fake_tx), "TX fake buffer overflow");
    memcpy(&fake_tx[fake_tx_len], data, size);
    fake_tx_len += size;
}

static void receive(const char *text)
{
    fake_rx_len = (uint32)strlen(text);
    memcpy(fake_rx, text, fake_rx_len);
    fake_rx_pos = 0;
}

static void assert_sent(const char *expected)
{
    TEST_ASSERT_EQUAL_UINT32(strlen(expected), fake_tx_len);
    TEST_ASSERT_EQUAL_MEMORY(expected, fake_tx, fake_tx_len);
}

void setUp(void)
{
    fake_rx_len = fake_rx_pos = fake_tx_len = fake_it_init_calls = 0;
}

void tearDown(void)
{
}

/* ---------------------------------------------------------------------------
 * Tests
 * ------------------------------------------------------------------------- */
void test_init_uses_interrupt_mode(void)
{
    Controller_Init();
    TEST_ASSERT_EQUAL_UINT32(1, fake_it_init_calls);
}

void test_each_command_has_its_reply(void)
{
    receive("H"); Controller_Runnable(); assert_sent("LEFT\r\n");
    setUp(); receive("J"); Controller_Runnable(); assert_sent("DOWN\r\n");
    setUp(); receive("K"); Controller_Runnable(); assert_sent("UP\r\n");
    setUp(); receive("L"); Controller_Runnable(); assert_sent("RIGHT\r\n");
}

void test_lower_case_commands(void)
{
    receive("hjkl");
    Controller_Runnable();
    assert_sent("LEFT\r\nDOWN\r\nUP\r\nRIGHT\r\n");
}

void test_other_characters_are_echoed(void)
{
    receive("xE\r1");
    Controller_Runnable();
    assert_sent("xE\r1");
}

void test_mixed_input_in_order(void)
{
    receive("aHb");
    Controller_Runnable();
    assert_sent("aLEFT\r\nb");
}

void test_nothing_received_nothing_sent(void)
{
    Controller_Runnable();
    TEST_ASSERT_EQUAL_UINT32(0, fake_tx_len);
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
    RUN_TEST(test_init_uses_interrupt_mode);
    RUN_TEST(test_each_command_has_its_reply);
    RUN_TEST(test_lower_case_commands);
    RUN_TEST(test_other_characters_are_echoed);
    RUN_TEST(test_mixed_input_in_order);
    RUN_TEST(test_nothing_received_nothing_sent);
    return UNITY_END();
}
