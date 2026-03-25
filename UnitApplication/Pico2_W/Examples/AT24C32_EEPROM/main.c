/**
 * File: main.c - AT24C32 EEPROM Example
 * Description: Demonstrates how to use the AT24C32 EEPROM driver to write and read back data.
 */

/* Library includes. */
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "AT24C32_HAL.h"

/* Test address within EEPROM address space (0x000-0xFFF) */
#define BYTE_TEST_ADDR  0x000
#define PAGE_TEST_ADDR  0x020  // Second page, starts at byte 32 (page-aligned)
#define READ_TEST_ADDR  0x040  // Third page

/* Number of bytes to use in the page write and sequential read tests */
#define PAGE_TEST_LEN   8

static void print_status(const char *label, AT24C32_Status status)
{
    if (status == AT24C32_OK)
    {
        printf("  [OK]  %s\n", label);
    }
    else
    {
        printf("  [ERR] %s (status=%d)\n", label, status);
    }
}

int main(void)
{
    stdio_init_all();

    printf("\nAT24C32 EEPROM Example\n");
    printf("----------------------\n\n");

    /* Initialize the AT24C32 */
    printf("Initializing AT24C32...\n");
    AT24C32_Status status = AT24C32_Init();
    if (status == AT24C32_ERROR_NO_DEVICE)
    {
        printf("ERROR: AT24C32 not found on I2C bus. Check wiring.\n");
        return 1;
    }
    if (status != AT24C32_OK)
    {
        printf("ERROR: AT24C32 initialization failed (status=%d).\n", status);
        return 1;
    }
    printf("AT24C32 initialized successfully.\n\n");

    /* --- Byte Write / Read --- */
    printf("--- Byte Write / Read ---\n");

    uint8_t byte_to_write = 0xA5;
    uint8_t byte_read_back = 0x00;

    printf("  Writing 0x%02X to address 0x%03X...\n", byte_to_write, BYTE_TEST_ADDR);
    status = AT24C32_WriteByte(BYTE_TEST_ADDR, byte_to_write);
    print_status("WriteByte", status);

    status = AT24C32_ReadByte(BYTE_TEST_ADDR, &byte_read_back);
    print_status("ReadByte", status);

    if (status == AT24C32_OK)
    {
        printf("  Read back: 0x%02X — %s\n\n",
               byte_read_back,
               (byte_read_back == byte_to_write) ? "MATCH" : "MISMATCH");
    }

    /* --- Page Write / Sequential Read --- */
    printf("--- Page Write / Sequential Read ---\n");

    const uint8_t page_data[PAGE_TEST_LEN] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    uint8_t read_buf[PAGE_TEST_LEN] = {0};

    printf("  Writing %d bytes to address 0x%03X...\n", PAGE_TEST_LEN, PAGE_TEST_ADDR);
    status = AT24C32_WritePage(PAGE_TEST_ADDR, page_data, PAGE_TEST_LEN);
    print_status("WritePage", status);

    status = AT24C32_ReadSequential(PAGE_TEST_ADDR, read_buf, PAGE_TEST_LEN);
    print_status("ReadSequential", status);

    if (status == AT24C32_OK)
    {
        printf("  Written: ");
        for (size_t i = 0; i < PAGE_TEST_LEN; i++)
        {
            printf("0x%02X ", page_data[i]);
        }
        printf("\n");

        printf("  Read:    ");
        for (size_t i = 0; i < PAGE_TEST_LEN; i++)
        {
            printf("0x%02X ", read_buf[i]);
        }
        printf("\n");

        printf("  Result:  %s\n\n",
               (memcmp(page_data, read_buf, PAGE_TEST_LEN) == 0) ? "MATCH" : "MISMATCH");
    }

    /* --- String Write / Read --- */
    printf("--- String Write / Read ---\n");

    const char *message = "Hello!";
    size_t msg_len = strlen(message) + 1; // Include null terminator
    char read_str[32] = {0};

    printf("  Writing string \"%s\" to address 0x%03X...\n", message, READ_TEST_ADDR);
    status = AT24C32_WritePage(READ_TEST_ADDR, (const uint8_t *)message, msg_len);
    print_status("WritePage", status);

    status = AT24C32_ReadSequential(READ_TEST_ADDR, (uint8_t *)read_str, msg_len);
    print_status("ReadSequential", status);

    if (status == AT24C32_OK)
    {
        printf("  Read back: \"%s\" — %s\n\n",
               read_str,
               (strcmp(message, read_str) == 0) ? "MATCH" : "MISMATCH");
    }

    printf("Done.\n");
    return 0;
}
