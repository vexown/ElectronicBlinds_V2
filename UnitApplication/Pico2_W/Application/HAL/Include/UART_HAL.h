/**
 * @file UART_HAL.h
 * @brief Hardware Abstraction Layer for UART functionality on RP2350
 * @details This HAL provides a simplified interface for UART operations on the Raspberry Pi Pico 2 W (RP2350) microcontroller.
 */

 #ifndef UART_HAL_H
 #define UART_HAL_H
 
 #include "pico/stdlib.h"
 #include "hardware/uart.h"
 #include "hardware/irq.h"
 #include <stdbool.h>
 #include <stdint.h>
 
 /** Maximum buffer size for UART operations */
 #define UART_MAX_BUFFER_SIZE 256
 
 /** UART configuration structure */
 typedef struct 
 {
    uart_inst_t* uart_id;     /**< UART instance (uart0 or uart1) */
    uint baud_rate;           /**< Baud rate in bps */
    uint data_bits;           /**< Data bits (5-8) */
    uint stop_bits;           /**< Stop bits (1-2) */
    uart_parity_t parity;     /**< Parity setting */
    uint tx_pin;              /**< TX pin number */
    uint rx_pin;              /**< RX pin number */
 } UART_Config_t;
 
 /** UART status/error codes */
 typedef enum 
 {
    UART_OK = 0,                /**< Operation successful */
    UART_ERROR_INVALID_PARAMS,  /**< Invalid parameters provided */
    UART_ERROR_NOT_INITIALIZED, /**< UART not initialized */
    UART_ERROR_BUSY,            /**< UART busy with operation */
    UART_ERROR_TIMEOUT,         /**< Operation timed out */
    UART_ERROR_BUFFER_FULL,     /**< Buffer full error */
    UART_ERROR_BUFFER_EMPTY,    /**< Buffer empty error */
    UART_ERROR_INVALID_BAUDRATE /**< Invalid baud rate error */
 } UART_Status_t;
 
 /** Callback function type for UART events */
 typedef void (*UART_CallbackFn_t)(void);
 

/**
 * @brief Initializes the UART peripheral.
 *
 * Configures the UART with the specified parameters, sets the correct GPIO functions for TX and RX,
 * and enables the FIFO for buffered (polling) operations.
 *
 * @param config Pointer to the UART configuration structure.
 * @return UART_Status_t UART_OK on success or an error code on failure.
 */
UART_Status_t UART_Init(const UART_Config_t* config);

/**
 * @brief Deinitializes the UART peripheral.
 *
 * Disables the UART and resets the internal state.
 *
 * @param uart_id Pointer to the UART instance (uart0 or uart1).
 * @return UART_Status_t UART_OK on success or an error code if not initialized.
 */
UART_Status_t UART_Deinit(uart_inst_t* uart_id);

/**
 * @brief Transmits data over UART.
 *
 * Sends the specified data and waits until every byte has been transmitted or the timeout occurs.
 *
 * @param uart_id Pointer to the UART instance.
 * @param data Pointer to the data buffer to transmit.
 * @param length Number of bytes to transmit.
 * @param timeout_ms Timeout period in milliseconds.
 * @return UART_Status_t UART_OK on success or an error code if the operation times out or parameters are invalid.
 */
UART_Status_t UART_Send(uart_inst_t* uart_id, const uint8_t* data, size_t length, uint32_t timeout_ms);

/**
 * @brief Receives data over UART.
 *
 * Reads the specified number of bytes from the UART interface, waiting until data is available
 * or the timeout period expires.
 *
 * @param uart_id Pointer to the UART instance.
 * @param data Buffer to store the received data.
 * @param length Number of bytes to receive.
 * @param timeout_ms Timeout period in milliseconds.
 * @return UART_Status_t UART_OK on success or an error code if the operation times out or parameters are invalid.
 */
UART_Status_t UART_Receive(uart_inst_t* uart_id, uint8_t* data, size_t length, uint32_t timeout_ms);

/**
 * @brief Registers a callback for UART receive events.
 *
 * Configures the UART to operate in interrupt mode by disabling FIFO buffering (for per-character handling),
 * clearing any pending data, and installing an exclusive IRQ handler. The provided callback is invoked
 * whenever a received byte triggers an interrupt.
 *
 * @param uart_id Pointer to the UART instance.
 * @param callback Function pointer to the callback. If NULL, any existing callback is cleared.
 * @return UART_Status_t UART_OK on success or an error code if the UART is not initialized.
 */
UART_Status_t UART_RegisterRxCallback(uart_inst_t* uart_id, UART_CallbackFn_t callback);

/**
 * @brief Checks if the UART transmit buffer is ready for sending a new byte.
 *
 * @param uart_id Pointer to the UART instance.
 * @return true if the UART transmit buffer is ready; otherwise false.
 */
bool UART_IsTxReady(uart_inst_t* uart_id);

/**
 * @brief Determines if data is available in the UART receive buffer.
 *
 * @param uart_id Pointer to the UART instance.
 * @return true if data is available for reading; otherwise false.
 */
bool UART_IsRxAvailable(uart_inst_t* uart_id);

/**
 * @brief Blocks until the UART transmit buffer is completely flushed.
 *
 * Waits until all data has been transmitted.
 *
 * @param uart_id Pointer to the UART instance.
 * @return UART_Status_t UART_OK on success or an error code if the UART is not initialized.
 */
UART_Status_t UART_FlushTx(uart_inst_t* uart_id);

/**
 * @brief Clears the UART receive buffer.
 *
 * Drains all unread data in the receive FIFO.
 *
 * @param uart_id Pointer to the UART instance.
 * @return UART_Status_t UART_OK on success or an error code if the UART is not initialized.
 */
UART_Status_t UART_FlushRx(uart_inst_t* uart_id);

/**
 * @brief Disables the UART receive interrupt.
 *
 * Disables the RX interrupt at both the NVIC and the UART peripheral level, clears the registered callback,
 * and resets the internal interrupt flag.
 *
 * @param uart_id Pointer to the UART instance.
 * @return UART_Status_t UART_OK on success or UART_ERROR_NOT_INITIALIZED if the UART is not initialized.
 */
UART_Status_t UART_DisableRxInterrupt(uart_inst_t* uart_id);


#if 0 /* Test Code - you can use this to test the UART HAL or as a reference to get an idea how to use it */

#include <stdio.h>
#include "pico/stdlib.h"
#include <string.h>
#include "UART_HAL.h"

// Test configuration
#define TEST_UART uart1  // Changed to UART1
#define TEST_BAUD_RATE 115200
// UART1 default pins for Pico W are:
#define TEST_TX_PIN 4    // Changed to GP4
#define TEST_RX_PIN 5    // Changed to GP5
#define BUFFER_SIZE 128

// Test status tracking
static volatile bool rx_callback_triggered = false;
static uint32_t test_count = 0;
static uint32_t tests_passed = 0;

// Buffer for loopback testing
static uint8_t rx_buffer[BUFFER_SIZE];
static const char test_message[] = "UART HAL Test Message 123!";

/**
 * @brief Print test result
 */
void print_test_result(const char* test_name, bool passed) {
    test_count++;
    if (passed) {
        tests_passed++;
        printf("✓ Test %lu: %s - PASSED\n", test_count, test_name);
    } else {
        printf("✗ Test %lu: %s - FAILED\n", test_count, test_name);
    }
}

/**
 * @brief RX callback for interrupt test
 */
void uart_rx_callback_kek(void) {
    static uint32_t last_time = 0;
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    
    printf("IRQ at %lu ms (delta: %lu ms)\n", 
           current_time, 
           current_time - last_time);
    last_time = current_time;
    
    rx_callback_triggered = true;
}

/**
 * @brief Test UART initialization
 */
void test_uart_init(void) {
    UART_Config_t config = {
        .uart_id = TEST_UART,
        .baud_rate = TEST_BAUD_RATE,
        .data_bits = 8,
        .stop_bits = 1,
        .parity = UART_PARITY_NONE,
        .tx_pin = TEST_TX_PIN,
        .rx_pin = TEST_RX_PIN
    };

    UART_Status_t status = UART_Init(&config);
    print_test_result("UART Initialization", status == UART_OK);
}

/**
 * @brief Test UART transmit functionality
 */
void test_uart_transmit(void) {
    UART_Status_t status = UART_Send(TEST_UART, (uint8_t*)test_message, strlen(test_message), 1000);
    print_test_result("UART Transmit", status == UART_OK);
    
    // Wait for transmission to complete
    sleep_ms(100);
}

/**
 * @brief Test UART receive functionality with detailed debugging
 */
void test_uart_receive(void) {
    memset(rx_buffer, 0, BUFFER_SIZE);
    printf("\nStarting detailed receive test...\n");
    
    // First clear any pending data
    UART_FlushRx(TEST_UART);
    sleep_ms(10);  // Give time for flush to complete
    
    // Send the test message
    printf("Sending test message: '%s'\n", test_message);
    printf("Message length: %d bytes\n", strlen(test_message));
    
    UART_Status_t tx_status = UART_Send(TEST_UART, (uint8_t*)test_message, strlen(test_message), 1000);
    printf("TX Status: %d\n", tx_status);
    
    // Small delay to ensure transmission is complete
    sleep_ms(50);
    
    // Check if data is available
    bool data_available = UART_IsRxAvailable(TEST_UART);
    printf("RX data available before receive: %s\n", data_available ? "yes" : "no");
    
    // Receive data
    UART_Status_t rx_status = UART_Receive(TEST_UART, rx_buffer, strlen(test_message), 1000);
    printf("Receive status: %d\n", rx_status);
    
    // Print both buffers for comparison
    printf("\nExpected data: ");
    for(size_t i = 0; i < strlen(test_message); i++) {
        printf("%02X ", test_message[i]);
    }
    printf("\nReceived data: ");
    for(size_t i = 0; i < strlen(test_message); i++) {
        printf("%02X ", rx_buffer[i]);
    }
    printf("\n\nReceived as text: '%s'\n", rx_buffer);
    
    bool data_matches = (memcmp(rx_buffer, test_message, strlen(test_message)) == 0);
    printf("Data match: %s\n", data_matches ? "yes" : "no");
    
    if (!data_matches) {
        printf("Mismatched positions: ");
        for(size_t i = 0; i < strlen(test_message); i++) {
            if (test_message[i] != rx_buffer[i]) {
                printf("%zu ", i);
            }
        }
        printf("\n");
    }
    
    print_test_result("UART Receive", data_matches);
}

/**
 * @brief Test UART interrupt functionality with detailed debugging
 */
void test_uart_interrupt(void) {
    printf("\nStarting detailed interrupt test...\n");
    
    // Clear any existing data and state
    rx_callback_triggered = false;
    UART_FlushRx(TEST_UART);
    
    // Ensure RX buffer is empty
    while(uart_is_readable(TEST_UART)) {
        uart_getc(TEST_UART);
    }
    sleep_ms(10);
    
    // Register callback
    printf("Registering callback...\n");
    UART_Status_t status = UART_RegisterRxCallback(TEST_UART, uart_rx_callback_kek);
    print_test_result("UART Callback Registration", status == UART_OK);
    
    // Add small delay
    sleep_ms(10);
    
    // Send test byte
    uint8_t test_byte[12] = "abcdefghijk";
    printf("Sending test bytes: %.12s\n", test_byte); // Print first 12 bytes
    status = UART_Send(TEST_UART, test_byte, 12, 1000);
    printf("Send status: %d\n", status);
    
    // Add delay after sending
    sleep_ms(5);
    
    // Wait for interrupt with timeout
    uint32_t timeout = 1000;
    printf("Waiting for interrupt trigger...\n");
    
    while (!rx_callback_triggered && timeout > 0) {
        sleep_ms(1);
        timeout--;
    }
    
    print_test_result("UART Interrupt Triggered", rx_callback_triggered);
    
    // Cleanup
    UART_RegisterRxCallback(TEST_UART, NULL);
    UART_DisableRxInterrupt(TEST_UART);
}

/**
 * @brief Test UART status functions
 */
void test_uart_status(void) {
    // Test TX ready
    bool tx_ready = UART_IsTxReady(TEST_UART);
    print_test_result("UART TX Ready Check", tx_ready);
    
    // Send data and test RX available
    UART_Send(TEST_UART, (uint8_t*)"Test", 4, 100);
    sleep_ms(10);
    bool rx_available = UART_IsRxAvailable(TEST_UART);
    print_test_result("UART RX Available Check", rx_available);
}

/**
 * @brief Test UART buffer operations
 */
void test_uart_buffer_ops(void) {
    // Test TX buffer flush
    UART_Status_t status = UART_FlushTx(TEST_UART);
    print_test_result("UART TX Buffer Flush", status == UART_OK);
    
    // Test RX buffer flush
    status = UART_FlushRx(TEST_UART);
    print_test_result("UART RX Buffer Flush", status == UART_OK);
}

/**
 * @brief Test error conditions
 */
void test_uart_errors(void) {
    // Test null pointer handling
    UART_Status_t status = UART_Send(TEST_UART, NULL, 10, 100);
    print_test_result("UART Null Pointer Handling", status == UART_ERROR_INVALID_PARAMS);
    
    // Test timeout handling
    uint8_t dummy_buffer[256];
    status = UART_Receive(TEST_UART, dummy_buffer, 256, 1); // 1ms timeout
    print_test_result("UART Timeout Handling", status == UART_ERROR_TIMEOUT);
}

/**
 * @brief Main test sequence
 */
int main(void) {
    // Initialize stdio for printf (using UART0)
    stdio_init_all();
    sleep_ms(2000);  // Wait for serial terminal connection
    
    printf("\nUART HAL Test Suite (Using UART1)\n");
    printf("================================\n\n");
    printf("Please connect GP4 (TX) to GP5 (RX) for loopback testing\n\n");
    
    /* To run the tests, you'll need to connect your Pico's UART1 TX (GP4) to RX (GP5) for loopback testing */
    test_uart_init();
    test_uart_transmit();
    test_uart_receive();
    test_uart_interrupt();
    test_uart_status();
    test_uart_buffer_ops();
    test_uart_errors();
    
    // Print test summary
    printf("\nTest Summary\n");
    printf("============\n");
    printf("Tests passed: %lu/%lu (%.1f%%)\n", tests_passed, test_count, (float)tests_passed / test_count * 100);
    
    // Cleanup
    UART_Deinit(TEST_UART);

    while(1) {
        tight_loop_contents();
    }
    
    return 0;
}

#endif /* Test Code */

#endif /* UART_HAL_H */