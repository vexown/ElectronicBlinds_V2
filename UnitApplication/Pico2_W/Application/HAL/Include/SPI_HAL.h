#ifndef SPI_HAL_H
#define SPI_HAL_H

/* Key Features */
/*
    - Support for both SPI instances (SPI0 and SPI1)
    - Pin validation for valid SPI pin combinations
    - All SPI modes supported (0-3)
    - Simple single-byte operations
    - Multiple byte transfer operations
    - Bidirectional transfers
    - Chip select handling
    - Error handling for all operations
    - Uses FreeRTOS memory management
    - Configuration structure for easy initialization
    - Support for different data sizes and bit orders
*/

/* Example use */
/*
void main() {
    // Configure SPI
    SPI_Config config = {
        .instance = SPI_INSTANCE_0,
        .sck_pin = 2,
        .mosi_pin = 3,
        .miso_pin = 0,
        .cs_pin = 1,           // Set to PIN_UNUSED if not using CS
        .speed_hz = 1000000,   // 1 MHz
        .mode = SPI_MODE_0,    // Most common mode
        .data_bits = 8,
        .msb_first = true
    };
    
    // Initialize SPI
    SPI_Status status = SPI_Init(&config);
    if (status != SPI_OK) {
        // Handle error
        return;
    }
    
    // Example: Write and read from a device
    SPI_ChipSelect(SPI_INSTANCE_0, true);  // Select device
    
    // Write command
    uint8_t cmd = 0x42;
    SPI_WriteByte(SPI_INSTANCE_0, cmd);
    
    // Read response
    uint8_t response;
    SPI_ReadByte(SPI_INSTANCE_0, &response);
    
    SPI_ChipSelect(SPI_INSTANCE_0, false);  // Deselect device
}
*/

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "FreeRTOS.h"
#include <string.h>

// SPI instance selection
typedef enum {
    SPI_INSTANCE_0 = 0,
    SPI_INSTANCE_1
} SPI_Instance;

// SPI Mode
typedef enum {
    SPI_MODE_0 = 0,  // CPOL=0, CPHA=0
    SPI_MODE_1,      // CPOL=0, CPHA=1
    SPI_MODE_2,      // CPOL=1, CPHA=0
    SPI_MODE_3       // CPOL=1, CPHA=1
} SPI_Mode;

// Error codes
typedef enum {
    SPI_OK = 0,
    SPI_ERROR_INVALID_INSTANCE,
    SPI_ERROR_INVALID_PINS,
    SPI_ERROR_INIT_FAILED,
    SPI_ERROR_TRANSFER_FAILED,
    SPI_ERROR_NULL_POINTER,
    SPI_ERROR_MEMORY_ALLOCATION
} SPI_Status;

// SPI configuration structure
typedef struct {
    SPI_Instance instance;
    uint8_t sck_pin;
    uint8_t mosi_pin;
    uint8_t miso_pin;
    uint8_t cs_pin;          // Set to PIN_UNUSED if not used
    uint32_t speed_hz;
    SPI_Mode mode;
    uint8_t data_bits;       // Typically 8
    bool msb_first;          // true for MSB first, false for LSB first
} SPI_Config;

// Initialize SPI with given configuration
SPI_Status SPI_Init(const SPI_Config* config);

// Write and read single byte
SPI_Status SPI_TransferByte(SPI_Instance instance, uint8_t tx_data, uint8_t* rx_data);

// Write single byte
SPI_Status SPI_WriteByte(SPI_Instance instance, uint8_t data);

// Read single byte
SPI_Status SPI_ReadByte(SPI_Instance instance, uint8_t* data);

// Write multiple bytes
SPI_Status SPI_WriteMultiple(SPI_Instance instance, const uint8_t* tx_data, size_t length);

// Read multiple bytes
SPI_Status SPI_ReadMultiple(SPI_Instance instance, uint8_t* rx_data, size_t length);

// Bidirectional transfer (simultaneous write and read)
SPI_Status SPI_TransferMultiple(SPI_Instance instance, const uint8_t* tx_data, 
                               uint8_t* rx_data, size_t length);

// Chip select control (if CS pin was specified during init)
SPI_Status SPI_ChipSelect(SPI_Instance instance, bool select);

// De-initialize SPI
SPI_Status SPI_DeInit(SPI_Instance instance);

#endif // SPI_HAL_H