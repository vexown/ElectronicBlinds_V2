/**
 * File: SPI_HAL.c
 * Description: High-level HAL for easier use of SPI on Raspberry Pico W
 */

/*******************************************************************************/
/*                                 INCLUDES                                    */
/*******************************************************************************/

#include "SPI_HAL.h"

#define PIN_UNUSED 255

// Helper function to get spi instance
static spi_inst_t* get_spi_inst(SPI_Instance instance) {
    return (instance == SPI_INSTANCE_0) ? spi0 : spi1;
}

// Store CS pins for each instance
static uint8_t cs_pins[2] = {PIN_UNUSED, PIN_UNUSED};

// Validate SPI pins
static bool is_valid_spi_pins(SPI_Instance instance, uint8_t sck, uint8_t mosi, uint8_t miso) {
    if (instance == SPI_INSTANCE_0) {
        // SPI0 valid pins: 
        // SCK: 2, 6, 18
        // MOSI: 3, 7, 19
        // MISO: 0, 4, 16
        const uint8_t valid_sck[] = {2, 6, 18};
        const uint8_t valid_mosi[] = {3, 7, 19};
        const uint8_t valid_miso[] = {0, 4, 16};
        
        bool sck_valid = false, mosi_valid = false, miso_valid = false;
        
        for (int i = 0; i < 3; i++) {
            if (sck == valid_sck[i]) sck_valid = true;
            if (mosi == valid_mosi[i]) mosi_valid = true;
            if (miso == valid_miso[i]) miso_valid = true;
        }
        
        return sck_valid && mosi_valid && miso_valid;
    } else {
        // SPI1 valid pins:
        // SCK: 10, 14, 26
        // MOSI: 11, 15, 27
        // MISO: 8, 12, 24
        const uint8_t valid_sck[] = {10, 14, 26};
        const uint8_t valid_mosi[] = {11, 15, 27};
        const uint8_t valid_miso[] = {8, 12, 24};
        
        bool sck_valid = false, mosi_valid = false, miso_valid = false;
        
        for (int i = 0; i < 3; i++) {
            if (sck == valid_sck[i]) sck_valid = true;
            if (mosi == valid_mosi[i]) mosi_valid = true;
            if (miso == valid_miso[i]) miso_valid = true;
        }
        
        return sck_valid && mosi_valid && miso_valid;
    }
}

SPI_Status SPI_Init(const SPI_Config* config) {
    if (config == NULL) {
        return SPI_ERROR_NULL_POINTER;
    }

    if (config->instance != SPI_INSTANCE_0 && config->instance != SPI_INSTANCE_1) {
        return SPI_ERROR_INVALID_INSTANCE;
    }

    if (!is_valid_spi_pins(config->instance, config->sck_pin, 
                          config->mosi_pin, config->miso_pin)) {
        return SPI_ERROR_INVALID_PINS;
    }

    spi_inst_t* spi = get_spi_inst(config->instance);

    // Initialize SPI peripheral
    spi_init(spi, config->speed_hz);
    
    // Configure SPI mode
    bool cpol = config->mode == SPI_MODE_2 || config->mode == SPI_MODE_3;
    bool cpha = config->mode == SPI_MODE_1 || config->mode == SPI_MODE_3;
    spi_set_format(spi, config->data_bits, cpol, cpha, config->msb_first);
    
    // Set up GPIO pins for SPI
    gpio_set_function(config->sck_pin, GPIO_FUNC_SPI);
    gpio_set_function(config->mosi_pin, GPIO_FUNC_SPI);
    gpio_set_function(config->miso_pin, GPIO_FUNC_SPI);
    
    // Configure CS pin if specified
    if (config->cs_pin != PIN_UNUSED) {
        gpio_init(config->cs_pin);
        gpio_set_dir(config->cs_pin, GPIO_OUT);
        gpio_put(config->cs_pin, 1);  // Deselect by default
        cs_pins[config->instance] = config->cs_pin;
    }

    return SPI_OK;
}

SPI_Status SPI_TransferByte(SPI_Instance instance, uint8_t tx_data, uint8_t* rx_data) {
    if (rx_data == NULL) {
        return SPI_ERROR_NULL_POINTER;
    }

    spi_inst_t* spi = get_spi_inst(instance);
    
    spi_write_read_blocking(spi, &tx_data, rx_data, 1);
    
    return SPI_OK;
}

SPI_Status SPI_WriteByte(SPI_Instance instance, uint8_t data) {
    spi_inst_t* spi = get_spi_inst(instance);
    spi_write_blocking(spi, &data, 1);
    return SPI_OK;
}

SPI_Status SPI_ReadByte(SPI_Instance instance, uint8_t* data) {
    if (data == NULL) {
        return SPI_ERROR_NULL_POINTER;
    }

    spi_inst_t* spi = get_spi_inst(instance);
    spi_read_blocking(spi, 0xFF, data, 1);  // Send dummy byte (0xFF) while reading
    return SPI_OK;
}

SPI_Status SPI_WriteMultiple(SPI_Instance instance, const uint8_t* tx_data, size_t length) {
    if (tx_data == NULL) {
        return SPI_ERROR_NULL_POINTER;
    }

    spi_inst_t* spi = get_spi_inst(instance);
    spi_write_blocking(spi, tx_data, length);
    return SPI_OK;
}

SPI_Status SPI_ReadMultiple(SPI_Instance instance, uint8_t* rx_data, size_t length) {
    if (rx_data == NULL) {
        return SPI_ERROR_NULL_POINTER;
    }

    // Allocate buffer for dummy bytes using FreeRTOS heap
    uint8_t* dummy_tx = (uint8_t*)pvPortMalloc(length);
    if (dummy_tx == NULL) {
        return SPI_ERROR_MEMORY_ALLOCATION;
    }

    // Fill with 0xFF (dummy bytes)
    memset(dummy_tx, 0xFF, length);

    spi_inst_t* spi = get_spi_inst(instance);
    spi_write_read_blocking(spi, dummy_tx, rx_data, length);

    vPortFree(dummy_tx);
    return SPI_OK;
}

SPI_Status SPI_TransferMultiple(SPI_Instance instance, const uint8_t* tx_data, 
                               uint8_t* rx_data, size_t length) {
    if (tx_data == NULL || rx_data == NULL) {
        return SPI_ERROR_NULL_POINTER;
    }

    spi_inst_t* spi = get_spi_inst(instance);
    spi_write_read_blocking(spi, tx_data, rx_data, length);
    return SPI_OK;
}

SPI_Status SPI_ChipSelect(SPI_Instance instance, bool select) {
    if (cs_pins[instance] == PIN_UNUSED) {
        return SPI_ERROR_INVALID_PINS;
    }
    
    gpio_put(cs_pins[instance], !select);  // Active low
    return SPI_OK;
}

SPI_Status SPI_DeInit(SPI_Instance instance) {
    spi_inst_t* spi = get_spi_inst(instance);
    spi_deinit(spi);
    return SPI_OK;
}


