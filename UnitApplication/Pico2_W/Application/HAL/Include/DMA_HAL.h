/**
 * @file DMA_HAL.h
 * @brief Header file for high-level DMA driver for RP2350
 * @details Provides interface declarations for simplified DMA operations
 */

#ifndef DMA_HAL_H
#define DMA_HAL_H

#if 0 /* Test Code - you can use this to test the DMA HAL or as a reference to get an idea how to use it */

#include "DMA_HAL.h"
#include <string.h>

/* Test data */
static const char __attribute__((aligned(4))) dma_test_src[] = "DMA HAL Test Data";
static char __attribute__((aligned(4))) dma_test_dst[32];
static uint8_t test_channel;

/* Test functions */
static void test_basic_transfer(void) {
    memset(dma_test_dst, 0, sizeof(dma_test_dst));

    test_channel = DMA_Init_Basic(dma_test_src, dma_test_dst, sizeof(dma_test_src));
    
    DMA_Start(test_channel);
    DMA_WaitComplete(test_channel);
    
    DMA_Release(test_channel);
}

static void test_paced_transfer(void) {
    memset(dma_test_dst, 0, sizeof(dma_test_dst));
    
    DMA_Config_t config = {
        .dest_addr = dma_test_dst,
        .src_addr = dma_test_src,
        .transfer_count = sizeof(dma_test_src),
        .data_size = DMA_SIZE_8,
        .src_increment = true,
        .dst_increment = true,
        .transfer_req_sig = HAL_DREQ_DMA_TIMER0,
        .treq_timer_rate_hz = 10000, // 10kHz transfer rate
        .enableIRQ0 = true
    };
    
    test_channel = DMA_Init(&config);
    
    DMA_Start(test_channel);
    DMA_WaitComplete(test_channel);
    
    DMA_Release(test_channel);
}

static void test_chained_transfer(void) {
    static char __attribute__((aligned(4))) chain_dst1[16];
    static char __attribute__((aligned(4))) chain_dst2[16];
    memset(chain_dst1, 0, sizeof(chain_dst1));
    memset(chain_dst2, 0, sizeof(chain_dst2));
    
    // Configure second channel
    DMA_Config_t config2 = {
        .dest_addr = chain_dst2,
        .src_addr = dma_test_src + 8,
        .transfer_count = 10,
        .data_size = DMA_SIZE_8,
        .src_increment = true,
        .dst_increment = true,
        .transfer_req_sig = HAL_DREQ_FORCE,
        .enableIRQ0 = true,
        .channelToChainTo = DMA_NO_CHAIN
    };

    int8_t chan2 = DMA_Init(&config2);

    // Configure first channel
    DMA_Config_t config1 = {
        .dest_addr = chain_dst1,
        .src_addr = dma_test_src,
        .transfer_count = 8,
        .data_size = DMA_SIZE_8,
        .src_increment = true,
        .dst_increment = true,
        .transfer_req_sig = HAL_DREQ_FORCE,
        .channelToChainTo = chan2 // Chain to second channel, which will start automatically when first channel completes
    };

    int8_t chan1 = DMA_Init(&config1);
    
    DMA_Start(chan1);  // Start first channel
    DMA_WaitComplete(chan2);  // Wait for second channel
    
    DMA_Release(chan1);
    DMA_Release(chan2);
}

static void test_ring_buffer(void) {
    #define RING_SIZE 16
    #define PATTERN_SIZE 4
    #define PATTERN_SIZE_BITS 2  // 2^2 = 4 bytes
    
    static char __attribute__((aligned(RING_SIZE))) ring_buffer[RING_SIZE];
    static const char __attribute__((aligned(4))) test_pattern[] = "1234";
    
    memset(ring_buffer, 0, RING_SIZE);
    
    DMA_Config_t config = {
        .dest_addr = ring_buffer,
        .src_addr = test_pattern,
        .transfer_count = RING_SIZE,
        .data_size = DMA_SIZE_8,
        .src_increment = true,
        .dst_increment = true,
        .transfer_req_sig = HAL_DREQ_FORCE,
        .enableIRQ0 = true,
        .ring_buffer_write_or_read = false,  // Wrap source address
        .ring_buffer_size_bits = PATTERN_SIZE_BITS
    };
    
    int8_t channel = DMA_Init(&config);
    assert(channel >= 0);
    
    DMA_Start(channel);
    DMA_WaitComplete(channel);
    
    // Verify pattern repeats
    for(int i = 0; i < RING_SIZE; i += PATTERN_SIZE) {
        assert(memcmp(&ring_buffer[i], test_pattern, PATTERN_SIZE) == 0);
    }
    
    DMA_Release(channel);
}

int main(void)
{
    setupHardware();

    /* Step through these tests in debugger to see results because printf is causing issues */
    test_basic_transfer();
    test_paced_transfer();
    test_chained_transfer();
    test_ring_buffer();

    while(1) {
        sleep_ms(1000);
    }
}

#endif

#include "hardware/dma.h"
#include "hardware/sync.h"
#include "pico/time.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief DMA-related macros
 */
#define DMA_MAX_CHANNELS 12     /* Maximum number of DMA channels */
#define DMA_TIMEOUT_MS 1000     /* Timeout for DMA operations in milliseconds */
#define DMA_TIMING_PIN 16       /* GPIO pin for measuring DMA complete transfer time - can be used with a Logic Analyzer */ 
#define DMA_NO_CHAIN 0xFF       /* No chaining of DMA channels */
#define DMA_NOT_PACED 0         /* No pacing of DMA transfers */
#define DMA_RING_DISABLED 0    /* Disable ring buffer mode */
#define DMA_RING_MIN_BITS 1    /* Minimum ring size (2 bytes) */
#define DMA_RING_MAX_BITS 15   /* Maximum ring size (32768 bytes) */

#define DMA_ERROR_RETURN 0xFF   /* Error return value */

/**
 * @brief Reduced version of dreq_num_t enum from pico-sdk
 */
typedef enum hal_dreq_num 
{
    HAL_DREQ_DMA_TIMER0 = 59,  ///< Select DMA_TIMER0 as DREQ
    HAL_DREQ_FORCE = 63        ///< Select FORCE as DREQ
} hal_dreq_num_t;

/**
 * @brief Configuration structure for DMA channel initialization
 */
typedef struct 
{
    void* dest_addr;                 /*!< Destination address for DMA transfer */
    const void* src_addr;            /*!< Source address for DMA transfer */
    uint32_t transfer_count;         /*!< Number of transfers to perform */
    uint8_t data_size;               /*!< Size of each transfer (4 bytes - full word, 2 bytes - half word, 1 byte - byte) */
    bool src_increment;              /*!< Whether to increment source address after each transfer */
    bool dst_increment;              /*!< Whether to increment destination address after each transfer */
    hal_dreq_num_t transfer_req_sig; /*!< Transfer Request (TREQ) Signal to pace the transfer rate. Use HAL_DREQ_FORCE for unpaced transfers */
    uint32_t treq_timer_rate_hz;     /*!< Timer-based TREQ rate in Hz, from 2288Hz to 150MHz (use only with transfer_req_sig set to HAL_DREQ_DMA_TIMER0) */
    bool enableIRQ0;                 /*!< Enable interrupts on completion of the whole set of transfers */
    uint8_t channelToChainTo;        /*!< Channel to chain to. It will be auto-triggered after this channel completes. Use DMA_NO_CHAIN to disable chaining */
    bool ring_buffer_write_or_read;  /*!< True for ring buffer write, false for read */
    uint8_t ring_buffer_size_bits;   /*!< Ring buffer size in bit shifts. Possible ring size: 2-32768 bytes (size_bits 1-15). DMA_RING_DISABLED to disable ring buffer */
} DMA_Config_t;

/**
 * @brief Initialize a DMA channel with specified configuration
 * @param config Pointer to DMA configuration structure
 * @return Channel number if successful, -1 if error
 */
int8_t DMA_Init(DMA_Config_t *config);

/**
 * @brief Start DMA transfer on specified channel
 * @param channel DMA channel number
 * @return true if successful, false if error
 */
bool DMA_Start(uint8_t channel);

/**
 * @brief Wait for DMA transfer to complete
 * @param channel DMA channel number
 * @return true if successful, false if timeout or error
 */
bool DMA_WaitComplete(uint8_t channel);

/**
 * @brief Release a DMA channel
 * @param channel DMA channel number
 */
void DMA_Release(uint8_t channel);

/**
 * @brief Check if DMA transfer is complete
 * @param channel DMA channel number
 * @return true if transfer is complete, false otherwise
 */
bool DMA_IsTransferComplete(uint8_t channel);

/**
 * @brief Abort a DMA transfer
 * @param channel DMA channel number
 * @return true if successful, false if error
 */
bool DMA_Abort(uint8_t channel);

/**
 * @brief Initialize a DMA channel with basic configuration
 * @param source_ptr_u8 Pointer to source data
 * @param destiniation_ptr_u8 Pointer to destination data
 * @param sizeof_data Size of data to transfer
 * @return Channel number if successful, -1 if error
 */
int8_t DMA_Init_Basic(const void* source_ptr_u8, void* destiniation_ptr_u8, uint32_t sizeof_data);

#endif /* DMA_HAL_H */