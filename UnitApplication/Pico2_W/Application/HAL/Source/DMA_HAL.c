/**
 * @file DMA_HAL.c
 * @brief High-level DMA driver for RP2350
 * @details Provides simplified DMA operations with error handling and safety checks
 */

/**
 * The RP-series microcontroller Direct Memory Access (DMA) master performs bulk data transfers on a processor’s
 * behalf. This leaves processors free to attend to other tasks, or enter low-power sleep states. The
 * data throughput of the DMA is also significantly higher than one of RP-series microcontroller’s processors.
 *
 * The DMA can perform one read access and one write access, up to 32 bits in size, every clock cycle.
 * There are 12 independent channels, which each supervise a sequence of bus transfers, usually in
 * one of the following scenarios:
 *      - Memory to peripheral
 *      - Peripheral to memory
 *      - Memory to memory
 * 
 * More info at Chapter 10.7 of RP2350 Datasheet: https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf
 * 
 */

/*******************************************************************************/
/*                                 INCLUDES                                    */
/*******************************************************************************/
#include "DMA_HAL.h"
#include "hardware/gpio.h"
#include "hardware/regs/intctrl.h"

/*******************************************************************************/
/*                             STATIC VARIABLES                                */
/*******************************************************************************/
static bool dma_channels_used[DMA_MAX_CHANNELS] = {false};
static int dma_irq0_channel = -1; // static variable to store the channel number for the IRQ0 handler

/*******************************************************************************/
/*                        STATIC FUNCTION DECLARATIONS                         */
/*******************************************************************************/
static uint claim_dma_channel(void);
static void configure_channel_properties(dma_channel_config *channel_config, const DMA_Config_t *config);
static void dma_irq0_handler(void);
static void dma_timing_gpio_init(void);
static bool calculate_timer_fraction(uint32_t desired_rate, uint16_t *denominator);
static bool setup_dma_timer(const DMA_Config_t *config);
static void setup_dma_interrupts(uint channel);

/*******************************************************************************/
/*                        STATIC FUNCTION DEFINITIONS                          */
/*******************************************************************************/

/**
 * @brief Claim an unused DMA channel
 * @return Channel number if successful, -1 if error
 */
static uint claim_dma_channel(void) 
{
    /* Attempt to claim an unused DMA channel, return -1 if none available */
    int channel = dma_claim_unused_channel(false);
    if (channel < 0) return DMA_ERROR_RETURN;

    /* Mark channel as used */
    dma_channels_used[channel] = true;

    return (uint)channel; // channel number is always positive, otherwise it would have returned DMA_ERROR_RETURN
}

/**
 * @brief Configure the DMA channel properties according to the user's requirements
 * @param channel_config Pointer to DMA channel configuration structure
 * @param config Pointer to DMA configuration structure
 */
static void configure_channel_properties(dma_channel_config *channel_config, const DMA_Config_t *config) 
{
    /* Set the data size for each individual transfer (1, 2 or 4 bytes) */
    channel_config_set_transfer_data_size(channel_config, config->data_size);

    /* Set the increment mode for source and destination addresses - this will determine 
       whether the addresses are incremented by the data size after each transfer */
    channel_config_set_read_increment(channel_config, config->src_increment);
    channel_config_set_write_increment(channel_config, config->dst_increment);

    /* In default config this is set to DREQ_FORCE which essentially means no pacing signal (send data as fast as possible) */
    /* If you want to use such signal, you need to configure the transfer request (TREQ) signal to pace the transfer rate */
    channel_config_set_dreq(channel_config, config->transfer_req_sig);

    if (config->channelToChainTo != DMA_NO_CHAIN && config->channelToChainTo < DMA_MAX_CHANNELS)
    {
        /* Chain to another channel after this channel completes */
        channel_config_set_chain_to(channel_config, config->channelToChainTo);
    }

    if (config->ring_buffer_size_bits != DMA_RING_DISABLED) 
    {
        if (config->ring_buffer_size_bits >= DMA_RING_MIN_BITS && config->ring_buffer_size_bits <= DMA_RING_MAX_BITS) 
        {
            /* Set the ring buffer mode with the specified size */
            /* Size of address wrap region works like this:
                - If 0, don’t wrap (DMA_RING_DISABLED)
                - For values n > 0, only the lower n bits of the address will change. This wraps the address on a (1 << n) byte boundary, 
                  facilitating access to naturally-aligned ring buffers. So in normal words, size of the ring buffer is 2^n bytes */
            channel_config_set_ring(channel_config, config->ring_buffer_write_or_read, config->ring_buffer_size_bits);
        }
    }
}

/**
 * @brief DMA IRQ0 handler which is called when the whole set of transfers is complete
 */
static void dma_irq0_handler(void) 
{
    gpio_put(DMA_TIMING_PIN, 0); // Set the timing pin low as indication of the Transfer End
    
    dma_hw->ints0 = 1u << dma_irq0_channel; // Clear the interrupt request
}

/**
 * @brief Initialize the GPIO pin for measuring DMA transfer time
 */
static void dma_timing_gpio_init(void) 
{
    gpio_init(DMA_TIMING_PIN);
    gpio_set_dir(DMA_TIMING_PIN, GPIO_OUT);
    gpio_put(DMA_TIMING_PIN, 0); // Set the timing pin low initially
}

/**
 * @brief Calculate the timer fraction for the DMA timer
 * @param desired_rate Desired transfer rate in Hz
 * @param denominator Pointer to store the calculated denominator
 * @return true if successful, false if error
 */
static bool calculate_timer_fraction(uint32_t desired_rate, uint16_t *denominator) 
{
    const uint32_t SYSTEM_CLOCK_FREQ = 150000000; // 150MHz
    const uint32_t MIN_TRANSFER_RATE = SYSTEM_CLOCK_FREQ / UINT16_MAX;
    const uint32_t MAX_TRANSFER_RATE = SYSTEM_CLOCK_FREQ;

    if (desired_rate < MIN_TRANSFER_RATE || desired_rate > MAX_TRANSFER_RATE) 
    {
        return false; // Desired rate out of range
    }

    /* Calculate fraction: desired_rate = SYSTEM_CLOCK_FREQ * (num/den)
     * Therefore: num/den = desired_rate/SYSTEM_CLOCK_FREQ
     * (Numinator is assumed to be 1) */
    uint32_t denominator_local = SYSTEM_CLOCK_FREQ / desired_rate;
    if (denominator_local > UINT16_MAX) 
    {
        denominator_local = UINT16_MAX;
    }
    
    *denominator = (uint16_t)denominator_local;
    
    return true;
}

/**
 * @brief Setup the DMA timer for pacing the transfer rate
 * @param config Pointer to DMA configuration structure
 * @return true if successful, false if error
 */
static bool setup_dma_timer(const DMA_Config_t *config) 
{
    /* Attempt to claim DMA timer 0 */
    if (!dma_timer_is_claimed(0))
    {
        dma_timer_claim(0); 
    }
    else
    {
        return false;
    }
    
    /* Set the multiplier for DMA timer 0 */
    /* The timer will run at the system_clock_freq (150MHz on RP2350) * numerator / denominator, so this is the speed
        that data elements will be transferred at via a DMA channel using this timer as a DREQ. The multiplier must be less than or equal to one.
        Minimum transfer rate is 150MHz * 1/65536 =~ 2.29kHz. 
        Maximum transfer rate is 150MHz * 1/1 = 150MHz */
    const uint16_t numerator = 1;
    uint16_t denominator;
    if (!calculate_timer_fraction(config->treq_timer_rate_hz, &denominator)) 
    {
        return false;
    }
    dma_timer_set_fraction(0, numerator, denominator);
    
    return true;
}

/**
 * @brief Setup the DMA interrupts for the given channel
 * @param channel DMA channel number
 */
static void setup_dma_interrupts(uint channel) 
{
    dma_irq0_channel = (int)channel; // store the channel number for the IRQ0 handler
    dma_timing_gpio_init();

    /* Enable DMA interrupts on the given channel using DMA_IRQ_0 line (on RP2350 there is 4 lines dedicated to DMA: DMA_IRQ_0-3) */
    /* The interupt will be triggered when the WHOLE set of transfers is complete NOT after each individual transfer */
    dma_channel_set_irq0_enabled(channel, true);

    /* Set the handler and enable the interrupt */
    irq_set_exclusive_handler(DMA_IRQ_0, dma_irq0_handler);
    irq_set_enabled(DMA_IRQ_0, true);
}

/*******************************************************************************/
/*                        GLOBAL FUNCTION DEFINITIONS                          */
/*******************************************************************************/

int8_t DMA_Init_Basic(const void* source_ptr_u8, void* destiniation_ptr_u8, uint32_t sizeof_data) 
{
    DMA_Config_t config = {
        .dest_addr = destiniation_ptr_u8,
        .src_addr = source_ptr_u8,
        .transfer_count = sizeof_data,
        .data_size = DMA_SIZE_8,
        .src_increment = true,
        .dst_increment = true,
        .transfer_req_sig = HAL_DREQ_FORCE, 
        .treq_timer_rate_hz = DMA_NOT_PACED,
        .enableIRQ0 = false,
        .channelToChainTo = DMA_NO_CHAIN,
        .ring_buffer_write_or_read = false,
        .ring_buffer_size_bits = DMA_RING_DISABLED
    };
    
    return DMA_Init(&config);
}
/**
 * @brief Initialize a DMA channel with specified configuration
 * @param config Pointer to DMA configuration structure
 * @return Channel number if successful, -1 if error
 */
int8_t DMA_Init(DMA_Config_t *config) 
{
    if (config == NULL) return -1;

    uint channel = claim_dma_channel();

    /* Get the default channel configuration for a given channel */
    dma_channel_config channel_config = dma_channel_get_default_config(channel);
    
    /* Now modify the default configuration to match the user's requirements */
    configure_channel_properties(&channel_config, config);

    /* If the transfer request signal is set to use DMA timer, attempt to claim it and set the multiplier */
    if(config->transfer_req_sig == HAL_DREQ_DMA_TIMER0)
    {
        if(!setup_dma_timer(config))
        {
            dma_channel_unclaim(channel);
            dma_channels_used[channel] = false;
            return -1;   
        }
    }

    /* Apply the DMA channel configuration with the specified parameters */
    dma_channel_configure(
        channel,           // DMA channel number
        &channel_config,   // Pointer to the DMA config structure
        config->dest_addr, // Initial write address
        config->src_addr,  // Initial read address
        config->transfer_count,  // Number of transfers to perform
        false              // True to start transfer immediately
    );

    if(config->enableIRQ0)
    {
        setup_dma_interrupts(channel);
    }

    /* Return the channel number - it is our handle for DMA operations */
    return (int8_t)channel; // casting is safe because channel range is 0-11 so it fits in int8_t
}

/**
 * @brief Start DMA transfer on specified channel
 * @param channel DMA channel number
 * @return true if successful, false if error
 */
bool DMA_Start(uint8_t channel) 
{
    if (channel >= DMA_MAX_CHANNELS || !dma_channels_used[channel]) return false;
    
    gpio_put(DMA_TIMING_PIN, 1); // Set the timing pin high as indication of the Transfer Start
    dma_channel_start(channel);
    return true;
}

/**
 * @brief Wait for DMA transfer to complete
 * @param channel DMA channel number
 * @return true if successful, false if timeout or error
 */
bool DMA_WaitComplete(uint8_t channel) 
{
    if (channel >= DMA_MAX_CHANNELS || !dma_channels_used[channel]) return false;

    dma_channel_wait_for_finish_blocking(channel);
    return true;
}

/**
 * @brief Check if DMA transfer is complete
 * @param channel DMA channel number
 * @return true if transfer is complete, false otherwise
 */
bool DMA_IsTransferComplete(uint8_t channel) 
{
    if (channel >= DMA_MAX_CHANNELS || !dma_channels_used[channel]) return false;
    
    return !dma_channel_is_busy(channel);
}

/**
 * @brief Abort a DMA transfer
 * @param channel DMA channel number
 * @return true if successful, false if error
 */
bool DMA_Abort(uint8_t channel)
{
    if (channel >= DMA_MAX_CHANNELS || !dma_channels_used[channel]) return false;
    
    dma_channel_abort(channel);

    return true;
}

/**
 * @brief Release a DMA channel
 * @param channel DMA channel number
 */
void DMA_Release(uint8_t channel) 
{
    if (channel < DMA_MAX_CHANNELS && dma_channels_used[channel]) 
    {
        if(dma_irq0_channel == channel)
        {
            /* Disable the DMA transfer complete interrupts */
            dma_channel_set_irq0_enabled(channel, false);
            irq_set_enabled(DMA_IRQ_0, false);
            irq_remove_handler(DMA_IRQ_0, dma_irq0_handler);

            dma_irq0_channel = -1;
        }

        /* Abort any ongoing transfer */
        dma_channel_abort(channel);

        /* Unclaim the channel and mark it as unused */
        dma_channel_unclaim(channel);
        dma_channels_used[channel] = false;
    }
}