/**
 * File: PIO_HAL.c
 * Description: High-level HAL for easier use of PIO on Raspberry Pico 2 W
 */

/**
 * PIO Hardware Abstraction Layer
 * 
 * Documentation references:
 * - PIO APIs Reference: SDK Documentation - Hardware PIO: https://www.raspberrypi.com/documentation/pico-sdk/hardware.html#group_hardware_pio
 * - PIO Theory: RP2350 datasheet, PIO chapter: https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf
 * 
 * Overview:
 * A programmable input/output block (PIO) is a versatile hardware interface supporting various IO standards.
 * - RP2040 has two PIO blocks
 * - RP2350 has three PIO blocks (our Raspberry Pico 2 W board uses RP2350)
 * 
 * PIO programs (.pio) can be sourced in two ways:
 * - From pico-sdk examples for common interfaces (UART, SPI, I2C etc.)
 * - Written directly in PIO assembly for custom implementations
 *
 * Simply check first if the program you need is already available in pico-sdk examples, and if not you get go ahead and write your own.
 * While using existing programs is convenient, custom PIO programming offers greater flexibility for implementing specialized interfaces not covered 
 * by standard implementations.
 *
 * PIO Instruction Set (total of 9 instructions):
 * - JMP  : Jump to address         (set PC to address if condition is true)
 * - WAIT : Wait for condition      (stall until condition is true)
 * - IN   : Input data              (shift 'bit count' bits in from 'Source' into Input Shift Register (ISR))
 * - OUT  : Output data             (shift 'bit count' bits out from Output Shift Register (OSR) to 'Destination')
 * - PUSH : Push data to FIFO       (push the contents of ISR into the RX FIFO as a single 32-bit word. Clears ISR)
 * - PULL : Pull data from FIFO     (load a 32-bit word from the TX FIFO into OSR)
 * - MOV  : Move data               (Copy data from Source to Destination. These can be pins, pindir, scratch registers, PC, ISR and more)
 * - IRQ  : Handle interrupts       (Set or clear the IRQ flag selected by Index argument)
 * - SET  : Set pins/values         (Write immediate value Data to Destination which can be pins, pindir, or scratch registers)
 * 
 * Note - How can FIFOs push/pull whole 32-bit word to Shift Register? 
 *        Think of it like having a 32-bit parallel bus between FIFOs and Shift Registers, 
 *        but for a serial (or smaller parallel) connection between Shift Registers and GPIO pins.
 *
 * For detailed instruction specifications, refer to the Instruction Set subchapter
 * in the RP2350 datasheet.
 * 
 * Each PIO contains four state machines that independently execute sequential programs to manipulate GPIOs
 * and transfer data. These are optimized for deterministic timing and IO operations.
 * 
 * State Machine Features:
 * - Two 32-bit bidirectional shift registers with configurable shift count
 * - Two 32-bit scratch registers
 * - 4x32 bit (4 words) bus FIFO in each direction (TX/RX), reconfigurable as 8x32 in single direction
 * - Fractional clock divider (16 integer, 8 fractional bits)
 * - Flexible GPIO mapping
 * - DMA interface with sustained throughput up to 1 word per clock
 * - IRQ flag management (set/clear/status)
 */

/*******************************************************************************/
/*                                 INCLUDES                                    */
/*******************************************************************************/
#include "PIO_HAL.h"

/*******************************************************************************/
/*                             STATIC VARIABLES                                */
/*******************************************************************************/
PIO_HAL_Status_t currentPIOStatus = PIO_HAL_SM_UNINITIALIZED;

/*******************************************************************************/
/*                        GLOBAL FUNCTION DEFINITIONS                          */
/*******************************************************************************/

PIO_HAL_Status_t PIO_Init(PIO_HAL_Config_t *PIO_config)
{
    currentPIOStatus = PIO_HAL_SM_UNINITIALIZED;

    /* Error checking */
    if (PIO_config == NULL) return PIO_HAL_ERROR_INVALID_PARAM;

    if (PIO_config->program == NULL || PIO_config->program_length == 0) return PIO_HAL_ERROR_INVALID_PARAM;

    if (PIO_config->pio_instance == NULL) return PIO_HAL_ERROR_INVALID_PARAM;

    if (PIO_config->state_machine_num >= 4) return PIO_HAL_ERROR_INVALID_PARAM;

    if (PIO_config->pin_count == 0) return PIO_HAL_ERROR_INVALID_PARAM;

    if (PIO_config->pin_base > 25) return PIO_HAL_ERROR_INVALID_PARAM; //On Raspberry Pico 2 W board GPIO0-GPIO25 can be used for PIO, ADC pins cannot be used for PIO apparently

    if (PIO_config->clock_div < 1.0f || PIO_config->clock_div > 65536.0f) return PIO_HAL_ERROR_INVALID_PARAM; //With RP2350 150MHz system clock, min freq = 150MHz/65536 = 2.2888kHz, max freq = 150MHz/1 = 150MHz

    if (PIO_config->pin_type >= PIO_INVALID_PIN) return PIO_HAL_ERROR_INVALID_PARAM;

    /* Attempt to load the program into the PIO instance */
    if(pio_can_add_program(PIO_config->pio_instance, PIO_config->program))
    {
        int offset_or_error = pio_add_program(PIO_config->pio_instance, PIO_config->program);
        if (offset_or_error < 0) // if value is negative, it is an error
        {
            return PIO_HAL_ERROR_ADD_PROGRAM_FAILED;
        }

        PIO_config->program_offset = (uint)offset_or_error; // if value is positive, it is the offset where the program was loaded
    }
    else
    {
        return PIO_HAL_ERROR_PROGRAM_TOO_LARGE;
    }

    /**
     * Get the default PIO state machine configuration for the PIO program
     * 
     * This function creates a default state machine config and sets up the wrap
     * boundaries for the program loop. The wrap points are specified relative 
     * to the program's loaded position (offset) in instruction memory.
     * 
     * In PIO programs, wrap defines where the program counter loops back to after
     * reaching the end, creating an infinite execution loop of instructions.
     */
    pio_sm_config sm_config = PIO_config->default_config(PIO_config->program_offset);

    /* Setup the function select for a GPIO to use output from the given PIO instance */
    pio_gpio_init(PIO_config->pio_instance, PIO_config->pin_base);
    /* Use a state machine to set the same pin direction for multiple consecutive pins for the PIO instance */
    pio_sm_set_consecutive_pindirs(PIO_config->pio_instance, PIO_config->state_machine_num, PIO_config->pin_base, PIO_config->pin_count, PIO_config->set_pins_as_output);

    if(PIO_config->pin_type == PIO_PIN_SIDESET)
    {
        /* Set the 'sideset' pins in a state machine configuration */
        sm_config_set_sideset_pins(&sm_config, PIO_config->pin_base);
    }
    else if(PIO_config->pin_type == PIO_PIN_SET)
    {
        /* Set the 'set' pins in a state machine configuration */
        sm_config_set_set_pins(&sm_config, PIO_config->pin_base, PIO_config->pin_count);
    }
    else if(PIO_config->pin_type == PIO_PIN_OUT)
    {
        /* Set the 'out' pins in a state machine configuration */
        sm_config_set_out_pins(&sm_config, PIO_config->pin_base, PIO_config->pin_count);
    }
    else /* PIO_PIN_IN */
    {
        /* Set the 'in' pins in a state machine configuration */
        sm_config_set_in_pins(&sm_config, PIO_config->pin_base);
    }

    /* Reset and configure the state machine to a consistent state:
     * - Disables the state machine
     * - Clears the FIFOs
     * - Applies the specified configuration
     * - Resets internal state (shift counters, etc)
     * - Jumps to initial program location
     *
     * Note: State machine remains disabled after this call */
    pio_sm_init(PIO_config->pio_instance, PIO_config->state_machine_num, PIO_config->program_offset, &sm_config);

    /* Set the current clock divider for a state machine */
    pio_sm_set_clkdiv(PIO_config->pio_instance, PIO_config->state_machine_num, PIO_config->clock_div);

    currentPIOStatus = PIO_HAL_SM_INITIALIZED;

    return PIO_HAL_OK;
}

PIO_HAL_Status_t PIO_EnableSM(PIO_HAL_Config_t *PIO_config)
{
    if (currentPIOStatus != PIO_HAL_SM_INITIALIZED) return PIO_HAL_SM_UNINITIALIZED;

    /* Enable a PIO state machine (true = enable, false = disable) */
    pio_sm_set_enabled(PIO_config->pio_instance, PIO_config->state_machine_num, true);

    currentPIOStatus = PIO_HAL_SM_RUNNING;

    return PIO_HAL_OK;
}

PIO_HAL_Status_t PIO_DisableSM(PIO_HAL_Config_t *PIO_config)
{
    if (currentPIOStatus != PIO_HAL_SM_RUNNING) return PIO_HAL_SM_DISABLED;

    /* Disable a PIO state machine (true = enable, false = disable) */
    pio_sm_set_enabled(PIO_config->pio_instance, PIO_config->state_machine_num, false);

    currentPIOStatus = PIO_HAL_SM_DISABLED;

    return PIO_HAL_OK;
}

PIO_HAL_Status_t PIO_TX_FIFO_Write(PIO_HAL_Config_t *PIO_config, uint32_t data)
{
    /* Write a word of data to a state machine's TX FIFO, blocking if the FIFO is full */
    pio_sm_put_blocking(PIO_config->pio_instance, PIO_config->state_machine_num, data);

    return PIO_HAL_OK;
}

PIO_HAL_Status_t PIO_ExecuteInstruction(PIO_HAL_Config_t *PIO_config, uint encoded_instruction)
{
    /* Interrupt the program to execute a single PIO instruction (once) 
            Normal execution:   Instr1 -> Instr2 -> Instr3 -> Instr4
            With pio_sm_exec:   Instr1 -> [Injected] -> Instr2 -> Instr3 -> Instr4 */
    /* See pio_instructions.h in pico-sdk for functions that can be used to encode an instruction (pio_encode_pull for example) */
    pio_sm_exec(PIO_config->pio_instance, PIO_config->state_machine_num, encoded_instruction);

    return PIO_HAL_OK;
}

