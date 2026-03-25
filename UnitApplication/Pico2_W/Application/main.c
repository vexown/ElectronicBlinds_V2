/**
 * File: main.c
 * Description: First executed user code after crt0.S assembly startup code defined
 * in the pico-sdk (located in pico-sdk/src/rp2_common/pico_crt0/crt0.S)
 * 
 * The boot process is as follows:
 *  Boot ROM -> boot2 (2nd stage boot code) -> OTA bootloader -> jump to application reset handler -> crt0.S (startup code) -> main()
 * 
 *  Boot ROM:
 *  - Location: Physically inside RP2350 chip
 *  - Size: 32KB (0x00000000-0x00008000)
 *  - Type: Mask ROM (permanent, factory programmed), cannot be modified
 *  - Source: https://github.com/raspberrypi/pico-bootrom-rp2350
 *  - Purpose: Initialize chip, flash boot sequence, USB mass storage boot, debug port selection
 *      > See here for initial entry point for both cores: https://github.com/raspberrypi/pico-bootrom-rp2350/blob/master/src/main/arm/arm8_bootrom_rt0.S
 * 
 *  Second Stage Boot (boot2):
 *  - Size: 256 bytes
 *  - Source: https://github.com/raspberrypi/pico-sdk/tree/master/src/rp2350/boot_stage2 (boot2_w25q080.S by default, see pico2_w.h in pico-sdk)
 *  - Purpose: Responsible for setting up external flash (QSPI, XIP mode) 
 *      > Sets up constants for flash communication (clock dividers, delays, etc.)
 *      > QSPI setup - defines commands for quad SPI mode, configures QSPI pins, set SCLK to 8mA drive, Disable Schmitt trigger on SD0-SD3 pins
 *      > QMI (QSPI Memory Interface) setup - sets timing parameters, configures read command format, sets up quad I/O mode
 *      > Flash initialization - performs dummy transfer, disables command prefix for subsequent reads, sets up continuous read mode
 *      > Finalization - On RP2350 boot stage2 is always called as a regular function, and should return normally
 * 
 * OTA Bootloader:
 * - Size: 256KB
 * - Source: my own code - see UnitApplication/Bootloader
 * - Purpose: Over-the-air update bootloader, responsible for updating the application firmware and jumping to it
 * 
 *  crt0.S:
 *  - Size: 1KB
 *  - Source: https://github.com/raspberrypi/pico-sdk/tree/master/src/rp2_common/pico_crt0
 *  - Purpose: 
 *      > Set up the vector table at address 0x0 (stack pointer, reset handler, interrupt handlers)
 *      > Check which core is executing, if it's core 1 - send it to bootroom to wait, if it's core 0 - continue with initialization
 *      > (For flash-based builds) Execute boot2 code if present
 *      > Copy initialized data from Flash to RAM, copy any RAM-based text sections to RAM if PICO_COPY_TO_RAM is enabled
 *      > Zero out BSS section in RAM (ensures all global variables are initialized to 0)
 *      > Set up C runtime environment - call runtime_init() for C library initialization, call main() function, call exit() function if main() returns
 */

/*******************************************************************************/
/*                                 INCLUDES                                    */
/*******************************************************************************/

/* Library includes. */
#include <stdio.h>
#include "pico/stdlib.h"
#include "OS_manager.h"

/*******************************************************************************/
/*                        STATIC FUNCTION DECLARATIONS                         */
/*******************************************************************************/

/* Hardware setup function */
static void setupHardware( void );

/*******************************************************************************/
/*                          GLOBAL FUNCTION DEFINITIONS                        */
/*******************************************************************************/

/* 
 * Function: main
 * 
 * Description: First user code function, does initial HW setup and calls the OS
 *              start function
 * 
 * Parameters:
 *   - none
 * 
 * Returns: technically 'int' value but the program is a loop we will never return
 */
int main(void)
{
    setupHardware();
    OS_start();
}

/*******************************************************************************/
/*                         STATIC FUNCTION DEFINITIONS                         */
/*******************************************************************************/

/* 
 * Function: main
 * 
 * Description: Configure and initialize Raspberry Pico W hardware 
 * 
 * Parameters:
 *   - none
 * 
 * Returns: void
 */
static void setupHardware(void)
{
    stdio_init_all();
}
