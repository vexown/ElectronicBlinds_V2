#!/bin/bash

# Script for Analyzing ELF and BIN Files for Embedded Systems
#
# This script analyzes an ELF file and its associated binary (BIN) file, generating various reports and outputs
# to aid in the inspection of memory usage, symbols, and memory sections. It segregates symbols based on memory 
# ranges for RAM and FLASH, generates disassembly, a hex dump with adjusted flash addresses, and provides a summary 
# of the binary and ELF file properties.
#
# Usage:
#   ./analyze_binary.sh (in the same directory as your .elf and .bin binaries)
#
# Dependencies:
#   - ARM toolchain (arm-none-eabi)
#   - `xxd` for hex dumping
#
#########################################################################################################################################################
#
# Raspberry Pi Pico 2 W Memory Map:
# (Source - Address Map chapter in https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf)
#
# - ROM: 0x00000000-0x00008000 (32kB)
#        This is ACTUAL ROM - Read Only Memory - The ROM contents are fixed at the time the silicon is manufactured
#        It's not some relict bullshit of calling regular writeable flash/EEPROM as ROM, as manufacturers and people often do.
#        It contains things like the Core 0 boot sequence, Core 1 Launch code, USB/UART bootloader but also some useful runtime APIs,
#        see https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf#page=376 for details
# - SRAM: 0x20000000 - 0x20082000 (520kB)
#        Divided into 10 banks. Banking is a physical partitioning of SRAM which improves performance by allowing multiple simultaneous
#        accesses. Logically, there is a single 520 kB contiguous memory. Each SRAM bank is accessed via a dedicated AHB5 arbiter.
#        This means different bus managers can access different SRAM banks in parallel, so up to six 32-bit SRAM accesses can take place 
#        every system clock cycle (one per manager).
# - Program Flash (XIP): 0x10000000 - 0x10400000 (4MB)
#        External Flash, accessed via QSPI interface using execute-in-place (XIP) hardware. Meaning it can be addressed and accessed by the 
#        system as it were internal memory. A 16 kB on-chip cache retains the values of recent reads and writes. This reduces the chances 
#        that XIP bus accesses must go to external memory, improving the average throughput and latency of the XIP interface.
# - APB Peripheral Registers: Start at 0x40000000
#        APB Peripherals include: SYSCFG, CLOCKS, PLL, UART, SPI, I2C, ADC, PWM, TIMER, WATCHDOG, OTP, SHA256, CORESIGHT etc.
# - AHB-Lite Peripheral Registers: Start at 0x50000000
#        AHB-Lite Peripherals: DMA, USBCTRL, PIO, XIP_AUX, HSTX_FIFO etc.
#        For DMA examples see https://github.com/raspberrypi/pico-examples/tree/master/dma 
# - Core-local Peripherals (SIO): Start at 0xD0000000
#        Used to access SIO - The Single-cycle IO subsystem (SIO) contains peripherals that require low-latency, deterministic access from the
#        processors. It is accessed via the AHB Fabric. The SIO has a dedicated bus interface for each processor.
#        The single-cycle IO block contains registers which processors must access quickly. FIFOs, doorbells and spinlocks support message passing and
#        synchronisation between the two cores. The shared GPIO registers provide fast, direct access to GPIO-capable pins. Interpolators can accelerate 
#        common software tasks.
#        The SIO contains: CPUID registers, Mailbox FIFOs, Doorbells, Hardware spinlocks, Interpolators, RISC-V platform timer, GPIO regs for bitbanging
# - Cortex-M33 Private Peripherals: Start at 0xE0000000 
#        The Private Peripheral Bus (PPB) memory region provides access to internal and external processor resources.
#        The PPB provides access to: System Control Space (SCS) - including MPU, SAU and NVIC, Data Watchpoint and Trace (DWT), Breakpoint Unit (BPU),
#        Embedded Trace Macrocell (ETM), CoreSight Micro Trace Buffer (MTB), Cross Trigger Interface (CTI), ROM table.
#
#########################################################################################################################################################
# 
# Linker script for Raspberry Pi Pico 2 W (RP2350) can be found at https://github.com/raspberrypi/pico-sdk/tree/master/src/rp2_common/pico_crt0/rp2350
# 
# Summary of the linker script (the default one):
#
# Memory Layout:
#   Defines three main regions:
#     - Flash memory (actual layout included from separate file)
#     - Main RAM (512KB starting at 0x20000000)
#     - Two 4KB scratch regions (X and Y) for core-specific operations
#
# Flash Memory Organization:
#   Starts with critical boot sections in strict order:
#     - Vector table (must be first)
#     - Binary info header (must be within first 1KB)
#     - Boot2 section (limited to 256 bytes)
#   Contains read-only sections:
#     - Program code (.text)
#     - Constants (.rodata)
#     - Binary information for tools
#     - Initial values for initialized variables
#
# RAM Organization:
#   Contains several key regions:
#     - .data (initialized variables, copied from flash)
#     - .bss (zero-initialized variables)
#     - Heap (grows upward from end of .bss)
#     - Vector table copy (for runtime modifications)
#     - Uninitialized data section
#
# Stack Organization:
#   Uses scratch memory regions:
#     - Core 0's stack in SCRATCH_Y
#     - Core 1's stack in SCRATCH_X (when used)
#   Stack grows downward from top of respective regions
#   Stack sizes determined by .stack_dummy sections
#
# Symbols for Runtime:
#   Provides locations needed by startup code:
#     - __StackTop for initial stack pointer
#     - data_start and __etext for data copying
#     - bss_start and bss_end for zeroing BSS

analyze_binary() {
    local BIN_FILE=$1
    local ELF_FILE=$2
    local COMPONENT_NAME=$3
    local OUTPUT_DIR="binary_analysis_${COMPONENT_NAME}"
    
    echo "Analyzing ${COMPONENT_NAME}:"
    echo "ELF: $ELF_FILE"
    echo "BIN: $BIN_FILE"
    echo "Output will be saved in $OUTPUT_DIR/"
    
    # Create output directory
    mkdir -p $OUTPUT_DIR

    # 1. Disassembly
    echo "Generating disassembly from ELF..."
    ${TOOLCHAIN_PREFIX}-objdump -d $ELF_FILE > $OUTPUT_DIR/disassembly.txt

    # 2. Symbol Analysis
    echo "Analyzing symbols from ELF (RAM and FLASH segregation)..."
    echo -e "Address\tSize\tType\tName" > $OUTPUT_DIR/symbols_ram.txt
    echo -e "Address\tSize\tType\tName" > $OUTPUT_DIR/symbols_flash.txt

    # Temporary files
    temp_ram_file=$(mktemp)
    temp_flash_file=$(mktemp)

    while read -r line; do
        address=$(echo $line | awk '{print $1}')
        size=$(echo $line | awk '{print $2}')
        type=$(echo $line | awk '{print $3}')
        name=$(echo $line | awk '{print $4}')

        address_dec=$((16#$address))

        if [[ $address_dec -ge 0x20000000 && $address_dec -lt 0x20042000 ]]; then
            echo -e "$line" >> $temp_ram_file
        elif [[ $address_dec -ge 0x10000000 && $address_dec -lt 0x10200000 ]]; then
            echo -e "$line" >> $temp_flash_file
        else
            echo "Error: Address $address (0x$address_dec) is outside of defined RAM/FLASH ranges" >> $OUTPUT_DIR/symbols_outside_ranges.txt
        fi
    done < <(${TOOLCHAIN_PREFIX}-nm --numeric-sort --size-sort --print-size $ELF_FILE)

    sort -k1,1n $temp_ram_file > $OUTPUT_DIR/symbols_ram.txt
    sort -k1,1n $temp_flash_file > $OUTPUT_DIR/symbols_flash.txt

    rm $temp_ram_file $temp_flash_file

    # Generate top 10 largest symbols
    echo "Generating top 10 largest symbols..."
    echo -e "Address\tSize\tType\tName" > $OUTPUT_DIR/large_symbols.txt
    sort -nrk 2 $OUTPUT_DIR/symbols_ram.txt $OUTPUT_DIR/symbols_flash.txt | head -n 10 >> $OUTPUT_DIR/large_symbols.txt

    # 3. Hex Dump
    echo "Generating hex dump from BIN with flash address adjustment..."
    FLASH_BASE_ADDR=0x10000000
    xxd -g 1 -u $BIN_FILE | awk -v base=$FLASH_BASE_ADDR '
    BEGIN { address = base }
    {
        printf("%08X: ", address)
        address += 16
        for (i=2; i<=NF; i++)
            printf("%s ", $i)
        print ""
    }' > $OUTPUT_DIR/hexdump.txt

    # 4. Memory Sections
    echo "Analyzing memory sections from ELF..."
    echo "Memory Sections Analysis:" > $OUTPUT_DIR/memory_sections.txt
    echo "=======================" >> $OUTPUT_DIR/memory_sections.txt
    echo "" >> $OUTPUT_DIR/memory_sections.txt
    
    # Create formatted section table with only loadable sections
    {
        printf "%-25s %-12s %-12s %-12s\n" "Section" "VMA" "LMA" "Size"
        printf "%-25s %-12s %-12s %-12s\n" "-------" "---" "---" "----"
        ${TOOLCHAIN_PREFIX}-objdump -h $ELF_FILE | grep "^[ ]*[0-9]" | \
        awk '$4 != "00000000" {printf "%-25s 0x%-10s 0x%-10s 0x%-10s\n", $2, $4, $5, $3}'
    } >> $OUTPUT_DIR/memory_sections.txt

    # 5. Generate Summary
    echo "Generating summary report..."
    SUMMARY_FILE="$OUTPUT_DIR/summary.txt"
    {
        echo "${COMPONENT_NAME} Binary Analysis Summary"
        echo "====================================="
        echo ""
        echo "File Overview:"
        file $ELF_FILE
        echo ""
        echo "Memory Usage [bytes] (from ELF):"
        ${TOOLCHAIN_PREFIX}-size $ELF_FILE
        echo ""
        echo "Total Size of Binary [bytes] (from BIN):"
        stat --format=%s $BIN_FILE
        echo ""
        echo "Top 10 Largest Symbols:"
        cat $OUTPUT_DIR/large_symbols.txt
        echo ""
        echo "Memory Sections (from ELF):"
        ${TOOLCHAIN_PREFIX}-objdump -h $ELF_FILE | grep -E 'Idx|\.text|\.data|\.bss|\.rodata'
        echo ""
    } > $SUMMARY_FILE

    echo "Analysis of ${COMPONENT_NAME} complete."
    echo "----------------------------------------"
}

# Main script
TOOLCHAIN_PREFIX=arm-none-eabi

# Check bootloader files
if [ ! -f "bootloader.elf" ] || [ ! -f "bootloader.bin" ]; then
    echo "Warning: Bootloader files not found"
else
    analyze_binary "bootloader.bin" "bootloader.elf" "bootloader"
fi

# Check application files
if [ ! -f "moduri_bank_A.elf" ] || [ ! -f "moduri_bank_A.bin" ]; then
    echo "Warning: Application (moduri) files not found"
else
    analyze_binary "moduri_bank_A.bin" "moduri_bank_A.elf" "moduri"
fi

echo "All analyses complete. Check the binary_analysis_* directories for results."
