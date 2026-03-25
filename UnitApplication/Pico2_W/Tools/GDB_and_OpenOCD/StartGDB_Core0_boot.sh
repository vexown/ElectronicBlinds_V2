#!/bin/bash

# Launch GDB for core0 with the specified ELF file and execute GDB commands from gdb_commands_core0.txt
gdb-multiarch output/bootloader.elf -x Tools/GDB_and_OpenOCD/gdb_commands_core0.txt