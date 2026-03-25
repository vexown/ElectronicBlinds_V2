#!/bin/bash

# Launch GDB for core1 with the specified ELF file and execute GDB commands from gdb_commands_core1.txt
gdb-multiarch output/moduri_bank_A.elf -x Tools/GDB_and_OpenOCD/gdb_commands_core1.txt
