#!/bin/bash

# Attach gdb-multiarch to a LIVE core0 (no reset, no load) and run the
# post-mortem dump in gdb_attach_core0.txt.
#
# The ELF is only used for symbol resolution, so pass the one matching the
# bank the frozen board is running from. Defaults to bank A.
#   Usage: ./Tools/GDB_and_OpenOCD/StartGDB_Core0_attach.sh [path/to.elf]
ELF="${1:-output/moduri_bank_A.elf}"

gdb-multiarch "$ELF" -x Tools/GDB_and_OpenOCD/gdb_attach_core0.txt
