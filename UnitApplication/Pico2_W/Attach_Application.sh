#!/bin/bash

# ****************************************************************************
# Attach_Application.sh
#
# Description:
# Post-mortem counterpart to Debug_Application.sh. Where Debug_Application.sh
# RESETS, reflashes and re-runs the target (great for normal debugging, fatal
# for diagnosing a freeze), this script ATTACHES to a board that is ALREADY
# RUNNING and halts it exactly where it is.
#
# Use this the next time the Pico freezes (no logs, alive LED not flashing):
# do NOT power-cycle it. Just run this script with the debug probe connected.
# It will:
#   1. Start OpenOCD against the RP2350.
#   2. Attach gdb-multiarch WITHOUT resetting or reloading.
#   3. Halt the core and dump backtrace, registers, PRIMASK (interrupt-mask
#      state), CFSR/HFSR and the watchdog scratch breadcrumb.
#
# This tells you whether the firmware deliberately parked itself
# (e.g. CriticalErrorHandler's IRQs-off spin -> PRIMASK set, PC in that loop)
# or whether it is a genuine hardware lockup / brownout.
#
# Note: the on-board watchdog is enabled with pause_on_debug=true, so halting
# under the debugger will not trigger a reset while you inspect.
#
# Usage:
#   ./Attach_Application.sh [path/to/matching.elf]   (defaults to bank A)
# ****************************************************************************

ELF="${1:-output/moduri_bank_A.elf}"

# Function to launch OpenOCD (same pattern as Debug_Application.sh)
launch_openocd() {
    if [[ "$OSTYPE" == "linux-gnu"* ]]; then
        gnome-terminal --title="OpenOCD (attach)" -- bash -c "./Tools/GDB_and_OpenOCD/StartOpenOCD.sh; exec bash"
    elif [[ "$OSTYPE" == "msys"* || "$OSTYPE" == "cygwin"* ]]; then
        wt.exe --title "OpenOCD (attach)" bash -c "./Tools/GDB_and_OpenOCD/StartOpenOCD.sh && exec bash"
    else
        echo "Unsupported OS for OpenOCD launch."
    fi
}

# Function to launch the attach GDB session
launch_gdb_attach() {
    if [[ "$OSTYPE" == "linux-gnu"* ]]; then
        gnome-terminal --title="Core0 GDB (attach)" -- bash -c "./Tools/GDB_and_OpenOCD/StartGDB_Core0_attach.sh '$ELF'; exec bash"
    elif [[ "$OSTYPE" == "msys"* || "$OSTYPE" == "cygwin"* ]]; then
        wt.exe --title "Core0 GDB (attach)" bash -c "./Tools/GDB_and_OpenOCD/StartGDB_Core0_attach.sh '$ELF' && exec bash"
    else
        echo "Unsupported OS for GDB launch."
    fi
}

# Launch OpenOCD and give it a moment to come up
launch_openocd
sleep 1

# Attach (no reset, no load)
launch_gdb_attach
