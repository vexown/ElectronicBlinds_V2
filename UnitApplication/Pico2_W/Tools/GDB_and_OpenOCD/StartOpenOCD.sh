#!/bin/bash

# ****************************************************************************
# Debug.sh
#
# Description:
# This script starts OpenOCD (Open On-Chip Debugger) with specific configuration files
# and settings for interfacing with a target board (in this case, the RP2350 microcontroller).
# OpenOCD is commonly used for debugging embedded systems.
#
# Usage:
# Called automatically from Debug.sh
#
# ****************************************************************************

# Set the path to the OpenOCD installation directory - we are using the OpenOCD version from RPi repository https://github.com/raspberrypi/openocd
OPENOCD_PATH="../Dependencies/openocd"

$OPENOCD_PATH/src/openocd \
    -s "$OPENOCD_PATH/tcl" \
    -f "$OPENOCD_PATH/tcl/interface/cmsis-dap.cfg" \
    -f "$OPENOCD_PATH/tcl/target/rp2350.cfg" \
    -c "adapter speed 5000"

# Explanation of each part of the command:

# openocd
# This is the main command that launches OpenOCD. OpenOCD provides debugging, in-system programming,
# and boundary-scan testing for embedded devices using various debug adapters.

# -f interface/cmsis-dap.cfg
# This option specifies the configuration file for the debug interface (adapter) being used.
# 'cmsis-dap.cfg' is a configuration file for using CMSIS-DAP, which is a common debug protocol
# supported by many ARM microcontroller debuggers. It configures OpenOCD to use the CMSIS-DAP adapter
# to communicate with the target microcontroller.

# -f target/rp2350.cfg
# This option specifies the configuration file for the target device (the microcontroller being debugged).
# 'rp2350.cfg' is the configuration file for the Raspberry Pi RP2350 microcontroller, specifying how OpenOCD
# should interact with this specific target. It sets up the necessary parameters for OpenOCD to correctly
# control and debug the RP2350.

# -c "adapter speed 5000"
# The '-c' option allows you to pass commands to OpenOCD directly from the command line.
# In this case, "adapter speed 5000" sets the communication speed between the debugger (CMSIS-DAP adapter)
# and the target (RP2350) to 5000 kHz (5 MHz). This is a high speed, often used to ensure fast
# communication during debugging. The speed value may need to be adjusted based on the capabilities of
# the debug adapter or the stability of the connection with the target device.

# The configuration files such as cmsis-dap.cfg and rp2350.cfg are included with the installation of OpenOCD. 
# These files provide pre-configured settings for various debug interfaces (like CMSIS-DAP) and
# target devices (such as the RP2350 microcontroller).