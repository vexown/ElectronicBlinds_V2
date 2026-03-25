#!/bin/bash

echo "Flashing both banks using Picoprobe and OpenOCD..."

# Set the path to the OpenOCD installation directory
OPENOCD_PATH="../Dependencies/openocd"

# Flash Bank A
echo "Flashing Bank A..."
$OPENOCD_PATH/src/openocd \
    -s "$OPENOCD_PATH/tcl" \
    -f "$OPENOCD_PATH/tcl/interface/cmsis-dap.cfg" \
    -f "$OPENOCD_PATH/tcl/target/rp2350.cfg" \
    -c "adapter speed 5000" \
    -c "program output/moduri_bank_A.elf verify reset exit"

if [ $? -ne 0 ]; then
    echo "Error: Failed to flash Bank A"
    exit 1
fi
echo "Bank A flashed successfully"

# Flash Bank B
echo "Flashing Bank B..."
$OPENOCD_PATH/src/openocd \
    -s "$OPENOCD_PATH/tcl" \
    -f "$OPENOCD_PATH/tcl/interface/cmsis-dap.cfg" \
    -f "$OPENOCD_PATH/tcl/target/rp2350.cfg" \
    -c "adapter speed 5000" \
    -c "program output/moduri_bank_B.elf verify reset exit"

if [ $? -ne 0 ]; then
    echo "Error: Failed to flash Bank B"
    exit 1
fi
echo "Bank B flashed successfully"

echo "Both banks flashed successfully!"
echo "Press any key to exit the script."
read -n 1 -s
echo "Exiting the script."
    
