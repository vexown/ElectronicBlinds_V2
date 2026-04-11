#!/bin/bash

echo "Erasing flash using Picoprobe and OpenOCD..."

# Set the path to the OpenOCD installation directory
OPENOCD_PATH="../Dependencies/openocd"

$OPENOCD_PATH/src/openocd \
    -s "$OPENOCD_PATH/tcl" \
    -f "$OPENOCD_PATH/tcl/interface/cmsis-dap.cfg" \
    -f "$OPENOCD_PATH/tcl/target/rp2350.cfg" \
    -c "adapter speed 5000" \
    -c "init; reset halt; flash erase_sector 0 0 last; reset; exit"

if [ $? -ne 0 ]; then
    echo "Error: Failed to erase flash"
    exit 1
fi

echo "Flash erased successfully!"
echo "Press any key to exit the script."
read -n 1 -s
echo "Exiting the script."
