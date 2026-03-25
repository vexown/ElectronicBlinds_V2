#!/bin/bash

echo "Flashing bootloader and metadata using Picoprobe and OpenOCD..."

# Set the path to the OpenOCD installation directory
OPENOCD_PATH="../Dependencies/openocd"

# Flash both components in a single OpenOCD session
echo "Flashing bootloader and metadata..."
$OPENOCD_PATH/src/openocd \
    -s "$OPENOCD_PATH/tcl" \
    -f "$OPENOCD_PATH/tcl/interface/cmsis-dap.cfg" \
    -f "$OPENOCD_PATH/tcl/target/rp2350.cfg" \
    -c "adapter speed 5000" \
    -c "init" \
    -c "halt" \
    -c "program output/bootloader.elf verify" \
    -c "program output/metadata.bin 0x1003F000 verify" \
    -c "reset" \
    -c "shutdown"

if [ $? -ne 0 ]; then
    echo "Error: Flashing failed"
    exit 1
fi

echo "All components flashed successfully!"
echo "Press any key to exit the script."
read -n 1 -s
echo "Exiting the script."