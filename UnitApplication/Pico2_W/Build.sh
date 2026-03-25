#!/bin/bash

# ****************************************************************************
# Build.sh
#
# Description:
# This script automates the process of building a project using CMake.
# It performs the following steps:
# 1. Checks for and installs required system packages if missing
# 2. Ensures project dependencies (FreeRTOS, pico-sdk, picotool) are initialized as git submodules
# 3. Checks if the 'build' directory exists and creates it if not.
# 4. Configures the build directory by running CMake.
# 5. Builds the project using CMake.
# 6. Copies output files to the output directory.
# 7. Prompts the user to press any key to exit the script.
#
# Usage:
# Run this script from the root directory of the project:
#   ./Build.sh                          # Build the default application
#   ./Build.sh --clean                  # Clean and rebuild
#   ./Build.sh --example=list           # List available examples
#   ./Build.sh --example=ExampleName    # Build a specific example
#   ./Build.sh --clean --example=ExampleName  # Clean and build an example
#
# ****************************************************************************

# Exit immediately if any command fails
set -e

# If a command in a pipeline fails, the whole pipeline fails
set -o pipefail

echo "########## Build.sh - start ##########"

# Check for required system packages
echo "Checking for required system packages..."

# Check if we're on a system with apt
if command -v apt-get &> /dev/null; then
    MISSING_PACKAGES=""
    
    # List of required packages
    REQUIRED_PACKAGES=(
        "cmake"
        "gcc-arm-none-eabi"
        "libnewlib-arm-none-eabi" 
        "build-essential" 
        "g++" 
        "libstdc++-arm-none-eabi-newlib"
        "libusb-1.0-0-dev"
        "pkg-config"
    )
    
    # Check each required package
    for pkg in "${REQUIRED_PACKAGES[@]}"; do
        if ! dpkg-query -W -f='${Status}' $pkg 2>/dev/null | grep -q "ok installed"; then
            echo "Package $pkg is not installed."
            MISSING_PACKAGES="$MISSING_PACKAGES $pkg"
        fi
    done
    
    # Install missing packages if any
    if [ ! -z "$MISSING_PACKAGES" ]; then
        echo "Installing missing packages:$MISSING_PACKAGES"
        sudo apt-get update
        sudo apt-get install -y $MISSING_PACKAGES
        echo "Required packages installed."
    else
        echo "All required packages are already installed."
    fi
else
    echo "Warning: This script cannot check for packages on non-Debian based systems."
    echo "Please ensure you have these packages installed:"
    echo "- cmake"
    echo "- gcc-arm-none-eabi"
    echo "- libnewlib-arm-none-eabi"
    echo "- build-essential"
    echo "- g++"
    echo "- libstdc++-arm-none-eabi-newlib"
    echo "- libusb-1.0-0-dev"
fi

# Check if git submodules are initialized
echo "Checking git submodules..."
# Create Dependencies directory if it doesn't exist
DEPS_DIR="$(cd "$(dirname "$0")/../Dependencies" && pwd)"
if [ ! -d "$DEPS_DIR" ]; then
    mkdir -p "$DEPS_DIR"
    echo "Created Dependencies directory"
fi

# Get git repo root
REPO_ROOT="$(git -C "$(dirname "$0")" rev-parse --show-toplevel)"

# Add or initialize a submodule.
# If not registered in .gitmodules (e.g. fresh repo copy), runs git submodule add.
# If registered but not cloned, runs git submodule update --init.
add_or_init_submodule()
{
    local NAME="$1"
    local URL="$2"
    local EXTRA_ARGS="$3"
    local SUBMODULE_PATH="UnitApplication/Dependencies/$NAME"
    local SUBMODULE_DIR="$REPO_ROOT/$SUBMODULE_PATH"

    if ! git -C "$REPO_ROOT" config --file "$REPO_ROOT/.gitmodules" --get "submodule.$SUBMODULE_PATH.url" &>/dev/null 2>&1; then
        echo "Registering and cloning submodule $NAME..."
        git -C "$REPO_ROOT" submodule add $EXTRA_ARGS "$URL" "$SUBMODULE_PATH"
    elif [ ! -d "$SUBMODULE_DIR/.git" ]; then
        echo "Initializing submodule $NAME..."
        git -C "$REPO_ROOT" submodule update --init --recursive -- "$SUBMODULE_PATH"
    fi
}

add_or_init_submodule "FreeRTOS-Kernel" "https://github.com/raspberrypi/FreeRTOS-Kernel.git"
add_or_init_submodule "pico-sdk"        "https://github.com/raspberrypi/pico-sdk.git"
add_or_init_submodule "picotool"        "https://github.com/raspberrypi/picotool.git"
add_or_init_submodule "openocd"         "https://github.com/raspberrypi/openocd.git" "--branch sdk-2.0.0"
add_or_init_submodule "debugprobe"      "https://github.com/raspberrypi/debugprobe.git"

# Verify FreeRTOS-Kernel is at the correct commit
FREERTOS_DIR="$DEPS_DIR/FreeRTOS-Kernel"
FREERTOS_COMMIT="4f7299d6ea746b27a9dd19e87af568e34bd65b15"
ACTUAL_COMMIT=$(cd "$FREERTOS_DIR" && git rev-parse HEAD)
if [ "$ACTUAL_COMMIT" != "$FREERTOS_COMMIT" ]; then
    echo "FreeRTOS-Kernel is not at the expected commit. Updating..."
    (cd "$FREERTOS_DIR" && git fetch && git checkout "$FREERTOS_COMMIT")
fi

# Verify pico-sdk is at the correct version
PICO_SDK_DIR="$DEPS_DIR/pico-sdk"
PICO_SDK_TAG="2.1.1"
ACTUAL_TAG=$(cd "$PICO_SDK_DIR" && git describe --tags --exact-match 2>/dev/null || echo "unknown")
if [ "$ACTUAL_TAG" != "$PICO_SDK_TAG" ]; then
    echo "pico-sdk is not at the expected version. Updating..."
    (cd "$PICO_SDK_DIR" && git fetch && git checkout "$PICO_SDK_TAG" && git submodule update --init)
fi

# Verify picotool is at the correct version
PICOTOOL_DIR="$DEPS_DIR/picotool"
PICOTOOL_TAG="2.1.1"
ACTUAL_TAG=$(cd "$PICOTOOL_DIR" && git describe --tags --exact-match 2>/dev/null || echo "unknown")
if [ "$ACTUAL_TAG" != "$PICOTOOL_TAG" ]; then
    echo "picotool is not at the expected version. Updating..."
    (cd "$PICOTOOL_DIR" && git fetch && git checkout "$PICOTOOL_TAG")
fi

# Verify OpenOCD is at the correct commit
OPENOCD_DIR="$DEPS_DIR/openocd"
OPENOCD_COMMIT="cf9c0b41cd5c45b2faf01b4fd1186f160342b7b7"
ACTUAL_COMMIT=$(cd "$OPENOCD_DIR" && git rev-parse HEAD)
if [ "$ACTUAL_COMMIT" != "$OPENOCD_COMMIT" ]; then
    echo "OpenOCD is not at the expected commit. Updating..."
    (cd "$OPENOCD_DIR" && git fetch && git checkout "$OPENOCD_COMMIT")
fi

# Verify debugprobe is at the correct version
DEBUGPROBE_DIR="$DEPS_DIR/debugprobe"
DEBUGPROBE_TAG="debugprobe-v2.2.2"
ACTUAL_TAG=$(cd "$DEBUGPROBE_DIR" && git describe --tags --exact-match 2>/dev/null || echo "unknown")
if [ "$ACTUAL_TAG" != "$DEBUGPROBE_TAG" ]; then
    echo "debugprobe is not at the expected version. Updating..."
    (cd "$DEBUGPROBE_DIR" && git fetch && git checkout "$DEBUGPROBE_TAG")
fi

# Build and install picotool
# Check if picotool needs to be built/installed (always if not found in /usr/local/bin)
if [ ! -f "/usr/local/bin/picotool" ]; then
    echo "Building and installing picotool..."
    
    # Create build directory if it doesn't exist
    if [ ! -d "$PICOTOOL_DIR/build" ]; then
        mkdir -p "$PICOTOOL_DIR/build"
    fi
    
    # Configure, build and install picotool
    cd "$PICOTOOL_DIR/build"
    export PICO_SDK_PATH="$PICO_SDK_DIR"
    cmake ..
    make
    sudo make install
    
    # Return to the original directory
    cd "$OLDPWD"
    
    echo "picotool built and installed successfully"
fi

# Build and install OpenOCD if not already installed
if [ ! -f "$OPENOCD_DIR/src/openocd" ]; then
    echo "Building and installing OpenOCD..."
    
    cd "$OPENOCD_DIR"
    
    # Install additional dependencies for OpenOCD
    if command -v apt-get &> /dev/null; then
        sudo apt-get install -y autoconf automake libtool libusb-1.0-0-dev texinfo libhidapi-dev
    fi
    
    # Configure and build
    ./bootstrap
    ./configure --disable-werror
    make -j4
    sudo make install
    
    # Return to the original directory
    cd "$OLDPWD"
    
    echo "OpenOCD built and installed successfully"
fi

# Export environment variables for CMake
export FREERTOS_KERNEL_PATH="$FREERTOS_DIR"
export PICO_SDK_PATH="$PICO_SDK_DIR"
export PICOTOOL_FETCH_FROM_GIT_PATH="$PICOTOOL_DIR"
export OPENOCD_DIR="$OPENOCD_DIR"
export DEBUGPROBE_DIR="$DEBUGPROBE_DIR"

echo "Using FreeRTOS-Kernel at: $FREERTOS_KERNEL_PATH"
echo "Using pico-sdk at: $PICO_SDK_PATH"
echo "Using picotool at: $PICOTOOL_DIR"
echo "Using OpenOCD at: $OPENOCD_DIR"
echo "Using debugprobe at: $DEBUGPROBE_DIR"

# --- Interactive Menu ---

CLEAN_BUILD=0
EXAMPLE_NAME=""

# Function to display the main menu
show_main_menu() {
    clear
    echo "****************************************"
    echo "*              Build Menu              *"
    echo "****************************************"
    echo "1. Build Application"
    echo "2. Clean and Build Application"
    echo "3. Build Example"
    echo "4. Clean and Build Example"
    echo "5. Exit"
    echo "****************************************"
    echo -n "Please select an option [1-5]: "
}

# Function to select an example
select_example() {
    clear
    echo "****************************************"
    echo "*           Select Example             *"
    echo "****************************************"
    
    # Find examples
    EXAMPLES=()
    if [ -d "Examples" ]; then
        for dir in Examples/*/; do
            if [ -d "$dir" ]; then
                EXAMPLES+=("$(basename "$dir")")
            fi
        done
    fi

    if [ ${#EXAMPLES[@]} -eq 0 ]; then
        echo "No examples found in the 'Examples' directory."
        echo "Press any key to return to the main menu."
        read -n 1 -s
        return 1
    fi

    echo "Available examples:"
    for i in "${!EXAMPLES[@]}"; do
        echo "$((i+1)). ${EXAMPLES[$i]}"
    done
    echo "$(( ${#EXAMPLES[@]} + 1 )). Back to Main Menu"
    echo "****************************************"
    
    while true; do
        echo -n "Please select an example: "
        read -r choice
        if [[ "$choice" -ge 1 && "$choice" -le ${#EXAMPLES[@]} ]]; then
            EXAMPLE_NAME="${EXAMPLES[$((choice-1))]}"
            return 0
        elif [[ "$choice" -eq $(( ${#EXAMPLES[@]} + 1 )) ]]; then
            return 1
        else
            echo "Invalid option. Please try again."
        fi
    done
}

# Main menu loop
while true; do
    show_main_menu
    read -r option
    case $option in
        1) # Build Application
            CLEAN_BUILD=0
            EXAMPLE_NAME=""
            break
            ;;
        2) # Clean and Build Application
            CLEAN_BUILD=1
            EXAMPLE_NAME=""
            break
            ;;
        3) # Build Example
            if select_example; then
                CLEAN_BUILD=0
                break
            fi
            ;;
        4) # Clean and Build Example
            if select_example; then
                CLEAN_BUILD=1
                break
            fi
            ;;
        5) # Exit
            echo "Exiting script."
            exit 0
            ;;
        *)
            echo "Invalid option. Please try again."
            sleep 1
            ;;
    esac
done

# --- End of Interactive Menu ---

# Handle clean build if requested
if [ $CLEAN_BUILD -eq 1 ]; then
    echo "Performing clean build..."
    if [ -d "build" ]; then
        rm -rf build
        echo "Removed existing build directory"
    fi
fi

# Create build directory if it does not exist
if [ ! -d "build" ]; then
    mkdir build
    echo "Created build directory"
else
    echo "Build directory already exists"
fi

# Configure the build directory
echo "Configuring the build directory..."
cd build

# Set CMake arguments based on whether an example is selected
CMAKE_ARGS=".."

# Check if an example was specified
if [[ ! -z "$EXAMPLE_NAME" ]]; then
    if [[ -f "../Examples/$EXAMPLE_NAME/main.c" ]]; then
        echo "Building with example: $EXAMPLE_NAME"
        # Add the MAIN_SOURCE parameter to CMake arguments
        CMAKE_ARGS="$CMAKE_ARGS -DMAIN_SOURCE=../Examples/$EXAMPLE_NAME/main.c"
    else
        echo "Warning: Example '$EXAMPLE_NAME' not found. Using default main.c"
        echo "Run ./Build.sh --example=list to see available examples"
    fi
fi

# Run CMake to configure the build system
# This command generates the necessary build files in the 'build' directory.
# These files include:
# - Makefiles or project files for the chosen build system (e.g., Make, Ninja, Visual Studio).
# - Configuration files that define how the project should be built.
# - Dependency files that track which files need to be recompiled when changes are made.
cmake $CMAKE_ARGS

# Build the project using the generated build files
# This command compiles the project based on the configuration done by the previous cmake command.
# It uses the generated Makefiles or project files to compile the source code into executables or libraries.
echo "Building..."
cmake --build . -- -j$(nproc)

echo "Build complete."

echo "Copying output files..."

cp -v ./Application/moduri_bank_A.elf     ../output
cp -v ./Application/moduri_bank_A.elf.map ../output
cp -v ./Application/moduri_bank_A.uf2     ../output
cp -v ./Application/moduri_bank_A.bin     ../output

cp -v ./Application/moduri_bank_B.elf     ../output
cp -v ./Application/moduri_bank_B.elf.map ../output
cp -v ./Application/moduri_bank_B.uf2     ../output
cp -v ./Application/moduri_bank_B.bin     ../output

cp -v ./Bootloader/bootloader.elf ../output
cp -v ./Bootloader/bootloader.elf.map ../output
cp -v ./Bootloader/bootloader.uf2 ../output
cp -v ./Bootloader/bootloader.bin ../output

cp -v ./Bootloader/metadata.bin ../output

echo "Done"

# Prompt the user to press any key to exit
echo "Press any key to exit the script."

# Handle user input
read -n 1 -s  # Read a single keypress silently
echo "Exiting the script."


