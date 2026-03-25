# ****************************************************************************
# CMakeLists.txt
#
# Description:
# This is the top-level CMake configuration file for the Moduri project.
# It sets up the project and includes necessary CMake modules for building
# the project. The configuration includes:
# 1. Minimum required CMake version.
# 2. Project definition with supported languages (C, C++, ASM).
# 3. Standard settings for C and C++.
# 4. Inclusion of the Pico SDK configuration and initialization.
# 5. Inclusion of the FreeRTOS kernel configuration.
# 6. Addition of subdirectories for project components.
# 7. ...any future top level build system configuration
#
# Usage:
# Place this file at the root of the project directory. Ensure that
# pico_sdk_import.cmake and FreeRTOS_Kernel_import.cmake are located
# in the same directory or adjust the paths accordingly.
#
# ****************************************************************************
cmake_minimum_required(VERSION 3.13)

# Print messages for script tracking and debugging
message("########## TOP CMakeLists.txt - start ##########")

# Specify build type within the Moduri project
option(DEBUG_BUILD "Enable debug build" ON)
option(RELEASE_BUILD "Enable release build" OFF)

if(DEBUG_BUILD AND RELEASE_BUILD)
    message(FATAL_ERROR "Cannot define both DEBUG_BUILD and RELEASE_BUILD.")
elseif(DEBUG_BUILD)
    add_compile_definitions(DEBUG_BUILD)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build." FORCE)
elseif(RELEASE_BUILD)
    add_compile_definitions(RELEASE_BUILD)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
else()
    message(FATAL_ERROR "Please define either DEBUG_BUILD or RELEASE_BUILD.")
endif()

# Specify parameters for pico-sdk
set(PICO_BOARD pico2_w)
set(PICO_PLATFORM rp2350)
set(PICO_COMPILER pico_arm_cortex_m33_gcc)

######################### Pre build imports ###############################

# Include the Pico SDK configuration script
# This script sets up the Raspberry Pi Pico SDK, platform, toolchain, and some CMake utility functions.
message("########## pico_sdk_import.cmake - start ##########")
include(pico_sdk_import.cmake)

# Include the FreeRTOS configuration script
# This script sets up variables and configuration for the FreeRTOS kernel.
message("########## FreeRTOS_Kernel_import.cmake - start ##########")
include(FreeRTOS_Kernel_import.cmake)

############################################################################

# Set project name and supported languages
project(Moduri C CXX ASM)

# Set C and C++ standards
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Export the compile commands for analysis and use by other tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Enable SSL support in pico-sdk
set(PICO_ENABLE_SSL 1 CACHE INTERNAL "")
# Set the user-defined mbedtls configuration file
set(MBEDTLS_CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/Application/OS/Include/mbedtls_config.h" CACHE STRING "" FORCE)

# Initialize the Pico SDK
# This macro sets up the Pico SDK and prepares it for use.
# Take note of the board configuration parameters set in https://github.com/raspberrypi/pico-sdk/blob/master/src/boards/include/boards/pico2_w.h
message("########## pico_sdk_init macro - start ##########")
pico_sdk_init()

# Add the FlashUtils library as a common dependency
add_subdirectory(Common/FlashUtils)

# Build bootloader first
add_subdirectory(Bootloader)

# Then build main application
add_subdirectory(Application)

message("########## TOP CMakeLists.txt - end ##########")