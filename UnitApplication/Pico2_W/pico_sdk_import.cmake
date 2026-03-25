# ------------------------------------------------------------------------------
# This CMake script enforces the use of the Pico SDK from the project's
# submodule at a specific version. This ensures consistent builds regardless of
# environment settings.
#
# Git repository: https://github.com/raspberrypi/pico-sdk
# Specific tag: 2.1.1
# ------------------------------------------------------------------------------

# Get the absolute path to the current file
get_filename_component(CURRENT_DIR "${CMAKE_CURRENT_LIST_DIR}" ABSOLUTE)

# Force using the submodule path
set(PICO_SDK_PATH "../Dependencies/pico-sdk" CACHE PATH "Path to the Pico SDK" FORCE)

message(STATUS "Using Pico SDK from submodule ('${PICO_SDK_PATH}')")

# Convert PICO_SDK_PATH to an absolute path
get_filename_component(PICO_SDK_PATH "${PICO_SDK_PATH}" REALPATH)

# Set the path to the pico_sdk_init.cmake file in the SDK directory
set(PICO_SDK_INIT_CMAKE_FILE ${PICO_SDK_PATH}/pico_sdk_init.cmake)

# Stop if the pico_sdk_init.cmake file is not found in the SDK directory
if (NOT EXISTS ${PICO_SDK_INIT_CMAKE_FILE})
    message(FATAL_ERROR "'${PICO_SDK_PATH}' does not contain pico_sdk_init.cmake file. Make sure the PICO_SDK_PATH points to the pico-sdk directory")
endif ()

# Include (load and execute) the SDK initialization file
message("########## pico_sdk_init.cmake - start ##########")
include(${PICO_SDK_INIT_CMAKE_FILE})
message("########## pico_sdk_init.cmake - end ##########")




