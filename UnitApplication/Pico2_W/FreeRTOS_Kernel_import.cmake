# ------------------------------------------------------------------------------
# This CMake script enforces the use of the FreeRTOS Kernel from the project's
# submodule at a specific version. This ensures consistent builds regardless of
# environment settings.
#
# Git repository: https://github.com/raspberrypi/FreeRTOS-Kernel
# Specific commit: 4f7299d6ea746b27a9dd19e87af568e34bd65b15
# ------------------------------------------------------------------------------

# Get the absolute path to the current file
get_filename_component(CURRENT_DIR "${CMAKE_CURRENT_LIST_DIR}" ABSOLUTE)

# Force using the submodule path
set(FREERTOS_KERNEL_PATH "../Dependencies/FreeRTOS-Kernel" CACHE PATH "Path to the FreeRTOS Kernel" FORCE)

message(STATUS "Using FreeRTOS Kernel from submodule ('${FREERTOS_KERNEL_PATH}')")

# Convert FREERTOS_KERNEL_PATH to an absolute path
get_filename_component(FREERTOS_KERNEL_PATH "${FREERTOS_KERNEL_PATH}" REALPATH)

# Define the path to the FreeRTOS port directory for RP2350
set(FREERTOS_KERNEL_RP2350_PATH "${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2350_ARM_NTZ")

# Stop if the FreeRTOS port directory does not exist
if (NOT EXISTS "${FREERTOS_KERNEL_RP2350_PATH}")
    message(FATAL_ERROR "Directory '${FREERTOS_KERNEL_RP2350_PATH}' not found. Make sure the FREERTOS_KERNEL_PATH points to the FreeRTOS kernel directory")
endif ()

# Include the FreeRTOS port directory for RP2350
message(STATUS "########## FreeRTOS RP2350 CMakeLists.txt - start ##########")
add_subdirectory(${FREERTOS_KERNEL_RP2350_PATH} FREERTOS_KERNEL)
message(STATUS "########## FreeRTOS RP2350 CMakeLists.txt - end ##########")
