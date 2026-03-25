message("########## Application/Standard CMakeLists.txt - start ##########")

# Define the main source file with a default value (original application main.c)
# This is used to give a possibility to the user to change the main.c source file to a one from Examples folder by calling a special build command
set(MAIN_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/main.c" CACHE FILEPATH "Path to the main.c file to use")
message(STATUS "Using main source: ${MAIN_SOURCE}")

# Add subdirectories of the Software Components
add_subdirectory(OS)
add_subdirectory(Misc)
add_subdirectory(WiFi)
add_subdirectory(Monitor)
add_subdirectory(HAL)
add_subdirectory(Drivers)
add_subdirectory(PIO)

# Aggregate all application source files into a single list
set(APP_SOURCES
    ${MAIN_SOURCE} 
    ${OS_SOURCE_FILES}
    ${MISC_SOURCE_FILES}
    ${WIFI_SOURCE_FILES}
    ${MONITOR_SOURCE_FILES}
    ${HAL_SOURCE_FILES}
    ${DRIVERS_SOURCE_FILES}
)

# Helper function to create a moduri target for a given bank
function(create_moduri_target target_name linker_script)

    add_executable(${target_name}
        ${APP_SOURCES} 
    )

    # Set the compiler flags
    # These compiler flags are for the GNU compiler collection (GCC):
    # https://gcc.gnu.org/onlinedocs/gcc/Option-Summary.html (general options but also ARM-specific ones)
    set(COMMON_WARNING_FLAGS
    -Wall                # Enable all standard warnings
    -Wextra              # Enable extra warnings
    -Wpedantic           # Warn about non-standard C/C++
    -Wshadow             # Warn when a variable shadows another
    -Wdouble-promotion   # Implicit float to double promotions
    -Wformat=2           # Stricter format checking
    -Wundef              # Undefined macro evaluates to 0
    -Wconversion         # Implicit type conversions
    -Wunused             # Unused entities
    -Wno-unused-function # Don't warn about unused static functions in headers
    -fstack-usage        # Generate stack usage information
    -Werror              # Treat warnings as errors
    )

    # C-specific warning flags
    set(C_WARNING_FLAGS
        -Wstrict-prototypes  # Function declared without parameter types
        -Wbad-function-cast  # Suspicious cast of function return value
    )

    # Specify optimization flags
    if(DEBUG_BUILD)
        add_compile_options(-Og) # -Og: Reasonable level of optimization while maintaining fast compilation and a good debugging experience
    else()
        add_compile_options(-O2) # -O2: GCC performs nearly all supported optimizations that do not involve a space-speed tradeoff
    endif()

    # Combine the Common and C-specific warning flags
    list(APPEND APP_COMPILE_OPTIONS ${COMMON_WARNING_FLAGS})
    list(APPEND APP_COMPILE_OPTIONS "$<$<COMPILE_LANGUAGE:C>:${C_WARNING_FLAGS}>")

    # Remove ThirdParty source files from the list of files to check for warnings
    # Double check if the file paths are correct, otherwise the files will not be removed (see in build output)
    list(APPEND APP_SOURCES_FILTERED ${APP_SOURCES})
    list(REMOVE_ITEM APP_SOURCES_FILTERED ${CMAKE_CURRENT_SOURCE_DIR}/WiFi/Source/WiFi_DHCP_server.c)
    list(REMOVE_ITEM APP_SOURCES_FILTERED ${CMAKE_CURRENT_SOURCE_DIR}/WiFi/Source/WiFi_DNS_server.c)

    # Add the warnings to the Application source files
    set_source_files_properties(${APP_SOURCES_FILTERED}
                                PROPERTIES COMPILE_OPTIONS 
                                "${APP_COMPILE_OPTIONS}"
                                )

    # Set linker warning flags (GNU ld - https://sourceware.org/binutils/docs/ld/Options.html)
    # Used flags:
    #       -Wl,--warn-common: Warn about any common symbols that are defined in multiple object files
    #       -Wl,--warn-section-align: Warn if there are any sections that are not properly aligned
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} \
    -Wl,--warn-common \
    -Wl,--warn-section-align \
    ")
                                  
    # Link the flash utils library to the target
    target_link_libraries(${target_name} PRIVATE flash_utils)

    # Set compile definitions if needed
    target_compile_definitions(${target_name} PRIVATE)

    # Set include directories
    target_include_directories(${target_name} PRIVATE
        ${OS_INCLUDE_DIR} 
        ${MISC_INCLUDE_DIR}
        ${WIFI_INCLUDE_DIR}
        ${MONITOR_INCLUDE_DIR}
        ${HAL_INCLUDE_DIR}
        ${DRIVERS_INCLUDE_DIR}
    )

    # Link libraries and dependencies
    target_link_libraries(${target_name} PRIVATE
        # Core system libraries
        pico_stdlib
        pico_cyw43_arch_lwip_sys_freertos
        
        # Network stack
        pico_lwip_iperf
        
        # Security libraries
        pico_mbedtls

        # Hardware peripherals
        hardware_i2c
        hardware_adc
        hardware_spi
        hardware_pwm
        hardware_pio
        hardware_dma
        
        # RTOS
        FreeRTOS-Kernel-Heap4
    )

    # Generate the PIO header files
    foreach(PIO_FILE ${PIO_SOURCE_FILES})
        pico_generate_pio_header(${target_name} ${PIO_FILE})
    endforeach()

    # Use the provided custom linker script for this target
    pico_set_linker_script(${target_name} ${CMAKE_CURRENT_SOURCE_DIR}/${linker_script})

    # Add extra outputs specific to the Pico SDK
    pico_add_extra_outputs(${target_name})
endfunction()

# Create two targets: one for Bank A and one for Bank B
create_moduri_target(moduri_bank_A "application_bank_A.ld")
create_moduri_target(moduri_bank_B "application_bank_B.ld")

message("########## Application/Standard CMakeLists.txt - end ##########")
