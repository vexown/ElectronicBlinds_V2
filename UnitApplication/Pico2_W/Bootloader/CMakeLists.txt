message("########## Bootloader CMakeLists.txt - start ##########")

#################################################### Metadata ####################################################

# Create object file from metadata source
add_library(metadata_obj OBJECT
    Source/initial_metadata.c
)

# Link the flash utils library to the metadata object
target_link_libraries(metadata_obj PRIVATE flash_utils)

# Specify the include directories
target_include_directories(metadata_obj PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Include
)

# Set compile options to ensure minimal sections
target_compile_options(metadata_obj PRIVATE
    -fno-builtin      # Don't use compiler built-in functions
    -fno-function-sections  # Don't put each function in its own section
    -fdata-sections   # Don't put each data item in its own section
    -ffreestanding    # Compile for freestanding environment (no OS)
)

# Custom command to create binary from object file
#
# The command looks something like this: 
#   arm-none-eabi-objcopy -O binary metadata_obj metadata.bin
#
# The -O binary option allows us to output a raw binary file.
#
# A "raw" binary file is a plain sequence of bytes without any metadata, headers, or formatting. 
# It is just the literal contents of memory as they should appear in a device, without any additional structure.
# This means:
#   - No headers (like ELF or COFF)  
#   - No addressing information (it must be loaded at a known location)  
#   - Directly loadable into memory or flashed onto a device  
#
# This format is commonly used for firmware images, bootloader payloads, and raw data storage.
add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/metadata.bin   # CMake - Output binary file
    COMMAND ${CMAKE_OBJCOPY} -O binary                # Use objcopy to create binary...
            $<TARGET_OBJECTS:metadata_obj>            # ...from the source object file...  
            ${CMAKE_CURRENT_BINARY_DIR}/metadata.bin  # ...to the output binary file
    DEPENDS metadata_obj                              # CMake - Dependency on object file (rebuild if object file changes)
    COMMENT "Generating metadata binary"              # CMake - Output message
)

# Add custom target to ensure binary gets built
add_custom_target(metadata ALL
    DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/metadata.bin
)

#################################################### Bootloader ####################################################

# Define the executable target for the bootloader and specify the source files
add_executable(bootloader
    ${CMAKE_CURRENT_SOURCE_DIR}/main.c
)

# Link the flash utils library to the bootloader
target_link_libraries(bootloader PRIVATE flash_utils)

# Specify the include directories
target_include_directories(bootloader PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Include
)

# Link libraries and dependencies
target_link_libraries(bootloader PRIVATE
                        pico_stdlib
                        hardware_flash
)

# Use custom linker script
pico_set_linker_script(bootloader ${CMAKE_CURRENT_SOURCE_DIR}/bootloader.ld)

# Generate extra outputs (map/bin/hex/uf2 files in addition to ELF)
pico_add_extra_outputs(bootloader)

message("########## Bootloader CMakeLists.txt - end ##########")