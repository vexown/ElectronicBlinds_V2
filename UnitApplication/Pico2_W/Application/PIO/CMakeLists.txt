
message("########## PIO/CMakeLists.txt - start ##########")

# Define the path to the assembly source files for PIO
set(PIO_SOURCE_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/blink.pio
    ${CMAKE_CURRENT_SOURCE_DIR}/PWM.pio
    )

# Export the variables to the parent scope
set(PIO_SOURCE_FILES ${PIO_SOURCE_FILES} PARENT_SCOPE)

message("########## PIO/CMakeLists.txt - end ##########")
