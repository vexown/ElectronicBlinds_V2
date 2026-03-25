
message("########## HAL/CMakeLists.txt - start ##########")

# Define the path to the source files for HAL
set(HAL_SOURCE_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/Source/GPIO_HAL.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Source/I2C_HAL.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Source/SPI_HAL.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Source/ADC_HAL.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Source/PWM_HAL.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Source/PIO_HAL.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Source/DMA_HAL.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Source/timer_HAL.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Source/UART_HAL.c
    )

# Define the path to the include directory for HAL
set(HAL_INCLUDE_DIR
    ${CMAKE_CURRENT_SOURCE_DIR}/Include
    )

# Export the variable to the parent scope
set(HAL_SOURCE_FILES ${HAL_SOURCE_FILES} PARENT_SCOPE)
set(HAL_INCLUDE_DIR ${HAL_INCLUDE_DIR} PARENT_SCOPE)

message("########## HAL/CMakeLists.txt - end ##########")
