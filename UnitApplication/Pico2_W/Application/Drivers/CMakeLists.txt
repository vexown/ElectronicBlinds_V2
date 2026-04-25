message("########## Drivers/CMakeLists.txt - start ##########")

# Enable C++ support
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Define the path to the source files for Drivers
set(DRIVERS_SOURCE_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/Source/SSD1306_HAL.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Source/MMC56x3.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Source/MMC56x3_wrapper.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Source/UBLOX_GPS_HAL.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Source/DS3231_HAL.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Source/AT24C32_HAL.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Source/h_bridge_controller.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Source/button_debouncer.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Source/VEML7700.c
    )

# Define the path to the include directory for Drivers
set(DRIVERS_INCLUDE_DIR
    ${CMAKE_CURRENT_SOURCE_DIR}/Include
    )

# Export the variables to the parent scope
set(DRIVERS_SOURCE_FILES ${DRIVERS_SOURCE_FILES} PARENT_SCOPE)
set(DRIVERS_INCLUDE_DIR ${DRIVERS_INCLUDE_DIR} PARENT_SCOPE)

message("########## Drivers/CMakeLists.txt - end ##########")