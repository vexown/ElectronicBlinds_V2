
message("########## Monitor/CMakeLists.txt - start ##########")

# Define the path to the source files for WiFi
set(MONITOR_SOURCE_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/Source/Monitor_main.c
    )

# Define the path to the include directory for WiFi
set(MONITOR_INCLUDE_DIR
    ${CMAKE_CURRENT_SOURCE_DIR}/Include
    )

# Export the variables to the parent scope
set(MONITOR_SOURCE_FILES ${MONITOR_SOURCE_FILES} PARENT_SCOPE)
set(MONITOR_INCLUDE_DIR ${MONITOR_INCLUDE_DIR} PARENT_SCOPE)

message("########## Monitor/CMakeLists.txt - end ##########")
