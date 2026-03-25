
message("########## OS/CMakeLists.txt - start ##########")

# Define the path to the source files for OS
set(OS_SOURCE_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/Source/OS_manager.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Source/OS_hooks.c
    )

# Define the path to the include directory for OS
set(OS_INCLUDE_DIR
    ${CMAKE_CURRENT_SOURCE_DIR}/Include
    )

# Export the variables to the parent scope
set(OS_SOURCE_FILES ${OS_SOURCE_FILES} PARENT_SCOPE)
set(OS_INCLUDE_DIR ${OS_INCLUDE_DIR} PARENT_SCOPE)

message("########## OS/CMakeLists.txt - end ##########")
