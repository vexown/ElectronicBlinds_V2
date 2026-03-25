
message("########## Misc/CMakeLists.txt - start ##########")

# Define the path to the source files for Misc
set(MISC_SOURCE_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/Source/Common.c
    )

# Define the path to the include directory for Misc
set(MISC_INCLUDE_DIR
    ${CMAKE_CURRENT_SOURCE_DIR}/Include
    )

# Export the variable to the parent scope
set(MISC_SOURCE_FILES ${MISC_SOURCE_FILES} PARENT_SCOPE)
set(MISC_INCLUDE_DIR ${MISC_INCLUDE_DIR} PARENT_SCOPE)

message("########## Misc/CMakeLists.txt - end ##########")
