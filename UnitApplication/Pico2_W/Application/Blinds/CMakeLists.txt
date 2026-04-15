message("########## Blinds/CMakeLists.txt - start ##########")

# Define the path to the source files for the Blinds module
set(BLINDS_SOURCE_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/Source/blinds_controller.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Source/time_calculations.c
    )

# Define the path to the include directory for the Blinds module
set(BLINDS_INCLUDE_DIR
    ${CMAKE_CURRENT_SOURCE_DIR}/Include
    )

# Export the variables to the parent scope
set(BLINDS_SOURCE_FILES ${BLINDS_SOURCE_FILES} PARENT_SCOPE)
set(BLINDS_INCLUDE_DIR  ${BLINDS_INCLUDE_DIR}  PARENT_SCOPE)

message("########## Blinds/CMakeLists.txt - end ##########")
