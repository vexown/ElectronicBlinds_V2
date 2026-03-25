message("########## FlashUtils CMakeLists.txt - start ##########")

add_library(flash_utils
    ${CMAKE_CURRENT_SOURCE_DIR}/Source/flash_operations.c
)

target_link_libraries(flash_utils PRIVATE
                        pico_stdlib
                        hardware_flash
)

target_include_directories(flash_utils PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/Include
    )

message("########## FlashUtils CMakeLists.txt - end ##########")