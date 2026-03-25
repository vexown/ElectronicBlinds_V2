
message("########## WiFi/CMakeLists.txt - start ##########")

# Define the path to the source files for WiFi
set(WIFI_SOURCE_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/Source/WiFi_DNS_server.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Source/WiFi_DHCP_server.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Source/WiFi_HTTP.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Source/WiFi_UDP.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Source/WiFi_TCP.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Source/WiFi_Init.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Source/WiFi_main.c 
    ${CMAKE_CURRENT_SOURCE_DIR}/Source/WiFi_OTA_download.c
    )

# Define the path to the include directory for WiFi
set(WIFI_INCLUDE_DIR
    ${CMAKE_CURRENT_SOURCE_DIR}/Include
    )

# Export the variables to the parent scope
set(WIFI_SOURCE_FILES ${WIFI_SOURCE_FILES} PARENT_SCOPE)
set(WIFI_INCLUDE_DIR ${WIFI_INCLUDE_DIR} PARENT_SCOPE)

message("########## WiFi/CMakeLists.txt - end ##########")
