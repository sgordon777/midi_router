# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(midi_router C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()




# Add executable. Default name is the project name, version 0.1
add_executable(midi_router
    src/main.c
    src/usb_descriptors.c
    )


# newly added
target_include_directories( ${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/src
    ${CMAKE_CURRENT_LIST_DIR}
    ${PICO_SDK_PATH}/lib/tinyusb/src
    ${PICO_SDK_PATH}/lib/tinyusb/hw           # <-- add this so "bsp/board_api.h" resolves

)


# pull in common dependencies
target_link_libraries(midi_router 
  pico_stdlib
  pico_stdlib
  tinyusb_device
  tinyusb_board
)

if (PICO_CYW43_SUPPORTED)
    target_link_libraries(midi_router 
    pico_cyw43_arch_none
    pico_stdlib
    tinyusb_device
    tinyusb_board
    )
endif()

# create map/bin/hex file etc.
pico_add_extra_outputs(midi_router)

# newly added
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)


# add url via pico_set_program_url
