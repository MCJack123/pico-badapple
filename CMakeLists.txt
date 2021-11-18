cmake_minimum_required(VERSION 3.12)

# Pull in PICO SDK (must be before project)
include(pico_sdk_import.cmake)

# We also need PICO EXTRAS
include(pico_extras_import.cmake)

project(badapple C CXX)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DPICO_SCANVIDEO_SYNC_PIN_BASE=20 -DPICO_SCANVIDEO_MAX_SCANLINE_BUFFER_WORDS=500 -DPICO_SCANVIDEO_MISSING_SCANLINE_COLOR=0")

# Initialize the Pico SDK
pico_sdk_init()

add_executable(badapple
    badapple.c
    hw_config.c
)

pico_enable_stdio_usb(badapple 1)
pico_enable_stdio_uart(badapple 0)

add_subdirectory(no-OS-FatFS-SD-SPI-RPi-Pico/FatFs_SPI fatfs)

target_link_libraries(badapple PRIVATE pico_multicore pico_stdlib pico_scanvideo_dpi hardware_pwm FatFs_SPI)
pico_add_extra_outputs(badapple)
