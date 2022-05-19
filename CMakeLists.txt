cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)
include(pimoroni_pico_import.cmake)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

project(fft-project C CXX ASM)

pico_sdk_init()

include(${PIMORONI_PICO_PATH}/drivers/st7789/st7789.cmake)
include(${PIMORONI_PICO_PATH}/libraries/bitmap_fonts/bitmap_fonts.cmake)
include(${PIMORONI_PICO_PATH}/libraries/pico_graphics/pico_graphics.cmake)
include(${PIMORONI_PICO_PATH}/libraries/generic_st7789/generic_st7789.cmake)

add_executable(fft)

target_sources(fft PRIVATE fft.cpp)

target_link_libraries(fft PRIVATE pico_stdlib hardware_pio hardware_dma hardware_adc hardware_irq generic_st7789)

pico_add_extra_outputs(fft)