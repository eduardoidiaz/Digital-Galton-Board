# cmake version
cmake_minimum_required(VERSION 3.13)

# include the sdk.cmake file
include(pico_sdk_import.cmake)

# give the project a name (anything you want)
project(Digital_Galton_Board C CXX ASM)

# initialize the sdk
pico_sdk_init()

add_executable(Digital_Galton_Board)

add_compile_options(-Ofast)

# enable usb output
pico_enable_stdio_usb(Digital_Galton_Board 1)
# pico_enable_stdio_uart(VGA_Animation_Demo 0)

# must match with pio filename and executable name from above
pico_generate_pio_header(Digital_Galton_Board ${CMAKE_CURRENT_LIST_DIR}/hsync.pio)
pico_generate_pio_header(Digital_Galton_Board ${CMAKE_CURRENT_LIST_DIR}/vsync.pio)
pico_generate_pio_header(Digital_Galton_Board ${CMAKE_CURRENT_LIST_DIR}/rgb.pio)

# must match with executable name and source file names
target_sources(Digital_Galton_Board PRIVATE board.c vga16_graphics_v2.c)

# must match with executable name
target_link_libraries(Digital_Galton_Board PRIVATE pico_stdlib pico_divider pico_multicore pico_bootsel_via_double_reset hardware_pio hardware_dma hardware_adc hardware_irq hardware_clocks hardware_pll)

# must match with executable name
pico_add_extra_outputs(Digital_Galton_Board)