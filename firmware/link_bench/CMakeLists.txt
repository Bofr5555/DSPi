# link_bench: the DSPi audio link bench image (WP LK3), a standalone Pico SDK
# project.  It is not part of firmware/CMakeLists.txt, so DSPi builds are not
# affected.  Build one directory per chip:
#
#   cmake -S firmware/link_bench -B <dir> -G Ninja -DPICO_PLATFORM=rp2350-arm-s -DPICO_BOARD=pico2
#   cmake -S firmware/link_bench -B <dir> -G Ninja -DPICO_PLATFORM=rp2040 -DPICO_BOARD=pico
#   cmake --build <dir>        -> <dir>/link_bench-rp2350.uf2 / link_bench-rp2040.uf2
#
# Documentation/Bench/link_gpio_bench.md describes the image.
cmake_minimum_required(VERSION 3.13)

set(LINK_BENCH_FW ${CMAKE_CURRENT_LIST_DIR}/..)
if (NOT PICO_SDK_PATH AND NOT DEFINED ENV{PICO_SDK_PATH})
    set(PICO_SDK_PATH ${LINK_BENCH_FW}/pico-sdk)
endif()

# As DSPi: at 307.2 MHz the default RP2040 flash divider of 2 is far above
# the flash's rating.  The image runs from RAM, so this only matters for the
# boot2 copy, but it costs nothing.  Must precede pico_sdk_init().
add_compile_definitions(PICO_FLASH_SPI_CLKDIV=6)

include(${LINK_BENCH_FW}/pico_sdk_import.cmake)
project(link_bench C CXX ASM)
set(CMAKE_C_STANDARD 11)
pico_sdk_init()

string(REGEX REPLACE "-.*" "" LINK_BENCH_CHIP "${PICO_PLATFORM}")   # rp2350-arm-s -> rp2350

find_package(Git QUIET)
set(LINK_BENCH_GIT "unknown")
if (GIT_FOUND)
    execute_process(COMMAND ${GIT_EXECUTABLE} describe --always --dirty --abbrev=8
                    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
                    OUTPUT_VARIABLE LINK_BENCH_GIT OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
endif()

add_executable(link_bench
    link_bench.c
    bench_gen.c
    bench_check.c
    ${LINK_BENCH_FW}/DSPi/link/bench_trim.c
    bench_cmd.c
    ${LINK_BENCH_FW}/DSPi/link/link_codec.c
    ${LINK_BENCH_FW}/DSPi/link/link_decoder.c
    # WP LK5: the production receiver and its decode benchmark
    bench_decode.c
    ${LINK_BENCH_FW}/DSPi/link/link_fast.c
)
if (PICO_PLATFORM MATCHES "rp2350-arm")
    # As in DSPi on the RP2350: the generated Thumb-2 bank decoder and frame
    # loop, and the CRC-16 by the DMA sniffer.
    target_sources(link_bench PRIVATE ${LINK_BENCH_FW}/DSPi/link/link_fast_m33.S
                                      ${LINK_BENCH_FW}/DSPi/link/link_fast_m33_loop.S)
    target_compile_definitions(link_bench PRIVATE LINK_FAST_M33=1 LINK_FAST_M33_SW=1 LINK_FAST_CRC_DMA=1)
endif()
set_target_properties(link_bench PROPERTIES OUTPUT_NAME link_bench-${LINK_BENCH_CHIP})
# Only the codec directory: firmware/DSPi itself holds DSPi's tusb_config.h.
target_include_directories(link_bench PRIVATE ${CMAKE_CURRENT_LIST_DIR} ${LINK_BENCH_FW}/DSPi/link)
target_compile_options(link_bench PRIVATE -O2 -Wall -Wextra)

# LK2's programs, unchanged, plus the IMPAIR pass-through.
foreach(p link_tx_20 link_tx_10 link_tx_5 link_rx_20 link_rx_10 link_rx_5)
    pico_generate_pio_header(link_bench ${LINK_BENCH_FW}/DSPi/${p}.pio)
endforeach()
pico_generate_pio_header(link_bench ${CMAKE_CURRENT_LIST_DIR}/bench_retime.pio)

target_compile_definitions(link_bench PRIVATE
    BENCH_GIT="${LINK_BENCH_GIT}"
    PICO_CORE1_STACK_SIZE=0x1000
    # Product string, so the console is easy to find among other Picos.
    USBD_PRODUCT="DSPi link bench"
)

target_link_libraries(link_bench
    pico_stdlib
    pico_multicore
    pico_unique_id
    hardware_pio
    hardware_dma
    hardware_clocks
    hardware_vreg
)

pico_set_binary_type(link_bench copy_to_ram)
pico_enable_stdio_usb(link_bench 1)
pico_enable_stdio_uart(link_bench 0)
pico_add_extra_outputs(link_bench)
