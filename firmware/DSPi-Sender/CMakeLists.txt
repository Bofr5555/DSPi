# DSPi alink sender (WP LK4, SYS2 section 4): a Pico 2 as a USB sound card that
# drives up to 8 alink outputs. Built instead of DSPi when the top-level
# option DSPI_ROLE_SENDER is ON:
#
#   cmake -S firmware -B <dir> -G Ninja -DPICO_PLATFORM=rp2350-arm-s -DPICO_BOARD=pico2 -DDSPI_ROLE_SENDER=ON
#   cmake --build <dir>        -> <dir>/DSPi-Sender/dspi_sender-rp2350.uf2
#
# -DDSPI_SENDER_BENCH=ON gives dspi_sender_bench-rp2350.uf2: alink 0 (GP2)
# sends the PRBS-7 test pattern from boot and SYNC is on GP21, so an alink
# bench RX board (firmware/alink_bench) checks it with no host on the sender.
#
# It shares DSPi's USB descriptors (usb_descriptors.c with DSPI_ROLE_SENDER:
# two more alts), feedback controller, soft VCXO, flash clock helpers and
# the alink codec (firmware/DSPi/alink) and LK2's PIO programs; its own
# sources are the encoder core, the engine, the UAC1 driver and the vendor
# requests (sender_vc.c: both envelopes, page 0 and the paged 1.1.7 stack).  RP2350 only.

if (NOT PICO_PLATFORM MATCHES "^rp2350")
    message(FATAL_ERROR "DSPI_ROLE_SENDER needs PICO_PLATFORM=rp2350-arm-s: the alink is RP2350-only")
endif()

set(DSPI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../DSPi)

option(DSPI_SENDER_BENCH "Sender bench build: alink 0 sends PRBS-7 from boot, SYNC on GP21" OFF)
if (DSPI_SENDER_BENCH)
    set(SENDER_NAME dspi_sender_bench-rp2350)
else()
    set(SENDER_NAME dspi_sender-rp2350)
endif()

add_executable(DSPi-Sender
    sender_main.c
    sender_tx.c
    sender_usb.c
    sender_vendor.c
    sender_vc.c
    sender_store.c
    sender_alink.c
    sender_fanout.c
    ${DSPI_DIR}/usb_descriptors.c
    ${DSPI_DIR}/usb_feedback_controller.c
    ${DSPI_DIR}/soft_vcxo.c
    ${DSPI_DIR}/flash_clkdiv.c
    ${DSPI_DIR}/alink/alink_codec.c
    ${DSPI_DIR}/alink/alink_decoder.c
)
set_target_properties(DSPi-Sender PROPERTIES OUTPUT_NAME ${SENDER_NAME})
target_compile_options(DSPi-Sender PRIVATE -O2 -Wall -Wextra)
set_source_files_properties(sender_alink.c sender_fanout.c ${DSPI_DIR}/alink/alink_codec.c
    PROPERTIES COMPILE_FLAGS "-O3")

foreach(p alink_tx_20 alink_tx_10 alink_tx_5 alink_rx_20 alink_rx_10 alink_rx_5)
    pico_generate_pio_header(DSPi-Sender ${DSPI_DIR}/${p}.pio)
endforeach()

target_include_directories(DSPi-Sender PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${DSPI_DIR}
    ${DSPI_DIR}/alink
    ${CMAKE_CURRENT_BINARY_DIR}/generated
)

target_compile_definitions(DSPi-Sender PRIVATE
    DSPI_ROLE_SENDER
    PICO_RP2040_USB_FAST_IRQ=1
    PICO_CORE1_STACK_SIZE=0x1000
)
if (DSPI_SENDER_BENCH)
    target_compile_definitions(DSPi-Sender PRIVATE DSPI_SENDER_BENCH)
endif()

# Git build stamp (REQ_GET_BUILD_INFO), as DSPi.
set(SENDER_BUILD_INFO ${CMAKE_CURRENT_BINARY_DIR}/generated/build_info.h)
add_custom_target(dspi_sender_build_info
    COMMAND ${CMAKE_COMMAND}
        -DREPO_DIR=${CMAKE_CURRENT_SOURCE_DIR}
        -DOUT_FILE=${SENDER_BUILD_INFO}
        -DRELEASE=OFF
        -DCONFIG_H=${DSPI_DIR}/config.h
        -DARTIFACT_DIR=${CMAKE_CURRENT_BINARY_DIR}
        -P ${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/gen_build_info.cmake
    BYPRODUCTS ${SENDER_BUILD_INFO}
    COMMENT "Stamping git build info (sender)"
    VERBATIM)
add_dependencies(DSPi-Sender dspi_sender_build_info)

target_link_libraries(DSPi-Sender
    pico_stdlib
    pico_multicore
    pico_unique_id
    hardware_pio
    hardware_dma
    hardware_clocks
    hardware_vreg
    hardware_pwm
    hardware_flash
    tinyusb_device
)

# The whole image runs from RAM: core 1 keeps sending while core 0 writes
# the settings sector.
pico_set_binary_type(DSPi-Sender copy_to_ram)
pico_enable_stdio_usb(DSPi-Sender 0)
pico_enable_stdio_uart(DSPi-Sender 0)
pico_add_extra_outputs(DSPi-Sender)
