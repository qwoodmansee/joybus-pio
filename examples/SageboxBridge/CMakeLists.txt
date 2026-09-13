add_executable(
  SageboxBridge
  main.cpp
  sagebox_framing.c
  sagebox_routing.c
  ../../src/joybus.c
  ../../src/GamecubeController.cpp
  ../../src/N64Controller.cpp
)

pico_generate_pio_header(SageboxBridge ${CMAKE_CURRENT_LIST_DIR}/../../src/joybus.pio)

target_link_libraries(SageboxBridge pico_stdlib hardware_pio pico_multicore)

# USB CDC is the link to the Pi: it carries the frames, and pico_stdio_usb also
# brings the reset interface `picotool load -f -x` uses to reflash the board
# over the same cable. UART stays off — GP16/17 are reserved for an optional
# backup link that nothing uses yet.
pico_enable_stdio_usb(SageboxBridge 1)
pico_enable_stdio_uart(SageboxBridge 0)

# The stream is BINARY. stdio's default CRLF translation would rewrite every
# 0x0A in a frame — a stick at that position, a button mask, a timestamp byte —
# as 0x0D 0x0A and desynchronise the reader. The frame writer passes
# cr_translation=false explicitly; this makes the default match, so a stray
# printf added later cannot reintroduce it.
target_compile_definitions(SageboxBridge PRIVATE PICO_STDIO_DEFAULT_CRLF=0)

# Core 1 owns the whole USB stack, so TinyUSB's device init AND the USB and
# alarm IRQ handlers all run on core 1's stack. The 2048-byte default was not
# enough once the frame batching buffer and the command reader sat on it: the
# overflow landed inside USB bring-up and the board stopped enumerating
# entirely, which reads as dead hardware rather than as a crash.
#
# Those buffers are static now (see g_tx and friends in main.cpp) so this is
# headroom rather than the fix. Both together, because either alone silently
# re-opens the same hole the next time a buffer is added.
target_compile_definitions(SageboxBridge PRIVATE PICO_CORE1_STACK_SIZE=4096)

# Boot bisect. 3 is the real firmware; see SAGEBOX_BOOT_STAGE in main.cpp for
# what each lower value amputates. Build all four with ./build-boot-stages.sh.
# A definition, not a #define in main.cpp, because PICO_CORE1_STACK_SIZE next to
# it MUST be a definition — multicore.c is compiled into this target and would
# never see a #define made in our own source.
set(SAGEBOX_BOOT_STAGE "3" CACHE STRING "SageboxBridge boot bisect stage: 0 USB only, 1 +front ports, 2 +console ports claimed, 3 full")
target_compile_definitions(SageboxBridge PRIVATE SAGEBOX_BOOT_STAGE=${SAGEBOX_BOOT_STAGE})

pico_add_extra_outputs(SageboxBridge)
