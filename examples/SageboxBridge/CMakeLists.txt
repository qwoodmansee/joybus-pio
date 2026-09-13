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

pico_add_extra_outputs(SageboxBridge)
