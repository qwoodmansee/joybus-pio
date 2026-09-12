add_executable(
  GcDiag
  main.cpp
  ../../src/joybus.c
  ../../src/GamecubeController.cpp
)

pico_generate_pio_header(GcDiag ${CMAKE_CURRENT_LIST_DIR}/../../src/joybus.pio)

target_link_libraries(GcDiag pico_stdlib hardware_pio hardware_watchdog pico_multicore)

pico_enable_stdio_usb(GcDiag 1)
pico_enable_stdio_uart(GcDiag 0)

pico_add_extra_outputs(GcDiag)
