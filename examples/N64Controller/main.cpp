#include "../example_led.h"
#include "N64Controller.hpp"
#include "gamecube_definitions.h"

#include <hardware/clocks.h>
#include <hardware/pio.h>
#include <pico/stdlib.h>
#include <stdio.h>

void print_bytes(const char *prefix, uint8_t *bytes, uint len);

N64Controller *controller;

int main(void) {
    set_sys_clock_khz(130'000, true);

    stdio_init_all();

    // Sagebox carrier: N64 DATA is GP4 (J9), with its 1 kΩ pull-up to 3V3.
    // The stock example used GP1, which is the GameCube channel on this board.
    uint joybus_pin = 4;

    controller = new N64Controller(joybus_pin, 120, pio0);
    n64_report_t report = default_n64_report;

    // Set up LED
    bool led = true;
    example_led_init();

    // One line per poll, ~20 a second, with Poll()'s result: the stock example
    // threw it away, so "no controller" printed the same zeros as "all released".
    uint32_t ok = 0, missed = 0;
    while (true) {
        bool answered = controller->Poll(&report, 0);
        answered ? ok++ : missed++;

        const uint8_t *raw = (const uint8_t *)&report;
        printf("%s ok %lu missed %lu raw %02x %02x %02x %02x | %s%s%s%s%s%s%s%s%s%s%s%s%s%s| stick %4d %4d\n",
               answered ? "OK  " : "NONE", (unsigned long)ok, (unsigned long)missed,
               raw[0], raw[1], raw[2], raw[3],
               report.a ? "A " : "", report.b ? "B " : "", report.z ? "Z " : "",
               report.start ? "Start " : "", report.l ? "L " : "", report.r ? "R " : "",
               report.dpad_up ? "D^ " : "", report.dpad_down ? "Dv " : "",
               report.dpad_left ? "D< " : "", report.dpad_right ? "D> " : "",
               report.c_up ? "C^ " : "", report.c_down ? "Cv " : "",
               report.c_left ? "C< " : "", report.c_right ? "C> " : "",
               (int)(int8_t)report.stick_x, (int)(int8_t)report.stick_y);

        // Toggle LED
        led = !led;
        example_led_put(led);
        sleep_ms(50);
    }
}
