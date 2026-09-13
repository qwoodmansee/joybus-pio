#include "../example_led.h"
#include "N64Console.hpp"
#include "n64_definitions.h"

// set_sys_clock_khz lives here. This example never included it and so has not
// built for some time; nothing noticed because only the two Sagebox targets
// were ever compiled out of this directory.
#include <hardware/clocks.h>
#include <hardware/pio.h>
#include <pico/stdlib.h>

N64Console *console;

int main(void) {
    set_sys_clock_khz(130'000, true);

    uint joybus_pin = 1;

    console = new N64Console(joybus_pin, pio0);
    n64_report_t report = default_n64_report;

    // Set up LED
    bool led = true;
    example_led_init();

    while (true) {
        console->WaitForPoll();
        console->SendReport(&report);

        // Toggle LED
        led = !led;
        example_led_put(led);
    }
}
