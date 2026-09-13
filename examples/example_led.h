// A blink that compiles on boards with no LED.
//
// Every example here blinked PICO_DEFAULT_LED_PIN, which the Pico 2 W does not
// define at all: its LED hangs off the CYW43, and GP25 is that chip's select
// line. Building these for a W therefore failed outright, and building a W as
// plain pico2 was worse — it compiled, toggled a wireless chip select, lit
// nothing, and left the LED looking like a boot signal when it was not one.
//
// Guarded once here rather than at fifteen call sites, so that `led` stays
// referenced on boards without a pin and no example picks up an
// unused-variable warning for a light it cannot turn on.
#ifndef JOYBUS_EXAMPLE_LED_H
#define JOYBUS_EXAMPLE_LED_H

#include <hardware/gpio.h>

#ifdef PICO_DEFAULT_LED_PIN

static inline void example_led_init(void) {
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
}

static inline void example_led_put(bool on) { gpio_put(PICO_DEFAULT_LED_PIN, on); }

#else

static inline void example_led_init(void) {}

static inline void example_led_put(bool on) { (void)on; }

#endif

#endif
