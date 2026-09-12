// GcDiag — answers "is the controller answering at all?" and, if not, WHICH
// PIN LOOKS ALIVE.
//
// Core 1 is a free-running edge counter on GP0..GP7. Core 0 reads every pin's
// idle level before the PIO claims GP1, fires raw PROBE (0x00) commands and
// reports edges-per-probe on every pin, then runs the real driver rate-limited
// with Poll()'s return value shown and a per-second pin summary.
//
// Lines the dashboard parses:
//   IDLE  0:H 1:H 2:L ...             idle level per pin, before any transmit
//   probe n/8: N byte(s) back ..      raw probe result
//   EDGES 0:0 1:18 2:0 ...            edges per pin during that probe
//   [OK|NONE ok=.. fail=..] raw ..    driver poll result
//   PINS L 0:H 1:H .. | E 0:0 1:512 ..  levels now | edges in the last second
//
// Serial input: 'b' (or 'r') reboots the board so the boot burst runs again —
// the dashboard sends it when it connects after the burst has already gone by.

#include <hardware/clocks.h>
#include <hardware/gpio.h>
#include <hardware/pio.h>
#include <hardware/watchdog.h>
#include <pico/multicore.h>
#include <pico/stdlib.h>
#include <stdio.h>
#include <string.h>

#include "GamecubeController.hpp"
#include "gamecube_definitions.h"
#include "joybus.h"

static const uint PIN = 1;                    // GP1 = Pico physical pin 2
static const uint RX_TIMEOUT_US = 5 * 4 * 10; // same leniency the driver uses
static const uint NPINS = 8;                  // GP0..GP7 watched

// ---- core 1: count edges on GP0..GP7 as fast as it can loop ----
static volatile uint32_t edge_count[NPINS];

static void core1_sampler() {
    uint32_t prev = gpio_get_all() & 0xFFu;
    while (true) {
        uint32_t cur = gpio_get_all() & 0xFFu;
        uint32_t ch = cur ^ prev;
        if (ch) {
            for (uint i = 0; i < NPINS; i++)
                if (ch & (1u << i)) edge_count[i]++;
            prev = cur;
        }
    }
}

static void edges_reset() { for (uint i = 0; i < NPINS; i++) edge_count[i] = 0; }

static void print_levels(const char *tag) {
    uint32_t v = gpio_get_all();
    printf("%s", tag);
    for (uint i = 0; i < NPINS; i++) printf(" %u:%c", i, (v >> i) & 1u ? 'H' : 'L');
}

static void print_edges(const char *tag) {
    printf("%s", tag);
    for (uint i = 0; i < NPINS; i++) printf(" %u:%lu", i, (unsigned long)edge_count[i]);
}

int main(void) {
    set_sys_clock_khz(130'000, true);
    stdio_init_all();
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    // All watched pins as plain inputs with NO internal pulls, so an idle level
    // reflects what the wiring is doing, not the Pico.
    for (uint i = 0; i < NPINS; i++) {
        gpio_init(i);
        gpio_set_dir(i, GPIO_IN);
        gpio_disable_pulls(i);
    }

    sleep_ms(2500);   // let USB CDC enumerate so the banner isn't lost
    printf("\n=== GcDiag ===  data on GP%u (physical pin 2)  sysclk %lu Hz\n", PIN, clock_get_hz(clk_sys));
    printf("Expected wiring: GND->pin 3, DATA->pin 2 (GP1) + 1k to 3V3, 3.43V logic->pin 36 (3V3 OUT)\n");

    sleep_ms(20);
    print_levels("IDLE"); printf("\n\n");

    multicore_launch_core1(core1_sampler);

    // ---------------- Phase 1: raw probes, with edges per pin ----------------
    joybus_port_t port;
    joybus_port_init(&port, PIN, pio0, -1, -1);
    uint answered = 0;
    for (int i = 1; i <= 8; i++) {
        uint8_t cmd = (uint8_t)GamecubeCommand::PROBE;
        uint8_t rx[8] = {0};
        edges_reset();
        joybus_send_bytes(&port, &cmd, 1);
        uint n = joybus_receive_bytes(&port, rx, 3, RX_TIMEOUT_US, true);
        sleep_us(300);   // let any straggling edges land before we snapshot
        printf("probe %d/8: %u byte(s) back", i, n);
        for (uint k = 0; k < n; k++) printf(" %02x", rx[k]);
        if (n == 3) {
            uint16_t dev = (uint16_t)(rx[0] << 8 | rx[1]);
            printf("   -> device 0x%04x status 0x%02x  CONTROLLER ANSWERED\n", dev, rx[2]);
            answered++;
        } else {
            printf("   -> NO ANSWER\n");
        }
        print_edges("EDGES"); printf("\n");
        gpio_put(PICO_DEFAULT_LED_PIN, i & 1);
        sleep_ms(250);
    }
    joybus_port_terminate(&port);

    if (answered == 0) {
        printf("\nRESULT: nothing on the wire. The controller never answered a probe.\n\n");
    } else {
        printf("\nRESULT: controller answered %u/8 probes. Wiring is fine; running the driver.\n\n", answered);
    }

    // ---------------- Phase 2: real driver, rate-limited, plus pin summary ----------------
    GamecubeController gcc(PIN, 120, pio0);
    gc_report_t report = default_gc_report, last = default_gc_report;
    absolute_time_t next_print = get_absolute_time();
    absolute_time_t next_pins = make_timeout_time_ms(1000);
    uint ok = 0, fail = 0;
    bool led = false;
    edges_reset();

    while (true) {
        int ch = getchar_timeout_us(0);
        if (ch == 'b' || ch == 'r') {
            printf("REBOOT: rerunning the boot burst\n");
            sleep_ms(50);
            watchdog_reboot(0, 0, 0);
        }
        bool responded = gcc.Poll(&report, false);
        if (responded) ok++; else fail++;

        bool changed = memcmp(&report, &last, sizeof(report)) != 0;
        if (changed || time_reached(next_print)) {
            next_print = make_timeout_time_ms(1000);
            const uint8_t *b = (const uint8_t *)&report;
            printf("[%s ok=%u fail=%u] raw %02x %02x %02x %02x %02x %02x %02x %02x\n",
                   responded ? "OK  " : "NONE", ok, fail,
                   b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]);
            last = report;
            led = !led;
            gpio_put(PICO_DEFAULT_LED_PIN, led);
        }
        if (time_reached(next_pins)) {
            next_pins = make_timeout_time_ms(1000);
            print_levels("PINS L"); printf(" |"); print_edges(" E"); printf("\n");
            edges_reset();
        }
    }
}
