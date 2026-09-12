// SageboxBridge — the Pico half of the SageRaces ↔ Sagebox integration.
//
// Polls a GameCube controller on GP1 and an N64 controller on GP4, frames every
// poll response per `imports/shared/integrations/sagebox/PROTOCOL.md` in the
// SageRaces monorepo, and writes the frames to USB CDC for the Pi's
// joybus-bridge to relay. Commands come back over the same pipe.
//
// The frames are RAW: the payload is byte-for-byte what the controller
// answered, and no interpretation happens here. That is deliberate — a
// decoding bug in SageRaces is a deploy away from fixed, one in here is a
// reflash of a board sealed inside a box under someone's TV.
//
// Core split. Core 1 owns the ENTIRE USB stack: it calls stdio_init_all(), it
// writes frames, it reads commands. Core 0 never touches stdio. That keeps the
// TinyUSB async context on one core, so a 500 ms blocked write can never stall
// the Joybus timing, and the two cores share exactly one thing: a lock-free
// ring of poll results.
//
// GP6 is the bypass switch. LOW means the DPDT switch has routed the controller
// straight to the console and the Pico is out of circuit; the firmware must
// then LISTEN ONLY and never drive the data lines. The pin is pulled down
// internally so an unwired or broken switch reads as bypassed — the safe
// direction, since the failure is "no input capture" rather than "two things
// driving one line".
//
// Build:  cmake --build examples/build --target SageboxBridge
// Flash:  picotool load -f -x examples/build/SageboxBridge/SageboxBridge.uf2
// Test:   examples/SageboxBridge/test/run-host-tests.sh   (framing, on the host)

#include <hardware/clocks.h>
#include <hardware/gpio.h>
#include <hardware/pio.h>
#include <hardware/sync.h>
#include <pico/multicore.h>
#include <pico/stdlib.h>

#include <new>
#include <string.h>

#include "GamecubeController.hpp"
#include "N64Controller.hpp"
#include "gamecube_definitions.h"
#include "n64_definitions.h"
#include "sagebox_framing.h"

// ---------------------------------------------------------------------------
// Wiring
// ---------------------------------------------------------------------------

/** GameCube controller DATA, with its 1 kΩ pull-up to 3V3. */
static constexpr uint PIN_GC_DATA = 1;
/** N64 controller DATA, with its 1 kΩ pull-up to 3V3. */
static constexpr uint PIN_N64_DATA = 4;
/** Bypass sense. LOW = bypassed = do not drive anything. */
static constexpr uint PIN_BYPASS = 6;

// GP2 (GameCube) and GP5 (N64) are the console-side taps, wired through 100 Ω
// and NOT used yet. Left untouched here on purpose: claiming them would put the
// Pico on the console side of a link it cannot yet speak for.

/** Port ids on the wire. Fixed, and mirrored by the bridge's port→console map. */
static constexpr uint8_t PORT_GC = 0;
static constexpr uint8_t PORT_N64 = 1;

/**
 * Per-controller poll rate. With both ports populated the achieved rate lands a
 * little under this: the two Joybus transactions are serialised on core 0 (a
 * GameCube poll is ~350 µs on the wire, an N64 poll ~160 µs), so the cooldowns
 * interleave rather than running in parallel.
 */
static constexpr uint POLL_HZ = 1000;

/** Firmware version, reported in the status reply. */
static constexpr uint8_t FW_MAJOR = 0;
static constexpr uint8_t FW_MINOR = 1;
static constexpr uint8_t FW_PATCH = 0;

// ---------------------------------------------------------------------------
// Cross-core state
// ---------------------------------------------------------------------------

/** One captured poll, on its way from core 0 to core 1. */
struct PollRecord {
    uint32_t t_us;
    uint16_t seq;
    uint8_t port;
    uint8_t len;
    uint8_t payload[8];
};

/** Power of two so the wrap is a mask. 256 records ≈ 128 ms of headroom. */
static constexpr uint32_t RING_CAPACITY = 256;
static PollRecord g_ring[RING_CAPACITY];
/** Written by core 0 only. */
static volatile uint32_t g_ring_head = 0;
/** Written by core 1 only. */
static volatile uint32_t g_ring_tail = 0;
/** Frames the ring could not hold. Their sequence numbers were already spent,
 *  so the loss shows up downstream as a gap — which is the point. */
static volatile uint32_t g_ring_overruns = 0;

/** Observed by core 0, reported by core 1. Single writer each, so plain volatile. */
static volatile bool g_gc_present = false;
static volatile bool g_n64_present = false;
static volatile bool g_bypassed = true;

/** Set by a SET_PROFILE command on core 1. Stored and echoed only — see below. */
static volatile uint8_t g_profile = SAGEBOX_PROFILE_PASSTHROUGH;

static bool ring_push(const PollRecord &record) {
    const uint32_t head = g_ring_head;
    const uint32_t next = (head + 1) & (RING_CAPACITY - 1);
    if (next == g_ring_tail) {
        g_ring_overruns++;
        return false;
    }
    g_ring[head] = record;
    // Publish the record before the index that makes it visible, or core 1 can
    // read a half-written entry.
    __dmb();
    g_ring_head = next;
    return true;
}

static bool ring_pop(PollRecord *out) {
    const uint32_t tail = g_ring_tail;
    if (tail == g_ring_head) return false;
    __dmb();
    *out = g_ring[tail];
    __dmb();
    g_ring_tail = (tail + 1) & (RING_CAPACITY - 1);
    return true;
}

// ---------------------------------------------------------------------------
// Core 1 — framing, USB writes, commands
// ---------------------------------------------------------------------------

/** Largest frame on the wire: header plus a GameCube payload. */
static constexpr size_t MAX_FRAME_BYTES = SAGEBOX_FRAME_HEADER_BYTES + 8;
/** Batch several frames per USB write; at 2 kHz combined this is ~30 ms worth. */
static constexpr size_t TX_BUFFER_BYTES = 1024;

static void write_usb(const uint8_t *bytes, size_t len) {
    // Nothing attached: drop rather than block. The sequence numbers keep
    // advancing, so a bridge that connects mid-stream resynchronises and its
    // own tracker resets on connect (PROTOCOL.md, "Reconnect policy").
    //
    // `stdio_usb_connected()` is TinyUSB's `tud_cdc_connected()`, which is DTR.
    // The Pi asserts it explicitly when it opens the port (see
    // pi-sc64 packages/joybus-bridge/serial_link.py, open_serial_port) -- a
    // host that leaves DTR low gets a completely silent box that looks exactly
    // like dead firmware. The two halves of that pair must move together.
    if (!stdio_usb_connected()) return;
    stdio_put_string(reinterpret_cast<const char *>(bytes), static_cast<int>(len), false, false);
    stdio_flush();
}

/** Reply to a command, as an ordinary frame addressed to the control port. */
static void send_control_reply(const uint8_t *body, size_t len) {
    uint8_t frame[MAX_FRAME_BYTES + 8];
    // seq is meaningless on a control frame — the bridge routes on `port` and
    // never feeds these to a sequence tracker — and core 1 must not touch core
    // 0's counter. Zero, rather than a racy read.
    const size_t n = sagebox_frame_encode(frame, sizeof(frame), 0, time_us_32(),
                                          SAGEBOX_PORT_CONTROL, body, len);
    if (n > 0) write_usb(frame, n);
}

static void send_status_reply(uint8_t cmd, uint8_t status) {
    uint8_t flags = 0;
    if (g_gc_present) flags |= SAGEBOX_STATUS_FLAG_GC_PRESENT;
    if (g_n64_present) flags |= SAGEBOX_STATUS_FLAG_N64_PRESENT;
    if (g_bypassed) flags |= SAGEBOX_STATUS_FLAG_BYPASSED;

    const uint8_t body[8] = {
        cmd, status, g_profile, flags, 1 /* wire protocol version */,
        FW_MAJOR, FW_MINOR, FW_PATCH,
    };
    send_control_reply(body, sizeof(body));
}

static void handle_command(const sagebox_command_t &command) {
    switch (command.cmd) {
        case SAGEBOX_CMD_SET_PROFILE: {
            if (command.payload_len != 1 || !sagebox_profile_is_valid(command.payload[0])) {
                send_status_reply(SAGEBOX_CMD_SET_PROFILE, SAGEBOX_CMD_ERR_BAD_ARGUMENT);
                return;
            }
            // Stored and echoed, nothing more. No profile remaps anything yet,
            // so every one of them currently behaves as passthrough on the
            // wire. The reply reports what is ACTUALLY active, which is why the
            // Pi forwards this value rather than the value it asked for.
            g_profile = command.payload[0];
            send_status_reply(SAGEBOX_CMD_SET_PROFILE, SAGEBOX_CMD_OK);
            return;
        }
        case SAGEBOX_CMD_GET_STATUS:
            send_status_reply(SAGEBOX_CMD_GET_STATUS, SAGEBOX_CMD_OK);
            return;
        default:
            send_status_reply(command.cmd, SAGEBOX_CMD_ERR_UNKNOWN_COMMAND);
            return;
    }
}

static void core1_main() {
    // stdio is initialised HERE, on core 1, so the whole USB stack — the async
    // context, its IRQ, every read and every write — lives on one core.
    stdio_init_all();

    sagebox_command_reader_t reader;
    sagebox_command_reader_reset(&reader);

    uint8_t tx[TX_BUFFER_BYTES];

    while (true) {
        // Commands first: a profile change should not queue behind a full ring.
        for (int i = 0; i < 64; i++) {
            const int ch = getchar_timeout_us(0);
            if (ch < 0) break;
            sagebox_command_t command;
            if (sagebox_command_reader_push(&reader, static_cast<uint8_t>(ch), &command)) {
                handle_command(command);
            }
        }

        size_t used = 0;
        PollRecord record;
        while (used + MAX_FRAME_BYTES <= sizeof(tx) && ring_pop(&record)) {
            used += sagebox_frame_encode(tx + used, sizeof(tx) - used, record.seq, record.t_us,
                                         record.port, record.payload, record.len);
        }

        if (used > 0) {
            write_usb(tx, used);
        } else {
            // Idle. A hot spin here would fight core 0 for the bus for nothing;
            // 200 µs is a fifth of a poll period, so it costs no latency worth
            // measuring.
            sleep_us(200);
        }
    }
}

// ---------------------------------------------------------------------------
// Core 0 — Joybus polling
// ---------------------------------------------------------------------------

// GamecubeController and N64Controller both leave `_initialized` UNSET in their
// constructors (see src/GamecubeController.cpp — the member is declared but
// never assigned), so a stack- or heap-allocated one starts with whatever was
// in that byte. If it starts true the driver skips the probe/origin handshake
// and the first poll comes back as a failure for no reason.
//
// Constructing into static storage sidesteps it without touching the library:
// static storage is zero-initialised before any constructor runs, so
// `_initialized` is reliably false. Placement new, because these need the
// real constructor to claim their PIO state machines.
alignas(GamecubeController) static uint8_t g_gc_storage[sizeof(GamecubeController)];
alignas(N64Controller) static uint8_t g_n64_storage[sizeof(N64Controller)];

static_assert(sizeof(gc_report_t) == 8, "GameCube poll response must be 8 bytes");
static_assert(sizeof(n64_report_t) == 4, "N64 poll response must be 4 bytes");

/** Core 0's sequence counter. One per EMITTED frame; wraps at 16 bits. */
static uint16_t g_next_seq = 0;

/**
 * Queue one poll response.
 *
 * The sequence number is spent here, before the ring's capacity is checked, so
 * a frame lost to a full ring leaves a gap downstream. A frame never produced —
 * because no controller answered that port — spends nothing: an empty second
 * port would otherwise read as a permanent 50 % loss rate forever.
 */
static void publish(uint8_t port, const void *payload, uint8_t len) {
    PollRecord record;
    record.seq = g_next_seq++;
    record.t_us = time_us_32();
    record.port = port;
    record.len = len;
    memcpy(record.payload, payload, len);
    ring_push(record);
}

/**
 * Hand a data pin to the PIO, or take it back as a plain high-Z input.
 *
 * Parking is what honours the bypass switch. The controllers are never
 * destructed to achieve it: both ports share one copy of the PIO program, and
 * `joybus_port_terminate` removes that program unconditionally, so tearing both
 * down would remove it twice.
 */
static void set_pin_parked(uint pin, bool parked) {
    if (parked) {
        gpio_set_function(pin, GPIO_FUNC_SIO);
        gpio_set_dir(pin, GPIO_IN);
        gpio_disable_pulls(pin);
    } else {
        gpio_set_function(pin, GPIO_FUNC_PIO0);
    }
}

int main() {
    set_sys_clock_khz(130'000, true);

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    // Pulled down so an unwired switch reads LOW = bypassed = never drive.
    gpio_init(PIN_BYPASS);
    gpio_set_dir(PIN_BYPASS, GPIO_IN);
    gpio_pull_down(PIN_BYPASS);

    // Both ports on pio0, sharing one copy of the joybus program: the GameCube
    // port loads it, the N64 port is handed the same offset and only claims its
    // own state machine.
    auto *gc = new (g_gc_storage) GamecubeController(PIN_GC_DATA, POLL_HZ, pio0);
    auto *n64 = new (g_n64_storage) N64Controller(PIN_N64_DATA, POLL_HZ, pio0, -1, gc->GetOffset());

    // Construction only puts the pins in the PIO's RECEIVE state, which does not
    // drive them, so it is safe even if we boot bypassed. Park immediately if so.
    bool parked = !gpio_get(PIN_BYPASS);
    set_pin_parked(PIN_GC_DATA, parked);
    set_pin_parked(PIN_N64_DATA, parked);
    g_bypassed = parked;

    multicore_launch_core1(core1_main);

    absolute_time_t next_blink = make_timeout_time_ms(500);
    bool led = false;

    while (true) {
        const bool bypassed = !gpio_get(PIN_BYPASS);
        if (bypassed != parked) {
            set_pin_parked(PIN_GC_DATA, bypassed);
            set_pin_parked(PIN_N64_DATA, bypassed);
            parked = bypassed;
        }
        g_bypassed = bypassed;

        if (bypassed) {
            // Listening only. Nothing is polled, nothing is driven, and both
            // ports report absent rather than reporting stale state.
            g_gc_present = false;
            g_n64_present = false;
            sleep_us(1000);
        } else {
            gc_report_t gc_report;
            if (gc->Poll(&gc_report, false)) {
                g_gc_present = true;
                publish(PORT_GC, &gc_report, sizeof(gc_report));
            } else {
                // Poll's return value is the only thing separating "no
                // controller" from "every button released": the report buffer
                // reads as neutral either way. An absent port emits nothing.
                g_gc_present = false;
            }

            n64_report_t n64_report;
            if (n64->Poll(&n64_report, false)) {
                g_n64_present = true;
                publish(PORT_N64, &n64_report, sizeof(n64_report));
            } else {
                g_n64_present = false;
            }
        }

        if (time_reached(next_blink)) {
            next_blink = make_timeout_time_ms(500);
            led = !led;
            gpio_put(PICO_DEFAULT_LED_PIN, led);
        }
    }
}
