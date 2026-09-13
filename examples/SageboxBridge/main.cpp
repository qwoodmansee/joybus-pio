// SageboxBridge — the Pico half of the SageRaces ↔ Sagebox integration.
//
// The box has four controller ports on the front (INPUTS) and four console
// cables out the back (OUTPUTS). Two of each are wired today: a GameCube pair
// and an N64 pair. Joybus is point to point, so a console cable sees exactly
// one controller — which means the Pico has to BE that controller. It polls the
// front ports, composes one report per output from whichever inputs are routed
// there, and answers each console's polls with the result. Everything the
// routing matrix does happens here; nothing downstream can route anything.
//
// It also frames every front-port poll per
// `imports/shared/integrations/sagebox/PROTOCOL.md` in the SageRaces monorepo
// and writes the frames to USB CDC for the Pi's joybus-bridge to relay.
// Commands — including the routing commands — come back over the same pipe.
//
// Those frames are RAW: the payload is byte-for-byte what the controller
// answered, and no interpretation happens here. That is deliberate — a decoding
// bug in SageRaces is a deploy away from fixed, one in here is a reflash of a
// board sealed inside a box under someone's TV. The routing matrix is the one
// place the firmware does interpret a report, and every rule it applies lives in
// sagebox_routing.c, which has no pico-sdk dependency and is checked on the host.
//
// Core split. CORE 0 owns the ENTIRE USB stack: it calls stdio_init_all() as
// the first thing main() does, it writes frames, it reads commands. CORE 1 owns
// every Joybus transaction and never touches stdio. The two share exactly three
// things: a lock-free ring of poll results with core 1 as producer, a handful of
// single-writer status flags, and the routing handoff below.
//
// Which core gets USB is NOT arbitrary, and the obvious arrangement is the
// broken one. Bringing the USB stack up on core 1 produces a board that does
// not enumerate at all: no serial device, no way to read a fault out of it,
// indistinguishable from dead hardware, recoverable only with the BOOTSEL
// button. It was bisected on the bench with staged builds — stdio on core 0
// enumerates with an idle core 1 and at 130 MHz, and every build that called
// stdio_init_all() from core 1 was silent, including one written before any of
// the routing work existed. Do not "tidy" this back the other way.
//
// A blocked USB write can therefore stall core 0 for up to half a second, which
// costs frame freshness to the Pi and nothing else. Joybus timing is on core 1
// and cannot be touched by it.
//
// Scheduling on core 1. A console's poll is the only deadline the Pico does not
// choose: miss it and the game drops a frame of input. A front-port poll is a
// deadline the Pico does choose, and a late one only makes a report slightly
// stale. So the loop checks the console-facing ports for pending work between
// every other step, and lets front-port polls slip when the consoles are busy.
//
// GP6 is the bypass switch, and it is GAMECUBE ONLY. LOW means the DPDT switch
// has wired the front GameCube port straight to the GameCube console and the
// Pico is out of that circuit; the firmware must then LISTEN ONLY and never
// drive GP1 or GP2. The pin is pulled down internally so an unwired or broken
// switch reads as bypassed — the safe direction, since the failure is "no input
// capture" rather than "two things driving one line". The N64 pair is on the
// other side of that switch and keeps running.
//
// Build:  cmake --build examples/build --target SageboxBridge
// Flash:  picotool load -f -x examples/build/SageboxBridge/SageboxBridge.uf2
// Test:   examples/SageboxBridge/test/run-host-tests.sh   (framing + routing)

#include <hardware/clocks.h>
#include <hardware/gpio.h>
#include <hardware/pio.h>
#include <hardware/sync.h>
#include <pico/multicore.h>
#include <pico/stdlib.h>

#include <new>
#include <stdio.h>
#include <string.h>

#include "GamecubeController.hpp"
#include "N64Controller.hpp"
#include "gamecube_definitions.h"
#include "joybus.h"
// For joybus_program (the instruction-space check), joybus_program_send_init
// (the bounded send) and NUM_PIO_STATE_MACHINES.
#include "joybus.pio.h"
#include "n64_definitions.h"
#include "sagebox_framing.h"
#include "sagebox_routing.h"

// ---------------------------------------------------------------------------
// Wiring
// ---------------------------------------------------------------------------

/** INPUT 0 — GameCube front port DATA, with its 1 kΩ pull-up to 3V3. */
static constexpr uint PIN_IN_GC = 1;
/** INPUT 1 — N64 front port DATA, with its 1 kΩ pull-up to 3V3. */
static constexpr uint PIN_IN_N64 = 4;
/** OUTPUT 0 — GameCube console cable, through 100 Ω. */
static constexpr uint PIN_OUT_GC = 2;
/** OUTPUT 1 — N64 console cable, through 100 Ω. */
static constexpr uint PIN_OUT_N64 = 5;
/** Bypass sense, GameCube side only. LOW = bypassed = do not drive GP1 or GP2. */
static constexpr uint PIN_BYPASS = 6;

// Only the console pulls the console-side lines up. The Pico contributes no
// pull of its own on GP2 or GP5, and the 100 Ω in series is what keeps a
// moment of contention from being a short.

/** Port ids on the wire. Fixed, and mirrored by the bridge's port→console map. */
static constexpr uint8_t PORT_GC = 0;
static constexpr uint8_t PORT_N64 = 1;

/**
 * Boot bisect stage. 3 is the real firmware; lower values amputate the board.
 *
 * This exists because a firmware that does not enumerate cannot tell you why.
 * The only instrument left is which build comes up, so the boot path is cut
 * into four and flashed one at a time:
 *
 *   0  core 1, USB and the LED. No PIO touched at all.
 *   1  + the two front ports, polled and framed. What a10dce2 did.
 *   2  + the two console ports claimed and their program loaded, never answered.
 *   3  + answering the consoles. The whole firmware.
 *
 * The LED blinks at every stage, from core 0, so a dark board means core 0
 * never reached its loop and a blinking board with no USB means core 1 did not
 * survive. Build all four with ./build-boot-stages.sh.
 */
#ifndef SAGEBOX_BOOT_STAGE
#define SAGEBOX_BOOT_STAGE 3
#endif

/** Stage 3 only. Lower stages claim the ports but never answer a console. */
static constexpr bool DEVICE_SIDE_ANSWERS = SAGEBOX_BOOT_STAGE >= 3;

/**
 * How often core 0 tries to poll each front port.
 *
 * This is a target, not a guarantee. A console poll always goes first, so under
 * load the achieved rate drops — which is the correct trade: front-port frames
 * are telemetry for SageRaces, console answers are the thing a runner feels.
 */
static constexpr uint32_t INPUT_POLL_PERIOD_US = 1000;

/**
 * The poll rate handed to the library's controller classes.
 *
 * The library's period is ALSO the gap it leaves between the probe and the
 * first poll inside `_init()`, and between consecutive polls. At 8 kHz that gap
 * was 125 µs and a real N64 controller never answered the origin poll, so
 * `Poll()` reported "absent" forever (bench, 2026-09-13: the N64Controller
 * example at 1 kHz saw the same controller fine). 1 kHz matches the example
 * and our own cadence, so the cooldown has usually expired by the time we call;
 * the residual busy-wait is bounded by INPUT_POLL_PERIOD_US.
 */
static constexpr uint LIBRARY_POLL_HZ = 1000;

/**
 * How long to leave an empty front port alone after a failed poll.
 *
 * A port with nothing plugged in costs a probe plus a receive timeout every
 * time it is tried, and that time is taken directly out of the consoles'
 * budget. 20 ms is short enough that plugging a controller in feels instant and
 * long enough that an empty port is nearly free.
 */
static constexpr uint32_t ABSENT_INPUT_RETRY_US = 20'000;

/** Firmware version, reported in the status reply. */
// Defined in sagebox_framing.h with the rest of the wire contract, because
// readers gate on it. 0.3.0 is the first firmware that can report console
// links; see the note beside the flags enum for why that bit needed a version
// when the earlier ones did not.
static constexpr uint8_t FW_MAJOR = SAGEBOX_FW_MAJOR;
static constexpr uint8_t FW_MINOR = SAGEBOX_FW_MINOR;
static constexpr uint8_t FW_PATCH = SAGEBOX_FW_PATCH;

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
/** Written by core 1, the Joybus core, only. */
static volatile uint32_t g_ring_head = 0;
/** Written by core 0, the USB core, only. */
static volatile uint32_t g_ring_tail = 0;
/** Frames the ring could not hold. Their sequence numbers were already spent,
 *  so the loss shows up downstream as a gap — which is the point. */
static volatile uint32_t g_ring_overruns = 0;

/** Observed by core 0, reported by core 1. Single writer each, so plain volatile. */
static volatile bool g_gc_present = false;
static volatile bool g_n64_present = false;
static volatile bool g_bypassed = true;

/** Set by core 0 if any joybus port could not claim what it needed. */
static volatile bool g_port_fault = false;

/**
 * Whether a powered console is on each output, published by core 1.
 *
 * Single writer, read by core 0 only for the status reply, so the worst a race
 * can do is report the previous answer one reply late.
 */
static volatile bool g_gc_console_link = false;
static volatile bool g_n64_console_link = false;

/** Set by a SET_PROFILE command on core 1. Stored and echoed only — see below. */
static volatile uint8_t g_profile = SAGEBOX_PROFILE_PASSTHROUGH;

/**
 * Whether this build actually rewrites sticks for a profile. It does not.
 *
 * `g_profile` is stored and echoed and read by nothing else, so oot-ess and
 * sm64 are byte-identical to passthrough on the wire. This constant is what
 * says so out loud, on the wire, in the status reply's flags byte — so that
 * SageRaces gates its "your box will rewrite stick values" copy on what the
 * BOX reports rather than on an assumption about which firmware is flashed.
 *
 * Flip it to true in the same commit that lands a real remap, never before and
 * never separately. A box claiming an assist it does not apply is worse than
 * one honestly claiming none: the first is trusted.
 */
static constexpr bool PROFILE_REMAP_IMPLEMENTED = false;

/** Which ports exist and as what. Fixed in firmware; reported by GET_PORTS. */
static sagebox_ports_t g_ports;

/**
 * The routing matrix. Core 0's alone: it validates a SET_ROUTING, applies it,
 * and reads it back when composing. Core 1 never sees routing at all — it
 * answers consoles with whatever report core 0 last published, which is what
 * lets its loop stay short enough to answer inside a console's reply window.
 *
 * Still DOUBLE BUFFERED despite being single-core, because the matrix is 20
 * bytes and `send_routing_reply` reads it from inside a command handler. A
 * structure that wide written in place under a reader tears whoever does it.
 * Writing the buffer the index does not point at and flipping afterwards means
 * a reader that latched the index is copying something nobody is touching.
 */
static sagebox_routing_t g_routing_staged;
static sagebox_routing_t g_routing_buffers[2];
static volatile uint8_t g_routing_index = 0;

static void apply_pending_routing_now();

/** Snapshot the matrix the polling loop is running right now. */
static void read_active_routing(sagebox_routing_t *out) {
    const uint8_t index = g_routing_index;
    __dmb();
    *out = g_routing_buffers[index];
}

/**
 * One output's next report, handed from core 0 to core 1.
 *
 * Double buffered and flipped by an index, because core 1 must be able to grab
 * a report in the microseconds between a console's last command bit and its
 * reply, and it cannot wait on a lock to do it. Core 0 fills the buffer the
 * index does NOT point at and flips afterwards, so core 1 always copies a slot
 * nobody is writing. `len` lives inside the slot rather than beside it, so the
 * flip publishes the report and its length as one indivisible fact — a report
 * whose length arrived separately could be read as eight bytes of a four-byte
 * answer.
 *
 * len == 0 means nothing is routed here and the console must be answered with
 * silence, not with a neutral report.
 */
struct OutputSlot {
    uint8_t report[SAGEBOX_MAX_REPORT_BYTES];
    uint8_t len;
};

struct OutputHandoff {
    OutputSlot slot[2];
    /** Written by core 0 only, read by core 1 only. */
    volatile uint8_t index;
};

static OutputHandoff g_output_handoff[2];

/** Core 0: publish an output's freshly composed report. */
static void publish_output_report(uint8_t output, const uint8_t *report, size_t len) {
    OutputHandoff &handoff = g_output_handoff[output];
    const uint8_t next = static_cast<uint8_t>(handoff.index ^ 1u);
    OutputSlot &slot = handoff.slot[next];

    if (len > 0) memcpy(slot.report, report, len);
    slot.len = static_cast<uint8_t>(len);
    // The slot must be whole before the index makes it visible.
    __dmb();
    handoff.index = next;
}

/** Core 1: take the newest report for an output. Returns bytes, 0 for silence. */
static size_t latch_output_report(uint8_t output, uint8_t *out) {
    const OutputHandoff &handoff = g_output_handoff[output];
    const uint8_t index = handoff.index;
    __dmb();
    const OutputSlot &slot = handoff.slot[index];
    const size_t len = slot.len;
    if (len > 0 && len <= SAGEBOX_MAX_REPORT_BYTES) memcpy(out, slot.report, len);
    return len;
}

static bool ring_push(const PollRecord &record) {
    const uint32_t head = g_ring_head;
    const uint32_t next = (head + 1) & (RING_CAPACITY - 1);
    if (next == g_ring_tail) {
        g_ring_overruns++;
        return false;
    }
    g_ring[head] = record;
    // Publish the record before the index that makes it visible, or core 0 can
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
/** GET_ROUTING is the widest reply: cmd, status, bypassed, then the matrix. */
static constexpr size_t MAX_CONTROL_BODY_BYTES = 3 + SAGEBOX_ROUTING_PAYLOAD_BYTES;
/** Batch several frames per USB write; at 2 kHz combined this is ~30 ms worth. */
static constexpr size_t TX_BUFFER_BYTES = 1024;

/**
 * Core 1's working buffers, in STATIC storage rather than on its stack.
 *
 * This is not a style preference, it is the bug that bricked the board. Core 1
 * gets PICO_CORE1_STACK_SIZE, which defaults to 2048 bytes, and these three
 * came to 1620 of it — measured in the disassembly, `subw sp, sp, #1620` in
 * core1_main's prologue, plus 36 bytes of pushed registers. That frame is
 * allocated BEFORE stdio_init_all() is called, leaving under 400 bytes for
 * TinyUSB's device init and for the USB and alarm IRQ handlers, which run on
 * this same stack. The overflow happened inside USB bring-up, so the board
 * never enumerated at all and looked dead rather than crashed.
 *
 * Core 1 is single threaded and the only reader or writer of all three, so
 * static costs nothing and takes the buffers out of the stack budget entirely.
 * The stack size is raised in CMakeLists.txt as well: one of those two fixes
 * alone would work, and relying on either alone is how this comes back.
 */
static uint8_t g_tx[TX_BUFFER_BYTES];
static sagebox_command_reader_t g_command_reader;
static sagebox_command_t g_command;

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
    uint8_t frame[SAGEBOX_FRAME_HEADER_BYTES + MAX_CONTROL_BODY_BYTES];
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
    if (PROFILE_REMAP_IMPLEMENTED) flags |= SAGEBOX_STATUS_FLAG_PROFILE_REMAP;
    if (g_port_fault) flags |= SAGEBOX_STATUS_FLAG_PORT_FAULT;
    if (g_gc_console_link) flags |= SAGEBOX_STATUS_FLAG_GC_CONSOLE_LINK;
    if (g_n64_console_link) flags |= SAGEBOX_STATUS_FLAG_N64_CONSOLE_LINK;

    const uint8_t body[8] = {
        cmd, status, g_profile, flags, 1 /* wire protocol version */,
        FW_MAJOR, FW_MINOR, FW_PATCH,
    };
    send_control_reply(body, sizeof(body));
}

/**
 * `[cmd, status, bypassed, ...20 matrix bytes]`.
 *
 * Sent for GET_ROUTING, and again after every SET_ROUTING — including a
 * rejected one, so the Pi always ends a routing exchange knowing exactly what
 * the box is doing rather than having to infer it from a status code.
 */
static void send_routing_reply(uint8_t cmd, uint8_t status) {
    sagebox_routing_t active;
    read_active_routing(&active);

    uint8_t body[MAX_CONTROL_BODY_BYTES];
    body[0] = cmd;
    body[1] = status;
    body[2] = g_bypassed ? 1 : 0;
    sagebox_routing_encode(&active, body + 3);
    send_control_reply(body, sizeof(body));
}

/** `[cmd, status, in0..in3, out0..out3]`, kinds as sagebox_routing.h numbers them. */
static void send_ports_reply(uint8_t cmd, uint8_t status) {
    uint8_t body[2 + 2 * SAGEBOX_PORT_COUNT];
    body[0] = cmd;
    body[1] = status;
    for (uint8_t i = 0; i < SAGEBOX_PORT_COUNT; i++) {
        body[2 + i] = g_ports.input_kinds[i];
        body[2 + SAGEBOX_PORT_COUNT + i] = g_ports.output_kinds[i];
    }
    send_control_reply(body, sizeof(body));
}

static void handle_set_routing(const sagebox_command_t &command) {
    sagebox_routing_t requested;
    if (command.payload_len != SAGEBOX_ROUTING_PAYLOAD_BYTES ||
        !sagebox_routing_decode(command.payload, command.payload_len, &requested) ||
        !sagebox_routing_validate(&requested, &g_ports)) {
        // Routing is left exactly as it was. A partially applied matrix would
        // be a box whose front panel and back panel disagree, with no way for
        // the runner to tell which half took.
        send_status_reply(SAGEBOX_CMD_SET_ROUTING, SAGEBOX_CMD_ERR_BAD_ARGUMENT);
        send_routing_reply(SAGEBOX_CMD_GET_ROUTING, SAGEBOX_CMD_ERR_BAD_ARGUMENT);
        return;
    }

    // Applied immediately, because routing is now entirely core 0's: this
    // handler and the composition that reads the matrix run on the same core.
    // There used to be a staging slot, a generation counter and a bounded wait
    // for the other core to collect it — machinery that is not merely redundant
    // now but WRONG, since the apply step happens later in this same loop and
    // the wait could never be satisfied from inside the handler. Every
    // SET_ROUTING would have stalled ten milliseconds and then reported failure
    // for a matrix it had accepted.
    g_routing_staged = requested;
    apply_pending_routing_now();

    send_status_reply(SAGEBOX_CMD_SET_ROUTING, SAGEBOX_CMD_OK);
    send_routing_reply(SAGEBOX_CMD_GET_ROUTING, SAGEBOX_CMD_OK);
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
            // wire — and the status reply says so through
            // PROFILE_REMAP_IMPLEMENTED, which is the flag to flip in whatever
            // commit lands the rewrite. The reply reports what is ACTUALLY
            // active, which is why the Pi forwards this value rather than the
            // value it asked for.
            //
            // The profile is a stick rewrite applied AFTER merging, on outputs
            // of kind n64 only. It is deliberately not folded into the routing
            // matrix: one is which controller reaches which console, the other
            // is what that controller's stick means.
            g_profile = command.payload[0];
            send_status_reply(SAGEBOX_CMD_SET_PROFILE, SAGEBOX_CMD_OK);
            return;
        }
        case SAGEBOX_CMD_GET_STATUS:
            send_status_reply(SAGEBOX_CMD_GET_STATUS, SAGEBOX_CMD_OK);
            return;
        case SAGEBOX_CMD_SET_ROUTING:
            handle_set_routing(command);
            return;
        case SAGEBOX_CMD_GET_ROUTING:
            if (command.payload_len != 0) {
                send_routing_reply(SAGEBOX_CMD_GET_ROUTING, SAGEBOX_CMD_ERR_BAD_ARGUMENT);
                return;
            }
            send_routing_reply(SAGEBOX_CMD_GET_ROUTING, SAGEBOX_CMD_OK);
            return;
        case SAGEBOX_CMD_GET_PORTS:
            if (command.payload_len != 0) {
                send_ports_reply(SAGEBOX_CMD_GET_PORTS, SAGEBOX_CMD_ERR_BAD_ARGUMENT);
                return;
            }
            send_ports_reply(SAGEBOX_CMD_GET_PORTS, SAGEBOX_CMD_OK);
            return;
        default:
            send_status_reply(command.cmd, SAGEBOX_CMD_ERR_UNKNOWN_COMMAND);
            return;
    }
}

/**
 * Core 0's forever loop: read commands, answer them, drain frames to the Pi.
 *
 * stdio was initialised by main() before core 1 existed, so the whole USB stack
 * — the async context, its IRQ, every read and every write — lives on this
 * core, and a blocked write can stall nothing but itself.
 */
static void poll_front_ports_and_compose(bool bypassed);
static void park_gamecube_front_port(bool bypassed);

static void core0_loop() {
    sagebox_command_reader_reset(&g_command_reader);

    // Read here, on the core that owns GP1, and published for core 1 to act on
    // for GP2. One reader means the two pins can never disagree about which way
    // the lever is.
    bool bypassed = !gpio_get(PIN_BYPASS);
    g_bypassed = bypassed;
    park_gamecube_front_port(bypassed);

    while (true) {
        // Commands first: a routing change should not queue behind a full ring.
        for (int i = 0; i < 64; i++) {
            const int ch = getchar_timeout_us(0);
            if (ch < 0) break;
            if (sagebox_command_reader_push(&g_command_reader, static_cast<uint8_t>(ch),
                                            &g_command)) {
                handle_command(g_command);
            }
        }

        const bool now_bypassed = !gpio_get(PIN_BYPASS);
        if (now_bypassed != bypassed) {
            park_gamecube_front_port(now_bypassed);
            bypassed = now_bypassed;
            g_bypassed = bypassed;
        }

        poll_front_ports_and_compose(bypassed);

        size_t used = 0;
        PollRecord record;
        while (used + MAX_FRAME_BYTES <= sizeof(g_tx) && ring_pop(&record)) {
            used += sagebox_frame_encode(g_tx + used, sizeof(g_tx) - used, record.seq, record.t_us,
                                         record.port, record.payload, record.len);
        }

        // The ring is now single-core: this loop fills it and drains it. Kept
        // rather than written straight to the USB buffer because it still does
        // the job it was built for — letting a poll happen on schedule and a USB
        // write happen when there is a batch worth writing.
        if (used > 0) write_usb(g_tx, used);
    }
}

// ---------------------------------------------------------------------------
// Core 0 — the console-facing side
// ---------------------------------------------------------------------------

// The library HAS device-side classes — GamecubeConsole and N64Console in
// src/ — and this firmware cannot use them. Both only offer blocking entry
// points: `WaitForPoll` and `WaitForPollStart` call `joybus_receive_bytes` with
// `first_byte_can_timeout = false`, so they park the core until a console
// speaks, with no way to ask first whether one has. On a board that must also
// poll two front ports and answer a second console, a call that never returns
// is not a driver, it is a deadlock.
//
// So the device side is driven here, non-blockingly, straight off joybus.h: the
// PIO receive program is always running and shifts a console's command bytes
// into the RX FIFO whether or not the CPU is looking, so "has this console
// asked for anything?" is a FIFO check. The command handling below is the same
// as the two console classes', including their timing constants.

/** One bit on the wire, at the Joybus 250 kbit rate. */
static constexpr uint INCOMING_BIT_LENGTH_US = 5;
/** Gap we allow between bytes of one command before calling it a lost frame. */
static constexpr uint DEVICE_RECEIVE_TIMEOUT_US = INCOMING_BIT_LENGTH_US * 10;
/** Let the console finish its stop bit before answering. */
static constexpr uint64_t DEVICE_REPLY_DELAY_US = INCOMING_BIT_LENGTH_US - 1;
/** Long enough for the rest of a 3-byte GameCube command we cannot parse. */
static constexpr uint GC_RESET_WAIT_US = (INCOMING_BIT_LENGTH_US * 8) * 2 + DEVICE_RECEIVE_TIMEOUT_US;
/** N64 commands are one byte, so there is nothing left to wait out. */
static constexpr uint N64_RESET_WAIT_US = DEVICE_RECEIVE_TIMEOUT_US;
/**
 * How long to wait for an idle line before transmitting.
 *
 * `joybus_send_bytes` spins on the line going high with NO timeout, which turns
 * an unplugged console cable — a floating line with nothing pulling it up — into
 * a wedged box. Checking first bounds it.
 */
static constexpr uint LINE_IDLE_TIMEOUT_US = 200;

/** An output: the console-facing port and the report the next poll will get. */
/**
 * How long a console line must sit CONTINUOUSLY low before we call the console
 * gone.
 *
 * It cannot be any low at all: every data bit pulls the line low, a zero for
 * three microseconds of its four. Only a low that outlasts any possible traffic
 * means nothing is powering the line. One millisecond is two orders of
 * magnitude past the longest legitimate low and still imperceptible to a runner
 * switching a console on.
 */
static constexpr uint32_t CONSOLE_ABSENT_US = 1000;

/** How long the line must sit high before we trust it again. */
static constexpr uint32_t CONSOLE_SETTLE_US = 1000;

struct DevicePort {
    joybus_port_t port;
    uint8_t kind;
    /** False when this port never got a PIO state machine. See g_port_fault. */
    bool ready;
    /** True while the bypass switch has taken this pin away from us. */
    bool parked;

    /**
     * Whether a powered console is on the other end.
     *
     * The Pico now boots BEFORE the consoles do — it lives on the Pi and comes
     * up with it, while a console is switched on whenever someone feels like
     * playing. At boot those lines sit at 0 V, and a receive state machine
     * staring at a dead line is not idle: it reads the permanent low as a start
     * bit and shifts garbage. Nothing resynchronises it later, so the console
     * powers up, starts polling, and is answered by a state machine still
     * chewing on noise from before it existed.
     *
     * This was invisible on the bench because every test there warm-rebooted
     * the Pico while the consoles were already on, which is the one order that
     * cannot reproduce it.
     */
    bool link_up;
    /** Current line level and when it last changed, for the timing above. */
    bool line_high;
    absolute_time_t level_since;
};

static DevicePort g_out_gc;
static DevicePort g_out_n64;

static bool wait_line_idle(uint pin) {
    const absolute_time_t deadline = make_timeout_time_us(LINE_IDLE_TIMEOUT_US);
    while (!gpio_get(pin)) {
        if (time_reached(deadline)) return false;
    }
    return true;
}

/** Largest thing a device ever sends: a GameCube origin. */
static constexpr size_t MAX_DEVICE_REPLY_BYTES = sizeof(gc_origin_t);

static void device_send(DevicePort &device, const void *bytes, size_t len) {
    if (len > MAX_DEVICE_REPLY_BYTES) return;

    // Copied because joybus_send_bytes takes a mutable pointer and the library
    // constants are const. It does not modify them, but casting the const away
    // at every call site is how that stops being true later.
    uint8_t scratch[MAX_DEVICE_REPLY_BYTES];
    memcpy(scratch, bytes, len);

    busy_wait_us(DEVICE_REPLY_DELAY_US);
    if (!wait_line_idle(device.port.pin)) {
        joybus_port_reset(&device.port);
        return;
    }

    // Deliberately NOT joybus_send_bytes. That function opens with an unbounded
    // `while (!gpio_get(pin))`, so a console cable that is unplugged — a line
    // with nothing pulling it up, which is exactly what GP2 looks like with the
    // console powered off — wedges the core forever. wait_line_idle above has
    // already established the line is idle with a timeout, so the rest of the
    // send is inlined here without the spin. Everything below is bounded by the
    // wire rate: the state machine drains the FIFO at 250 kbit.
    joybus_program_send_init(device.port.pio, device.port.sm, device.port.offset, device.port.pin,
                             &device.port.config);
    for (size_t i = 0; i < len; i++) {
        joybus_send_byte(&device.port, scratch[i], i == len - 1);
    }
}

/** True if this console has said anything we have not read yet. */
static bool device_has_pending(const DevicePort &device) {
    return !pio_sm_is_rx_fifo_empty(device.port.pio, device.port.sm);
}

/**
 * Track whether a powered console is on the other end, and resynchronise when
 * one appears.
 *
 * The state machine is reset on BOTH edges, and both resets matter. Going down,
 * it stops a half-received command from outliving the console that started it.
 * Coming up, it throws away everything shifted in while the line was dead —
 * which is the whole bug this exists for, because a receive program pointed at
 * 0 V does not idle, it reads a permanent start bit and fills its FIFO with
 * zeroes. Without the second reset the console powers on, polls, and is
 * answered by a state machine still working through noise from before it was
 * switched on.
 */
/** Mirror a port's link state where the status reply can see it. */
static void publish_console_link(const DevicePort &device) {
    if (device.kind == SAGEBOX_KIND_GAMECUBE) {
        g_gc_console_link = device.link_up;
    } else {
        g_n64_console_link = device.link_up;
    }
}

static void update_device_link(DevicePort &device) {
    if (!device.ready) return;

    const bool high = gpio_get(device.port.pin) != 0;
    const absolute_time_t now = get_absolute_time();

    if (high != device.line_high) {
        device.line_high = high;
        device.level_since = now;
    }

    const int64_t held_us = absolute_time_diff_us(device.level_since, now);

    if (device.link_up) {
        if (!high && held_us >= static_cast<int64_t>(CONSOLE_ABSENT_US)) {
            device.link_up = false;
            joybus_port_reset(&device.port);
            publish_console_link(device);
        }
        return;
    }

    if (high && held_us >= static_cast<int64_t>(CONSOLE_SETTLE_US)) {
        joybus_port_reset(&device.port);
        device.link_up = true;
        publish_console_link(device);
    }
}

static void service_gc_device(DevicePort &device) {
    if (!device.ready || device.parked) return;
    update_device_link(device);
    if (!device.link_up || !device_has_pending(device)) return;

    const uint8_t command = joybus_receive_byte(&device.port);
    switch (static_cast<GamecubeCommand>(command)) {
        case GamecubeCommand::PROBE:
        case GamecubeCommand::RESET:
            device_send(device, &default_gc_status, sizeof(gc_status_t));
            return;
        case GamecubeCommand::ORIGIN:
        case GamecubeCommand::RECALIBRATE:
            device_send(device, &default_gc_origin, sizeof(gc_origin_t));
            return;
        case GamecubeCommand::POLL: {
            // The poll is 3 bytes. The remaining two are already on their way,
            // so this wait is bounded by the timeout and measured in tens of
            // microseconds.
            uint8_t rest[2];
            if (joybus_receive_bytes(&device.port, rest, 2, DEVICE_RECEIVE_TIMEOUT_US, true) != 2 ||
                rest[0] > 0x07) {
                busy_wait_us(GC_RESET_WAIT_US);
                joybus_port_reset(&device.port);
                return;
            }
            // Latched HERE, between the console's last command bit and the
            // reply, so the answer carries the newest composition core 0 has
            // produced rather than whatever was current when the loop last came
            // round.
            uint8_t report[SAGEBOX_MAX_REPORT_BYTES];
            if (latch_output_report(0, report) != SAGEBOX_GC_REPORT_BYTES) {
                // Silence is the answer. An output with no route must look like
                // an empty port, not like a controller holding neutral — the
                // difference is whether the game shows "controller unplugged".
                joybus_port_reset(&device.port);
                return;
            }
            device_send(device, report, SAGEBOX_GC_REPORT_BYTES);
            return;
        }
        default:
            busy_wait_us(GC_RESET_WAIT_US);
            joybus_port_reset(&device.port);
            return;
    }
}

static void service_n64_device(DevicePort &device) {
    if (!device.ready || device.parked) return;
    update_device_link(device);
    if (!device.link_up || !device_has_pending(device)) return;

    const uint8_t command = joybus_receive_byte(&device.port);
    switch (static_cast<N64Command>(command)) {
        case N64Command::PROBE:
        case N64Command::RESET:
            device_send(device, &default_n64_status, sizeof(n64_status_t));
            return;
        case N64Command::POLL: {
            uint8_t report[SAGEBOX_MAX_REPORT_BYTES];
            if (latch_output_report(1, report) != SAGEBOX_N64_REPORT_BYTES) {
                joybus_port_reset(&device.port);
                return;
            }
            device_send(device, report, SAGEBOX_N64_REPORT_BYTES);
            return;
        }
        default:
            // Pak reads and writes land here. The status reply above already
            // says there is no pak, so a console that asks anyway gets nothing
            // and moves on.
            busy_wait_us(N64_RESET_WAIT_US);
            joybus_port_reset(&device.port);
            return;
    }
}

/**
 * Answer whatever the consoles have asked for.
 *
 * Called between every other step of the loop. This is the only thing on core 0
 * with a deadline it does not control, so everything else is arranged around
 * it.
 */
static void service_devices() {
    // Constant-folded, and written as a runtime test on purpose: it keeps both
    // service functions referenced at every stage, so a bisect build cannot
    // quietly differ from the real one by dropping code the compiler warned
    // about.
    if (!DEVICE_SIDE_ANSWERS) return;
    service_gc_device(g_out_gc);
    service_n64_device(g_out_n64);
}

// ---------------------------------------------------------------------------
// Core 0 — front-port polling
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

/** Null when that front port could not claim a PIO state machine. */
static GamecubeController *g_gc_in = nullptr;
static N64Controller *g_n64_in = nullptr;

/** How many pio0 state machines this firmware has handed out so far. */
static uint g_state_machines_taken = 0;

/**
 * Take the next pio0 state machine, without claiming it and without panicking.
 *
 * The COUNTER is the point, and it is load-bearing. The obvious version of this
 * function just scanned for the first unclaimed machine and returned it, on the
 * assumption that the caller would claim it before asking again. The compiler
 * does not share that assumption: it emitted all four calls back to back,
 * BEFORE the first claim, so every one of them found machine 0 free and
 * returned 0. The first port claimed it and the second hit pio_sm_claim on an
 * already-claimed machine, which panics — killing core 1 silently while core 0
 * carried on answering USB. The box looked healthy and polled nothing.
 *
 * Handing out an index from a counter cannot collapse that way. Each call has a
 * visible side effect and returns a different machine no matter how the calls
 * are scheduled, so uniqueness does not depend on the caller claiming in
 * between. The `is_claimed` check remains only to skip a machine some future
 * co-tenant on pio0 has taken; nothing does today.
 *
 * Claiming itself is still joybus_port_init's job. The lesson is not "claim
 * earlier", it is that a lookup whose correctness depends on unrelated code
 * running between two calls is not a lookup, it is a race.
 *
 * @return a state machine index, or -1 when pio0 is exhausted.
 */
static int take_state_machine() {
    while (g_state_machines_taken < NUM_PIO_STATE_MACHINES) {
        const uint sm = g_state_machines_taken++;
        if (!pio_sm_is_claimed(pio0, sm)) return static_cast<int>(sm);
    }
    g_port_fault = true;
    return -1;
}

/**
 * Bring up the four joybus ports. Never panics; sets g_port_fault instead.
 *
 * Ports are claimed front ports first so that a shortage costs an output rather
 * than an input: an input the box cannot read makes every route from it dead,
 * whereas a missing output costs exactly one console.
 */
static void setup_joybus_ports() {
#if SAGEBOX_BOOT_STAGE >= 1
    if (!pio_can_add_program(pio0, &joybus_program)) {
        g_port_fault = true;
        return;
    }
    const int offset = pio_add_program(pio0, &joybus_program);

    const int gc_in_sm = take_state_machine();
    if (gc_in_sm >= 0) {
        g_gc_in = new (g_gc_storage) GamecubeController(PIN_IN_GC, LIBRARY_POLL_HZ, pio0, gc_in_sm,
                                                        offset);
    }

    const int n64_in_sm = take_state_machine();
    if (n64_in_sm >= 0) {
        g_n64_in = new (g_n64_storage) N64Controller(PIN_IN_N64, LIBRARY_POLL_HZ, pio0, n64_in_sm,
                                                     offset);
    }

#endif

#if SAGEBOX_BOOT_STAGE >= 2
    const int gc_out_sm = take_state_machine();
    if (gc_out_sm >= 0) {
        joybus_port_init(&g_out_gc.port, PIN_OUT_GC, pio0, gc_out_sm, offset);
        g_out_gc.ready = true;
    }

    const int n64_out_sm = take_state_machine();
    if (n64_out_sm >= 0) {
        joybus_port_init(&g_out_n64.port, PIN_OUT_N64, pio0, n64_out_sm, offset);
        g_out_n64.ready = true;
    }
#endif
}

static_assert(sizeof(gc_report_t) == SAGEBOX_GC_REPORT_BYTES,
              "GameCube poll response must be 8 bytes");
static_assert(sizeof(n64_report_t) == SAGEBOX_N64_REPORT_BYTES,
              "N64 poll response must be 4 bytes");

/** What the routing sees: one entry per input port, core 0's alone. */
static sagebox_input_state_t g_inputs[SAGEBOX_PORT_COUNT];
/** Next time each input may be polled. Absent ports back off; see below. */
static absolute_time_t g_input_next_poll[SAGEBOX_PORT_COUNT];

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
 * destructed to achieve it: all four ports share one copy of the PIO program,
 * and `joybus_port_terminate` removes that program unconditionally, so tearing
 * one down would pull the program out from under the other three.
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

/**
 * Apply the GameCube bypass switch.
 *
 * GP1 and GP2 only. The N64 pair is on the other side of the DPDT switch and
 * keeps routing normally — a GameCube bypass must not silently kill an N64
 * race. While parked, the console port's state machine is re-initialised on the
 * way out, because a floating pin will have shifted noise into its FIFO that
 * would otherwise be read as a command.
 */
/**
 * The GP1 half, owned by core 0 because core 0 polls that port.
 *
 * GAMECUBE ONLY, and that is the invariant to protect here. The bypass switch
 * is a GameCube switch: GP4 and GP5 are on the other side of it and must keep
 * polling and answering while the lever is left, or throwing it would silently
 * kill an N64 race. An earlier version darkened both sides and would have done
 * exactly that. Nothing in the bypass path may touch the N64 pair.
 */
static void park_gamecube_front_port(bool bypassed) { set_pin_parked(PIN_IN_GC, bypassed); }

/**
 * The GP2 half, owned by core 1 because core 1 owns that port's state machine.
 *
 * Split from the GP1 half so each pin is parked by the core that talks to it.
 * Two cores reconfiguring one pin's function is a race with no upside, and the
 * console port additionally needs its state machine re-initialised on the way
 * out: a floating pin will have shifted noise into the RX FIFO that would
 * otherwise be read as a console command.
 */
static void park_gamecube_console_port(bool bypassed) {
    set_pin_parked(PIN_OUT_GC, bypassed);
    g_out_gc.parked = bypassed;
    if (!bypassed && g_out_gc.ready) joybus_port_reset(&g_out_gc.port);
}

/**
 * Make the staged matrix the active one.
 *
 * Still double buffered and flipped rather than written in place, even though
 * both sides are core 0 now: `send_routing_reply` can read the active matrix
 * from inside a command handler, and a 20-byte structure written in place under
 * a reader is a torn matrix whichever core does it.
 */
static void apply_pending_routing_now() {
    const uint8_t next = g_routing_index ^ 1u;
    g_routing_buffers[next] = g_routing_staged;
    __dmb();
    g_routing_index = next;
}

/** Build every output's next report from the inputs and the active routing. */
static void compose_outputs() {
    const sagebox_routing_t *routing = g_routing_buffers + g_routing_index;
    uint8_t report[SAGEBOX_MAX_REPORT_BYTES];

    const size_t gc_len = sagebox_compose_output(routing, &g_ports, g_inputs, 0, report);
    publish_output_report(0, report, gc_len);

    const size_t n64_len = sagebox_compose_output(routing, &g_ports, g_inputs, 1, report);
    publish_output_report(1, report, n64_len);

    // Outputs 2 and 3 are unwired, so there is nothing to hand them. They still
    // exist in the matrix, and sagebox_routing_validate is what keeps anything
    // from being routed to them.
}

/** Note that an input answered, or did not, and back off if it did not. */
static void record_input(uint8_t port, bool present, const void *payload, uint8_t len) {
    g_inputs[port].present = present ? 1 : 0;
    if (present) {
        memcpy(g_inputs[port].report, payload, len);
        g_input_next_poll[port] = make_timeout_time_us(INPUT_POLL_PERIOD_US);
    } else {
        // Poll's return value is the only thing separating "no controller" from
        // "every button released": the report buffer reads as neutral either
        // way. An absent port emits no frame, and is left alone for a while so
        // its probe and timeout stop eating the consoles' budget.
        g_input_next_poll[port] = make_timeout_time_us(ABSENT_INPUT_RETRY_US);
    }
}

// ---------------------------------------------------------------------------
// Boot probes — SAGEBOX_BOOT_STAGE below zero
// ---------------------------------------------------------------------------
//
// These are not the firmware. They exist because the one thing this project
// never verified on hardware is the thing it is built on: SageboxBridge calls
// stdio_init_all() from CORE 1, and GcDiag — the only build proven to enumerate
// on this board — calls it from CORE 0 and launches core 1 afterwards. a10dce2
// was built but never flashed, so "core 1 owns the USB stack" has never once
// been observed to work here.
//
// Each probe adds one ingredient to a known-good shape:
//
//   -1  varA  stdio on core 0 and nothing else. No core 1, no PIO, no clock change.
//   -2  varB  varA plus core 1 launched into an empty sleep loop.
//   -3  varC  varB plus set_sys_clock_khz(130 MHz) before stdio init.
//
// All three enumerated on the bench, and every build that ran stdio_init_all()
// on core 1 did not. That is what put USB on core 0 in the firmware above.
//
// Kept rather than deleted: if the box ever stops enumerating again, these
// three answer "is it the clock, the second core, or us?" in three flashes
// instead of another afternoon.
//
// Every probe prints a heartbeat over USB CDC twice a second. Nothing lights up
// on this board at all — it is a Pico 2 W, where the LED is on the CYW43 and
// GP25 is that chip's select line — so the serial line is the only signal.
#if SAGEBOX_BOOT_STAGE < 0

#define SAGEBOX_PROBE_SET_CLOCK (SAGEBOX_BOOT_STAGE <= -3)
#define SAGEBOX_PROBE_LAUNCH_CORE1 (SAGEBOX_BOOT_STAGE <= -2)

static void probe_core1_main() {
    while (true) {
        sleep_ms(10);
    }
}

int main() {
#if SAGEBOX_PROBE_SET_CLOCK
    set_sys_clock_khz(130'000, true);
#endif

    stdio_init_all();

#if SAGEBOX_PROBE_LAUNCH_CORE1
    multicore_launch_core1(probe_core1_main);
#endif

    uint32_t beat = 0;
    while (true) {
        printf("sagebox boot probe stage %d beat %u\r\n", SAGEBOX_BOOT_STAGE, beat++);
        sleep_ms(500);
    }
}

#else

/**
 * Core 1 — every Joybus transaction the box makes.
 *
 * Launched only once USB is already up on core 0. Everything below this point
 * touches hardware that can be missing, busy or unpowered; everything core 0
 * did before launching it was plain memory and cannot fail.
 */
static void console_core_main() {
    // Port init only puts a pin in the PIO's RECEIVE state, which does not
    // drive it, so it is safe even if we boot bypassed. Park immediately if so.
    bool parked = g_bypassed;
    park_gamecube_console_port(parked);

    while (true) {
        // Nothing else belongs in this loop. A console's poll waits in the PIO
        // RX FIFO until this core reads it, and every instruction between the
        // command's last bit and the reply is latency the game sees. A held
        // trigger flickering on screen was exactly this: front-port polling
        // shared the loop, the library's poll cooldown busy-waited up to a
        // millisecond, and the console's reply arrived long after it had given
        // up. Composition, routing, USB and the front ports all live on core 0
        // now, and the only thing this core waits for is a console.
        const bool bypassed = g_bypassed;
        if (bypassed != parked) {
            park_gamecube_console_port(bypassed);
            parked = bypassed;
        }

        service_devices();
    }
}

/**
 * True when a front port's line is idle high, so there is something to talk to.
 *
 * A Joybus line idles high because the powered end pulls it up. A controller
 * whose console is switched off takes its 3.43 V from that console, and clamps
 * DATA low through its own protection diode — so "controller plugged in" and
 * "controller powered" are different questions, and only the second one can be
 * polled.
 *
 * Checked BEFORE Poll() rather than relying on the timeout inside it. The
 * library's send is bounded now, but a bounded wait is still a wait, and paying
 * it twice per loop for two dead ports is time taken from the consoles for an
 * answer the pin already gave us for free.
 */
static bool front_port_is_powered(uint pin) { return gpio_get(pin) != 0; }

/** Poll the front ports, frame what they said, and compose the outputs. */
static void poll_front_ports_and_compose(bool bypassed) {
    if (bypassed) {
        // The switch has disconnected GP1, so input 0 cannot be polled and is
        // simply absent. Anything routed from it — including a route to the N64
        // cable — sees an empty port, which is exactly what it is.
        g_inputs[PORT_GC].present = 0;
    } else if (g_gc_in != nullptr && time_reached(g_input_next_poll[PORT_GC])) {
        if (!front_port_is_powered(PIN_IN_GC)) {
            record_input(PORT_GC, false, nullptr, 0);
        } else {
            gc_report_t report;
            const bool present = g_gc_in->Poll(&report, false);
            record_input(PORT_GC, present, &report, sizeof(report));
            if (present) publish(PORT_GC, &report, sizeof(report));
        }
    }
    g_gc_present = g_inputs[PORT_GC].present != 0;

    if (g_n64_in != nullptr && time_reached(g_input_next_poll[PORT_N64])) {
        if (!front_port_is_powered(PIN_IN_N64)) {
            record_input(PORT_N64, false, nullptr, 0);
        } else {
            n64_report_t report;
            const bool present = g_n64_in->Poll(&report, false);
            record_input(PORT_N64, present, &report, sizeof(report));
            if (present) publish(PORT_N64, &report, sizeof(report));
        }
    }
    g_n64_present = g_inputs[PORT_N64].present != 0;

    compose_outputs();
}

int main() {
    set_sys_clock_khz(130'000, true);

    // USB FIRST, on CORE 0, before anything else exists.
    //
    // This ordering is not a preference, it is the only arrangement that works
    // on this board. Bringing the USB stack up on core 1 produces a board that
    // does not enumerate AT ALL — no serial device, nothing to read a fault
    // from, indistinguishable from dead hardware, and only recoverable with the
    // BOOTSEL button. That was bisected on the bench: probes with
    // stdio_init_all() on core 0 enumerate with an idle core 1 and at 130 MHz,
    // and every build that called it from core 1 was silent, including one from
    // before any of the routing work existed.
    //
    // So core 0 owns the entire USB stack — init, frame writes, command reads,
    // control replies — plus the front ports and composition. Core 1 answers
    // consoles and does nothing else.
    stdio_init_all();

    // Pulled UP. The switch's sense row grounds GP6 in bypass and leaves it OPEN
    // in box mode (the box-side lug is unwired), so open must read HIGH = box.
    // A pull-down made both lever positions read as bypassed and the Pico never
    // polled or drove anything (found on the bench 2026-09-13).
    // The wiring, so the polarity can be checked without a meter: switch row 2
    // is C2 = GP6 (J12), A2 = GND (J6), B2 = EMPTY. Lever left grounds GP6
    // through A2; lever right connects it to B2, which goes nowhere. So LOW is
    // bypass and OPEN is box.
    //
    // THE FAIL-SAFE IS CURRENTLY INVERTED, and that is a deliberate trade. With
    // a pull-up, a sense wire that falls off reads HIGH = box mode, and the
    // Pico will happily drive GP2 while the DPDT switch may also be driving it.
    // The 100 Ω in series is what makes that survivable rather than a short.
    // The alternative, a pull-down, is worse today: it reads BOTH lever
    // positions as bypassed, so the box never polls or drives anything at all.
    // TODO: wire B2 to 3V3 (C8, guide solder item 10b), then switch back to
    // gpio_pull_down — at that point open means broken, and failing to bypass
    // is the safe direction again.
    gpio_init(PIN_BYPASS);
    gpio_set_dir(PIN_BYPASS, GPIO_IN);
    gpio_pull_up(PIN_BYPASS);

    sagebox_ports_wired_2x2(&g_ports);
    sagebox_routing_identity(&g_routing_buffers[0]);
    sagebox_routing_identity(&g_routing_buffers[1]);
    sagebox_routing_identity(&g_routing_staged);

    memset(g_inputs, 0, sizeof(g_inputs));
    for (uint8_t i = 0; i < SAGEBOX_PORT_COUNT; i++) {
        g_inputs[i].kind = g_ports.input_kinds[i];
        g_input_next_poll[i] = get_absolute_time();
    }

    g_out_gc.kind = SAGEBOX_KIND_GAMECUBE;
    g_out_n64.kind = SAGEBOX_KIND_N64;
    // Never parked: the bypass switch is GameCube only, so nothing takes GP5
    // away from the Pico. Set explicitly rather than relying on BSS, because
    // "why is this one false forever" is the question a reader will have.
    g_out_n64.parked = false;

    // Every output starts silent. Until core 0 has polled a front port and
    // composed something, a console must see no controller rather than a
    // neutral one it would read as a real, idle pad.
    for (uint8_t output = 0; output < 2; output++) {
        g_output_handoff[output].slot[0].len = 0;
        g_output_handoff[output].slot[1].len = 0;
        g_output_handoff[output].index = 0;
    }

    // All four ports on pio0 — RP2350 gives it exactly four state machines, and
    // this uses every one — sharing ONE copy of the joybus program. There is no
    // room left for a fifth port on pio0; wiring inputs 2-3 and outputs 2-3
    // will mean moving a pair to pio1.
    //
    // Done here, on core 0, before core 1 exists: the state machines are handed
    // out by a counter, and a counter read from two cores is a race for no
    // reason. Core 1 only ever uses what it is given.
    setup_joybus_ports();

    g_bypassed = !gpio_get(PIN_BYPASS);

    // Console answering starts only now that USB is up and can report whatever
    // happens next. Core 1 can find no state machines, or no console, and core 0
    // will still be there to say so over the status flags.
    multicore_launch_core1(console_core_main);

    core0_loop();
}

#endif  // SAGEBOX_BOOT_STAGE < 0
