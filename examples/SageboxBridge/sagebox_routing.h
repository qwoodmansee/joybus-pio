// Sagebox routing matrix — the pure logic half.
//
// Everything here is decided by arithmetic on bytes: which inputs feed which
// output, how several inputs merge into one report, and how a GameCube report
// becomes an N64 one (and back). None of it touches a pin, a PIO state machine
// or a clock, and none of it includes a pico-sdk header. That is deliberate:
// this is the part that can be proven correct on a laptop, and it is the part
// where a wrong answer is invisible — a mis-scaled stick or a dropped reserved
// bit produces a controller that *works*, just not the way the runner pressed
// it.
//
// The contract is the routing spec shared by the three halves of this feature
// (SageRaces, the Pi bridge, this firmware). Byte layouts below are that
// document's, not an interpretation of it.
//
// Reports are handled as RAW BYTES rather than as `gc_report_t` / `n64_report_t`
// from the pico headers. Those are bit-field structs that drag in
// <pico/stdlib.h>, and relying on a compiler's bit-field allocation to match a
// wire format is exactly the kind of silent breakage this module exists to
// prevent. The byte and bit constants below are the wire format, written out.
#ifndef SAGEBOX_ROUTING_H
#define SAGEBOX_ROUTING_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// --- Shape -----------------------------------------------------------------

/** 4 front ports in, 4 console cables out. Fixed, even where unwired. */
#define SAGEBOX_PORT_COUNT 4u

/** An output may list every input once, so the ordered list tops out at 4. */
#define SAGEBOX_MAX_SOURCES 4u

/** Padding for an unused source slot on the wire. */
#define SAGEBOX_SOURCE_NONE 0xFFu

/** `[merge, src0, src1, src2, src3]` */
#define SAGEBOX_ROUTE_ENTRY_BYTES 5u

/** The whole matrix, in output order. */
#define SAGEBOX_ROUTING_PAYLOAD_BYTES (SAGEBOX_PORT_COUNT * SAGEBOX_ROUTE_ENTRY_BYTES)

/** Port kinds, as the spec numbers them. */
enum {
    SAGEBOX_KIND_NONE = 0,
    SAGEBOX_KIND_GAMECUBE = 1,
    SAGEBOX_KIND_N64 = 2,
};

/** Merge modes, as the spec numbers them. */
enum {
    SAGEBOX_MERGE_PRIORITY = 0,
    SAGEBOX_MERGE_COMBINE = 1,
    SAGEBOX_MERGE_COUNT = 2,
};

// --- Report byte layouts ----------------------------------------------------

#define SAGEBOX_GC_REPORT_BYTES 8u
#define SAGEBOX_N64_REPORT_BYTES 4u

/** Largest report either kind produces; buffers are sized by this. */
#define SAGEBOX_MAX_REPORT_BYTES SAGEBOX_GC_REPORT_BYTES

enum {
    SAGEBOX_GC_BYTE_BUTTONS0 = 0,
    SAGEBOX_GC_BYTE_BUTTONS1 = 1,
    SAGEBOX_GC_BYTE_STICK_X = 2,
    SAGEBOX_GC_BYTE_STICK_Y = 3,
    SAGEBOX_GC_BYTE_CSTICK_X = 4,
    SAGEBOX_GC_BYTE_CSTICK_Y = 5,
    SAGEBOX_GC_BYTE_L_ANALOG = 6,
    SAGEBOX_GC_BYTE_R_ANALOG = 7,
};

/** Byte 0. Bits 6-7 are the reserved zeros the console expects. */
enum {
    SAGEBOX_GC_A = 1u << 0,
    SAGEBOX_GC_B = 1u << 1,
    SAGEBOX_GC_X = 1u << 2,
    SAGEBOX_GC_Y = 1u << 3,
    SAGEBOX_GC_START = 1u << 4,
    SAGEBOX_GC_ORIGIN = 1u << 5,
    SAGEBOX_GC_BUTTONS0_MASK = 0x3Fu,
};

/** Byte 1. Bit 7 is the reserved ONE the console expects. */
enum {
    SAGEBOX_GC_DPAD_LEFT = 1u << 0,
    SAGEBOX_GC_DPAD_RIGHT = 1u << 1,
    SAGEBOX_GC_DPAD_DOWN = 1u << 2,
    SAGEBOX_GC_DPAD_UP = 1u << 3,
    SAGEBOX_GC_Z = 1u << 4,
    SAGEBOX_GC_R = 1u << 5,
    SAGEBOX_GC_L = 1u << 6,
    SAGEBOX_GC_RESERVED_ONE = 1u << 7,
    SAGEBOX_GC_BUTTONS1_MASK = 0x7Fu,
};

enum {
    SAGEBOX_N64_BYTE_BUTTONS0 = 0,
    SAGEBOX_N64_BYTE_BUTTONS1 = 1,
    SAGEBOX_N64_BYTE_STICK_X = 2,
    SAGEBOX_N64_BYTE_STICK_Y = 3,
};

enum {
    SAGEBOX_N64_DPAD_RIGHT = 1u << 0,
    SAGEBOX_N64_DPAD_LEFT = 1u << 1,
    SAGEBOX_N64_DPAD_DOWN = 1u << 2,
    SAGEBOX_N64_DPAD_UP = 1u << 3,
    SAGEBOX_N64_START = 1u << 4,
    SAGEBOX_N64_Z = 1u << 5,
    SAGEBOX_N64_B = 1u << 6,
    SAGEBOX_N64_A = 1u << 7,
};

enum {
    SAGEBOX_N64_C_RIGHT = 1u << 0,
    SAGEBOX_N64_C_LEFT = 1u << 1,
    SAGEBOX_N64_C_DOWN = 1u << 2,
    SAGEBOX_N64_C_UP = 1u << 3,
    SAGEBOX_N64_R = 1u << 4,
    SAGEBOX_N64_L = 1u << 5,
    /** Bits 6-7 are reserved zeros. */
    SAGEBOX_N64_BUTTONS1_MASK = 0x3Fu,
};

/** GameCube analog centre. */
#define SAGEBOX_GC_CENTRE 128
/** A GameCube C-stick past this far from centre counts as a C button. */
#define SAGEBOX_GC_CSTICK_THRESHOLD 40
/** Full N64 stick deflection, signed. */
#define SAGEBOX_N64_STICK_RANGE 80
/** How far a C button pushes a synthesised GameCube C-stick. */
#define SAGEBOX_GC_CSTICK_RANGE 100

// --- Configuration ----------------------------------------------------------

/** One output's route. `sources` is ordered; 0xFF pads the unused tail. */
typedef struct {
    uint8_t merge;
    uint8_t sources[SAGEBOX_MAX_SOURCES];
} sagebox_route_t;

/** The whole matrix. Index is the output port. */
typedef struct {
    sagebox_route_t outputs[SAGEBOX_PORT_COUNT];
} sagebox_routing_t;

/** Which ports physically exist, and as what. Fixed in firmware. */
typedef struct {
    uint8_t input_kinds[SAGEBOX_PORT_COUNT];
    uint8_t output_kinds[SAGEBOX_PORT_COUNT];
} sagebox_ports_t;

/** One input port as the polling loop last saw it. */
typedef struct {
    uint8_t kind;
    /** 0 when nothing answered the last poll. An absent input routes as absent. */
    uint8_t present;
    uint8_t report[SAGEBOX_MAX_REPORT_BYTES];
} sagebox_input_state_t;

// --- Ports ------------------------------------------------------------------

/**
 * The board as wired today: GameCube on 0, N64 on 1, nothing on 2 and 3, the
 * same on both sides. Shared between the firmware and its tests so the two can
 * never disagree about which ports are routable.
 */
void sagebox_ports_wired_2x2(sagebox_ports_t *out);

// --- Routing configuration --------------------------------------------------

/** Boot and reset default: output 0 <- [0], output 1 <- [1], 2 and 3 empty. */
void sagebox_routing_identity(sagebox_routing_t *out);

/**
 * Decode a SET_ROUTING payload.
 *
 * Shape only — this rejects a payload that is not 20 bytes, an unknown merge
 * mode, a source that is neither 0-3 nor the 0xFF pad, and a 0xFF pad followed
 * by a real source (the list is ordered, so a hole in it is a malformed list,
 * not an empty slot). Whether the decoded matrix is *legal for this board* is
 * `sagebox_routing_validate`'s question.
 *
 * @return 1 on success, 0 if the payload is malformed. `out` is untouched on
 *         failure, so a rejected command cannot half-apply.
 */
int sagebox_routing_decode(const uint8_t *payload, size_t len, sagebox_routing_t *out);

/**
 * Check a decoded matrix against the board.
 *
 * Rejects a source whose input port is unwired, a duplicate source within one
 * output, and any source at all on an unwired output.
 *
 * @return 1 if the matrix may be applied, 0 otherwise.
 */
int sagebox_routing_validate(const sagebox_routing_t *routing, const sagebox_ports_t *ports);

/** Write the matrix as the 20 wire bytes. `out` must hold 20. */
void sagebox_routing_encode(const sagebox_routing_t *routing, uint8_t *out);

/** How many real sources an output lists, stopping at the first pad. */
uint8_t sagebox_route_source_count(const sagebox_route_t *route);

// --- Translation ------------------------------------------------------------

/** Neutral reports: sticks centred, nothing pressed, reserved bits correct. */
void sagebox_gc_report_neutral(uint8_t *out);
void sagebox_n64_report_neutral(uint8_t *out);

/** A GameCube controller's report as the N64 console would have seen it. */
void sagebox_translate_gc_to_n64(const uint8_t *gc, uint8_t *n64);

/** An N64 controller's report as the GameCube console would have seen it. */
void sagebox_translate_n64_to_gc(const uint8_t *n64, uint8_t *gc);

/**
 * Convert `in` of kind `from_kind` into kind `to_kind`.
 *
 * @return bytes written into `out`, or 0 if either kind is `none`.
 */
size_t sagebox_translate(const uint8_t *in, uint8_t from_kind, uint8_t to_kind, uint8_t *out);

// --- Composition ------------------------------------------------------------

/**
 * Build one output's report from the routed inputs.
 *
 * Every present source is translated into the OUTPUT's kind first, then merged,
 * so `combine` across a GameCube and an N64 controller is well defined.
 *
 * @return bytes written into `out` (8 for a GameCube output, 4 for an N64 one),
 *         or 0 when the output should emit nothing at all — an unwired output,
 *         an empty source list, or every listed source absent. A 0 means the
 *         console must see no controller, not a neutral one.
 */
size_t sagebox_compose_output(const sagebox_routing_t *routing, const sagebox_ports_t *ports,
                              const sagebox_input_state_t *inputs, uint8_t output, uint8_t *out);

#ifdef __cplusplus
}
#endif

#endif
