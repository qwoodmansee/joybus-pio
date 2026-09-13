#include "sagebox_routing.h"

#include <string.h>

// --- Small helpers ----------------------------------------------------------

static int clamp_int(int value, int low, int high) {
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static int abs_int(int value) { return value < 0 ? -value : value; }

// --- Ports ------------------------------------------------------------------

void sagebox_ports_wired_2x2(sagebox_ports_t *out) {
    out->input_kinds[0] = SAGEBOX_KIND_GAMECUBE;
    out->input_kinds[1] = SAGEBOX_KIND_N64;
    out->input_kinds[2] = SAGEBOX_KIND_NONE;
    out->input_kinds[3] = SAGEBOX_KIND_NONE;

    out->output_kinds[0] = SAGEBOX_KIND_GAMECUBE;
    out->output_kinds[1] = SAGEBOX_KIND_N64;
    out->output_kinds[2] = SAGEBOX_KIND_NONE;
    out->output_kinds[3] = SAGEBOX_KIND_NONE;
}

// --- Routing configuration --------------------------------------------------

void sagebox_routing_identity(sagebox_routing_t *out) {
    for (uint8_t o = 0; o < SAGEBOX_PORT_COUNT; o++) {
        out->outputs[o].merge = SAGEBOX_MERGE_PRIORITY;
        for (uint8_t s = 0; s < SAGEBOX_MAX_SOURCES; s++) {
            out->outputs[o].sources[s] = SAGEBOX_SOURCE_NONE;
        }
    }
    out->outputs[0].sources[0] = 0;
    out->outputs[1].sources[0] = 1;
}

uint8_t sagebox_route_source_count(const sagebox_route_t *route) {
    uint8_t count = 0;
    while (count < SAGEBOX_MAX_SOURCES && route->sources[count] != SAGEBOX_SOURCE_NONE) count++;
    return count;
}

int sagebox_routing_decode(const uint8_t *payload, size_t len, sagebox_routing_t *out) {
    if (len != SAGEBOX_ROUTING_PAYLOAD_BYTES) return 0;

    // Decoded into a local first. A payload whose fourth entry is malformed
    // must leave the caller's copy of the ACTIVE routing untouched, and the
    // cheapest way to guarantee that is to never write to it until the whole
    // payload has passed.
    sagebox_routing_t decoded;

    for (uint8_t o = 0; o < SAGEBOX_PORT_COUNT; o++) {
        const uint8_t *entry = payload + (size_t)o * SAGEBOX_ROUTE_ENTRY_BYTES;

        if (entry[0] >= SAGEBOX_MERGE_COUNT) return 0;
        decoded.outputs[o].merge = entry[0];

        int padding_seen = 0;
        for (uint8_t s = 0; s < SAGEBOX_MAX_SOURCES; s++) {
            const uint8_t source = entry[1 + s];
            if (source == SAGEBOX_SOURCE_NONE) {
                padding_seen = 1;
            } else {
                // The list is ordered, so a real source after a pad is a hole,
                // not a short list. Accepting it would silently renumber the
                // caller's priority order.
                if (padding_seen) return 0;
                if (source >= SAGEBOX_PORT_COUNT) return 0;
            }
            decoded.outputs[o].sources[s] = source;
        }
    }

    *out = decoded;
    return 1;
}

int sagebox_routing_validate(const sagebox_routing_t *routing, const sagebox_ports_t *ports) {
    for (uint8_t o = 0; o < SAGEBOX_PORT_COUNT; o++) {
        const sagebox_route_t *route = routing->outputs + o;
        const uint8_t count = sagebox_route_source_count(route);

        if (route->merge >= SAGEBOX_MERGE_COUNT) return 0;

        // An unwired output may exist in the config — all four always do — but
        // nothing may be routed to it.
        if (ports->output_kinds[o] == SAGEBOX_KIND_NONE) {
            if (count > 0) return 0;
            continue;
        }

        for (uint8_t s = 0; s < count; s++) {
            const uint8_t source = route->sources[s];
            if (source >= SAGEBOX_PORT_COUNT) return 0;
            if (ports->input_kinds[source] == SAGEBOX_KIND_NONE) return 0;
            for (uint8_t earlier = 0; earlier < s; earlier++) {
                if (route->sources[earlier] == source) return 0;
            }
        }
    }
    return 1;
}

void sagebox_routing_encode(const sagebox_routing_t *routing, uint8_t *out) {
    for (uint8_t o = 0; o < SAGEBOX_PORT_COUNT; o++) {
        uint8_t *entry = out + (size_t)o * SAGEBOX_ROUTE_ENTRY_BYTES;
        entry[0] = routing->outputs[o].merge;
        for (uint8_t s = 0; s < SAGEBOX_MAX_SOURCES; s++) {
            entry[1 + s] = routing->outputs[o].sources[s];
        }
    }
}

// --- Neutral reports --------------------------------------------------------

void sagebox_gc_report_neutral(uint8_t *out) {
    out[SAGEBOX_GC_BYTE_BUTTONS0] = 0;
    // The console reads byte 1 bit 7 as a framing constant, not a button. A
    // report with it clear is read as garbage.
    out[SAGEBOX_GC_BYTE_BUTTONS1] = SAGEBOX_GC_RESERVED_ONE;
    out[SAGEBOX_GC_BYTE_STICK_X] = SAGEBOX_GC_CENTRE;
    out[SAGEBOX_GC_BYTE_STICK_Y] = SAGEBOX_GC_CENTRE;
    out[SAGEBOX_GC_BYTE_CSTICK_X] = SAGEBOX_GC_CENTRE;
    out[SAGEBOX_GC_BYTE_CSTICK_Y] = SAGEBOX_GC_CENTRE;
    out[SAGEBOX_GC_BYTE_L_ANALOG] = 0;
    out[SAGEBOX_GC_BYTE_R_ANALOG] = 0;
}

void sagebox_n64_report_neutral(uint8_t *out) {
    out[SAGEBOX_N64_BYTE_BUTTONS0] = 0;
    out[SAGEBOX_N64_BYTE_BUTTONS1] = 0;
    out[SAGEBOX_N64_BYTE_STICK_X] = 0;
    out[SAGEBOX_N64_BYTE_STICK_Y] = 0;
}

// --- Translation ------------------------------------------------------------

void sagebox_translate_gc_to_n64(const uint8_t *gc, uint8_t *n64) {
    const uint8_t b0 = gc[SAGEBOX_GC_BYTE_BUTTONS0];
    const uint8_t b1 = gc[SAGEBOX_GC_BYTE_BUTTONS1];

    uint8_t out0 = 0;
    if (b0 & SAGEBOX_GC_A) out0 |= SAGEBOX_N64_A;
    if (b0 & SAGEBOX_GC_B) out0 |= SAGEBOX_N64_B;
    if (b0 & SAGEBOX_GC_START) out0 |= SAGEBOX_N64_START;
    if (b1 & SAGEBOX_GC_Z) out0 |= SAGEBOX_N64_Z;
    if (b1 & SAGEBOX_GC_DPAD_UP) out0 |= SAGEBOX_N64_DPAD_UP;
    if (b1 & SAGEBOX_GC_DPAD_DOWN) out0 |= SAGEBOX_N64_DPAD_DOWN;
    if (b1 & SAGEBOX_GC_DPAD_LEFT) out0 |= SAGEBOX_N64_DPAD_LEFT;
    if (b1 & SAGEBOX_GC_DPAD_RIGHT) out0 |= SAGEBOX_N64_DPAD_RIGHT;

    uint8_t out1 = 0;
    if (b1 & SAGEBOX_GC_L) out1 |= SAGEBOX_N64_L;
    if (b1 & SAGEBOX_GC_R) out1 |= SAGEBOX_N64_R;

    // X and Y sit where OoT's GameCube layout puts the C buttons a runner
    // actually uses; the C-stick keeps working alongside them, so either input
    // reaches the same N64 bit.
    if (b0 & SAGEBOX_GC_X) out1 |= SAGEBOX_N64_C_RIGHT;
    if (b0 & SAGEBOX_GC_Y) out1 |= SAGEBOX_N64_C_LEFT;

    const int cstick_x = (int)gc[SAGEBOX_GC_BYTE_CSTICK_X] - SAGEBOX_GC_CENTRE;
    const int cstick_y = (int)gc[SAGEBOX_GC_BYTE_CSTICK_Y] - SAGEBOX_GC_CENTRE;
    if (cstick_x > SAGEBOX_GC_CSTICK_THRESHOLD) out1 |= SAGEBOX_N64_C_RIGHT;
    if (cstick_x < -SAGEBOX_GC_CSTICK_THRESHOLD) out1 |= SAGEBOX_N64_C_LEFT;
    if (cstick_y > SAGEBOX_GC_CSTICK_THRESHOLD) out1 |= SAGEBOX_N64_C_UP;
    if (cstick_y < -SAGEBOX_GC_CSTICK_THRESHOLD) out1 |= SAGEBOX_N64_C_DOWN;

    n64[SAGEBOX_N64_BYTE_BUTTONS0] = out0;
    n64[SAGEBOX_N64_BYTE_BUTTONS1] = out1 & SAGEBOX_N64_BUTTONS1_MASK;

    // The GameCube stick reads 0-255 around 128 but only reaches about ±100 in
    // practice, so the scale maps that usable travel onto the N64's ±80 rather
    // than the theoretical ±127. A controller that does swing further is
    // clamped, not wrapped.
    const int stick_x = ((int)gc[SAGEBOX_GC_BYTE_STICK_X] - SAGEBOX_GC_CENTRE) *
                        SAGEBOX_N64_STICK_RANGE / SAGEBOX_GC_CSTICK_RANGE;
    const int stick_y = ((int)gc[SAGEBOX_GC_BYTE_STICK_Y] - SAGEBOX_GC_CENTRE) *
                        SAGEBOX_N64_STICK_RANGE / SAGEBOX_GC_CSTICK_RANGE;
    n64[SAGEBOX_N64_BYTE_STICK_X] =
        (uint8_t)(int8_t)clamp_int(stick_x, -SAGEBOX_N64_STICK_RANGE, SAGEBOX_N64_STICK_RANGE);
    n64[SAGEBOX_N64_BYTE_STICK_Y] =
        (uint8_t)(int8_t)clamp_int(stick_y, -SAGEBOX_N64_STICK_RANGE, SAGEBOX_N64_STICK_RANGE);
}

void sagebox_translate_n64_to_gc(const uint8_t *n64, uint8_t *gc) {
    const uint8_t b0 = n64[SAGEBOX_N64_BYTE_BUTTONS0];
    const uint8_t b1 = n64[SAGEBOX_N64_BYTE_BUTTONS1];

    uint8_t out0 = 0;
    if (b0 & SAGEBOX_N64_A) out0 |= SAGEBOX_GC_A;
    if (b0 & SAGEBOX_N64_B) out0 |= SAGEBOX_GC_B;
    if (b0 & SAGEBOX_N64_START) out0 |= SAGEBOX_GC_START;
    // X and Y have no N64 source. They stay clear rather than inheriting a C
    // button, which would double up with the C-stick below.

    uint8_t out1 = SAGEBOX_GC_RESERVED_ONE;
    if (b0 & SAGEBOX_N64_Z) out1 |= SAGEBOX_GC_Z;
    if (b0 & SAGEBOX_N64_DPAD_UP) out1 |= SAGEBOX_GC_DPAD_UP;
    if (b0 & SAGEBOX_N64_DPAD_DOWN) out1 |= SAGEBOX_GC_DPAD_DOWN;
    if (b0 & SAGEBOX_N64_DPAD_LEFT) out1 |= SAGEBOX_GC_DPAD_LEFT;
    if (b0 & SAGEBOX_N64_DPAD_RIGHT) out1 |= SAGEBOX_GC_DPAD_RIGHT;
    if (b1 & SAGEBOX_N64_L) out1 |= SAGEBOX_GC_L;
    if (b1 & SAGEBOX_N64_R) out1 |= SAGEBOX_GC_R;

    // Byte 0 bits 6-7 are reserved zeros and byte 1 bit 7 a reserved one; the
    // masks below are what keeps a future extra button from landing on them.
    gc[SAGEBOX_GC_BYTE_BUTTONS0] = out0 & SAGEBOX_GC_BUTTONS0_MASK;
    gc[SAGEBOX_GC_BYTE_BUTTONS1] = (out1 & SAGEBOX_GC_BUTTONS1_MASK) | SAGEBOX_GC_RESERVED_ONE;

    int cstick_x = 0;
    int cstick_y = 0;
    if (b1 & SAGEBOX_N64_C_RIGHT) cstick_x += SAGEBOX_GC_CSTICK_RANGE;
    if (b1 & SAGEBOX_N64_C_LEFT) cstick_x -= SAGEBOX_GC_CSTICK_RANGE;
    if (b1 & SAGEBOX_N64_C_UP) cstick_y += SAGEBOX_GC_CSTICK_RANGE;
    if (b1 & SAGEBOX_N64_C_DOWN) cstick_y -= SAGEBOX_GC_CSTICK_RANGE;
    gc[SAGEBOX_GC_BYTE_CSTICK_X] = (uint8_t)clamp_int(SAGEBOX_GC_CENTRE + cstick_x, 1, 255);
    gc[SAGEBOX_GC_BYTE_CSTICK_Y] = (uint8_t)clamp_int(SAGEBOX_GC_CENTRE + cstick_y, 1, 255);

    const int stick_x = SAGEBOX_GC_CENTRE + (int)(int8_t)n64[SAGEBOX_N64_BYTE_STICK_X] *
                                                SAGEBOX_GC_CSTICK_RANGE / SAGEBOX_N64_STICK_RANGE;
    const int stick_y = SAGEBOX_GC_CENTRE + (int)(int8_t)n64[SAGEBOX_N64_BYTE_STICK_Y] *
                                                SAGEBOX_GC_CSTICK_RANGE / SAGEBOX_N64_STICK_RANGE;
    // Clamped to 1-255, not 0-255: a GameCube axis reading exactly 0 is what a
    // disconnected or uncalibrated stick reports, and some titles treat it as
    // such.
    gc[SAGEBOX_GC_BYTE_STICK_X] = (uint8_t)clamp_int(stick_x, 1, 255);
    gc[SAGEBOX_GC_BYTE_STICK_Y] = (uint8_t)clamp_int(stick_y, 1, 255);

    // The N64 controller has no analog triggers. A digital press becomes a
    // fully depressed one so a game reading the analog axis still sees it.
    gc[SAGEBOX_GC_BYTE_L_ANALOG] = (b1 & SAGEBOX_N64_L) ? 255 : 0;
    gc[SAGEBOX_GC_BYTE_R_ANALOG] = (b1 & SAGEBOX_N64_R) ? 255 : 0;
}

size_t sagebox_translate(const uint8_t *in, uint8_t from_kind, uint8_t to_kind, uint8_t *out) {
    if (from_kind == SAGEBOX_KIND_NONE || to_kind == SAGEBOX_KIND_NONE) return 0;

    if (from_kind == to_kind) {
        const size_t len =
            to_kind == SAGEBOX_KIND_GAMECUBE ? SAGEBOX_GC_REPORT_BYTES : SAGEBOX_N64_REPORT_BYTES;
        memcpy(out, in, len);
        return len;
    }

    if (from_kind == SAGEBOX_KIND_GAMECUBE) {
        sagebox_translate_gc_to_n64(in, out);
        return SAGEBOX_N64_REPORT_BYTES;
    }

    sagebox_translate_n64_to_gc(in, out);
    return SAGEBOX_GC_REPORT_BYTES;
}

// --- Merging ----------------------------------------------------------------

/** Take `candidate` if it sits further from `centre` than what we have. */
static uint8_t furthest_from(uint8_t accumulated, uint8_t candidate, int centre) {
    const int have = abs_int((int)accumulated - centre);
    const int want = abs_int((int)candidate - centre);
    return want > have ? candidate : accumulated;
}

/** Same, for the N64's signed axes. */
static uint8_t furthest_from_signed(uint8_t accumulated, uint8_t candidate) {
    const int have = abs_int((int)(int8_t)accumulated);
    const int want = abs_int((int)(int8_t)candidate);
    return want > have ? candidate : accumulated;
}

static void combine_gc(uint8_t *accumulated, const uint8_t *addition) {
    accumulated[SAGEBOX_GC_BYTE_BUTTONS0] =
        (accumulated[SAGEBOX_GC_BYTE_BUTTONS0] | addition[SAGEBOX_GC_BYTE_BUTTONS0]) &
        SAGEBOX_GC_BUTTONS0_MASK;
    accumulated[SAGEBOX_GC_BYTE_BUTTONS1] =
        ((accumulated[SAGEBOX_GC_BYTE_BUTTONS1] | addition[SAGEBOX_GC_BYTE_BUTTONS1]) &
         SAGEBOX_GC_BUTTONS1_MASK) |
        SAGEBOX_GC_RESERVED_ONE;

    accumulated[SAGEBOX_GC_BYTE_STICK_X] = furthest_from(
        accumulated[SAGEBOX_GC_BYTE_STICK_X], addition[SAGEBOX_GC_BYTE_STICK_X], SAGEBOX_GC_CENTRE);
    accumulated[SAGEBOX_GC_BYTE_STICK_Y] = furthest_from(
        accumulated[SAGEBOX_GC_BYTE_STICK_Y], addition[SAGEBOX_GC_BYTE_STICK_Y], SAGEBOX_GC_CENTRE);
    accumulated[SAGEBOX_GC_BYTE_CSTICK_X] =
        furthest_from(accumulated[SAGEBOX_GC_BYTE_CSTICK_X], addition[SAGEBOX_GC_BYTE_CSTICK_X],
                      SAGEBOX_GC_CENTRE);
    accumulated[SAGEBOX_GC_BYTE_CSTICK_Y] =
        furthest_from(accumulated[SAGEBOX_GC_BYTE_CSTICK_Y], addition[SAGEBOX_GC_BYTE_CSTICK_Y],
                      SAGEBOX_GC_CENTRE);

    if (addition[SAGEBOX_GC_BYTE_L_ANALOG] > accumulated[SAGEBOX_GC_BYTE_L_ANALOG]) {
        accumulated[SAGEBOX_GC_BYTE_L_ANALOG] = addition[SAGEBOX_GC_BYTE_L_ANALOG];
    }
    if (addition[SAGEBOX_GC_BYTE_R_ANALOG] > accumulated[SAGEBOX_GC_BYTE_R_ANALOG]) {
        accumulated[SAGEBOX_GC_BYTE_R_ANALOG] = addition[SAGEBOX_GC_BYTE_R_ANALOG];
    }
}

static void combine_n64(uint8_t *accumulated, const uint8_t *addition) {
    accumulated[SAGEBOX_N64_BYTE_BUTTONS0] |= addition[SAGEBOX_N64_BYTE_BUTTONS0];
    accumulated[SAGEBOX_N64_BYTE_BUTTONS1] =
        (accumulated[SAGEBOX_N64_BYTE_BUTTONS1] | addition[SAGEBOX_N64_BYTE_BUTTONS1]) &
        SAGEBOX_N64_BUTTONS1_MASK;
    accumulated[SAGEBOX_N64_BYTE_STICK_X] = furthest_from_signed(
        accumulated[SAGEBOX_N64_BYTE_STICK_X], addition[SAGEBOX_N64_BYTE_STICK_X]);
    accumulated[SAGEBOX_N64_BYTE_STICK_Y] = furthest_from_signed(
        accumulated[SAGEBOX_N64_BYTE_STICK_Y], addition[SAGEBOX_N64_BYTE_STICK_Y]);
}

// --- Composition ------------------------------------------------------------

size_t sagebox_compose_output(const sagebox_routing_t *routing, const sagebox_ports_t *ports,
                              const sagebox_input_state_t *inputs, uint8_t output, uint8_t *out) {
    if (output >= SAGEBOX_PORT_COUNT) return 0;

    const uint8_t kind = ports->output_kinds[output];
    if (kind == SAGEBOX_KIND_NONE) return 0;

    const sagebox_route_t *route = routing->outputs + output;
    const uint8_t count = sagebox_route_source_count(route);
    if (count == 0) return 0;

    const size_t len =
        kind == SAGEBOX_KIND_GAMECUBE ? SAGEBOX_GC_REPORT_BYTES : SAGEBOX_N64_REPORT_BYTES;

    uint8_t merged[SAGEBOX_MAX_REPORT_BYTES];
    int merged_any = 0;

    for (uint8_t s = 0; s < count; s++) {
        const uint8_t source = route->sources[s];
        if (source >= SAGEBOX_PORT_COUNT) continue;

        const sagebox_input_state_t *input = inputs + source;
        // An input with nothing plugged in is skipped, not merged as neutral:
        // under `priority` that is the whole point, and under `combine` a
        // neutral contribution would still win an axis the others left centred.
        if (!input->present || input->kind == SAGEBOX_KIND_NONE) continue;

        uint8_t translated[SAGEBOX_MAX_REPORT_BYTES];
        if (sagebox_translate(input->report, input->kind, kind, translated) != len) continue;

        if (!merged_any) {
            memcpy(merged, translated, len);
            merged_any = 1;
            if (route->merge == SAGEBOX_MERGE_PRIORITY) break;
            continue;
        }

        if (kind == SAGEBOX_KIND_GAMECUBE) {
            combine_gc(merged, translated);
        } else {
            combine_n64(merged, translated);
        }
    }

    if (!merged_any) return 0;

    memcpy(out, merged, len);
    return len;
}
