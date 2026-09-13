// Host test for the Sagebox routing matrix.
//
// The routing logic is the half of this firmware that can be wrong without
// looking wrong. A mis-scaled stick still moves the character; a dropped
// reserved bit still boots the game; a `combine` that lets a centred stick win
// still reads as "the second controller does nothing". None of that shows up on
// a logic analyser, and all of it shows up mid-race.
//
// So every byte layout, every clamp and every merge rule in the routing spec
// gets a vector here, checked against the same C the Pico runs — not against a
// second reading of the document.
//
// Build and run:  ./test/run-host-tests.sh

#include "../sagebox_framing.h"
#include "../sagebox_routing.h"

#include <stdio.h>
#include <string.h>

static int failures = 0;
static int checks = 0;

#define CHECK(cond, ...)                                  \
    do {                                                  \
        checks++;                                         \
        if (!(cond)) {                                    \
            failures++;                                   \
            printf("  FAIL %s:%d  ", __FILE__, __LINE__); \
            printf(__VA_ARGS__);                          \
            printf("\n");                                 \
        }                                                 \
    } while (0)

static void check_bytes(const char *what, const uint8_t *got, const uint8_t *want, size_t len) {
    checks++;
    if (memcmp(got, want, len) == 0) return;
    failures++;
    printf("  FAIL %s\n    got  ", what);
    for (size_t i = 0; i < len; i++) printf("%02x ", got[i]);
    printf("\n    want ");
    for (size_t i = 0; i < len; i++) printf("%02x ", want[i]);
    printf("\n");
}

static sagebox_ports_t board(void) {
    sagebox_ports_t ports;
    sagebox_ports_wired_2x2(&ports);
    return ports;
}

// --- Ports ------------------------------------------------------------------

static void test_only_two_ports_of_each_side_are_wired(void) {
    const sagebox_ports_t ports = board();
    CHECK(ports.input_kinds[0] == SAGEBOX_KIND_GAMECUBE, "input 0 kind %u", ports.input_kinds[0]);
    CHECK(ports.input_kinds[1] == SAGEBOX_KIND_N64, "input 1 kind %u", ports.input_kinds[1]);
    CHECK(ports.input_kinds[2] == SAGEBOX_KIND_NONE, "input 2 kind %u", ports.input_kinds[2]);
    CHECK(ports.input_kinds[3] == SAGEBOX_KIND_NONE, "input 3 kind %u", ports.input_kinds[3]);
    CHECK(ports.output_kinds[0] == SAGEBOX_KIND_GAMECUBE, "output 0 kind %u",
          ports.output_kinds[0]);
    CHECK(ports.output_kinds[1] == SAGEBOX_KIND_N64, "output 1 kind %u", ports.output_kinds[1]);
    CHECK(ports.output_kinds[2] == SAGEBOX_KIND_NONE, "output 2 kind %u", ports.output_kinds[2]);
    CHECK(ports.output_kinds[3] == SAGEBOX_KIND_NONE, "output 3 kind %u", ports.output_kinds[3]);
}

// --- Identity ---------------------------------------------------------------

static void test_identity_is_straight_through_and_valid(void) {
    sagebox_routing_t routing;
    sagebox_routing_identity(&routing);
    const sagebox_ports_t ports = board();

    CHECK(sagebox_routing_validate(&routing, &ports) == 1, "identity rejected by validation");

    CHECK(routing.outputs[0].sources[0] == 0, "output 0 source %u", routing.outputs[0].sources[0]);
    CHECK(routing.outputs[1].sources[0] == 1, "output 1 source %u", routing.outputs[1].sources[0]);
    CHECK(sagebox_route_source_count(&routing.outputs[0]) == 1, "output 0 source count");
    CHECK(sagebox_route_source_count(&routing.outputs[1]) == 1, "output 1 source count");
    CHECK(sagebox_route_source_count(&routing.outputs[2]) == 0, "output 2 is not empty");
    CHECK(sagebox_route_source_count(&routing.outputs[3]) == 0, "output 3 is not empty");

    uint8_t encoded[SAGEBOX_ROUTING_PAYLOAD_BYTES];
    sagebox_routing_encode(&routing, encoded);
    const uint8_t want[SAGEBOX_ROUTING_PAYLOAD_BYTES] = {
        SAGEBOX_MERGE_PRIORITY, 0,    0xFF, 0xFF, 0xFF,  //
        SAGEBOX_MERGE_PRIORITY, 1,    0xFF, 0xFF, 0xFF,  //
        SAGEBOX_MERGE_PRIORITY, 0xFF, 0xFF, 0xFF, 0xFF,  //
        SAGEBOX_MERGE_PRIORITY, 0xFF, 0xFF, 0xFF, 0xFF,
    };
    check_bytes("identity wire bytes", encoded, want, sizeof(want));
}

static void test_identity_passes_each_controller_through_untouched(void) {
    sagebox_routing_t routing;
    sagebox_routing_identity(&routing);
    const sagebox_ports_t ports = board();

    sagebox_input_state_t inputs[SAGEBOX_PORT_COUNT];
    memset(inputs, 0, sizeof(inputs));
    inputs[0].kind = SAGEBOX_KIND_GAMECUBE;
    inputs[0].present = 1;
    const uint8_t gc[SAGEBOX_GC_REPORT_BYTES] = {SAGEBOX_GC_A, 0x80, 200, 128, 128, 128, 0, 33};
    memcpy(inputs[0].report, gc, sizeof(gc));
    inputs[1].kind = SAGEBOX_KIND_N64;
    inputs[1].present = 1;
    const uint8_t n64[SAGEBOX_N64_REPORT_BYTES] = {SAGEBOX_N64_B, SAGEBOX_N64_C_UP, 0xB0, 0x10};
    memcpy(inputs[1].report, n64, sizeof(n64));

    uint8_t out[SAGEBOX_MAX_REPORT_BYTES];
    CHECK(sagebox_compose_output(&routing, &ports, inputs, 0, out) == SAGEBOX_GC_REPORT_BYTES,
          "output 0 length");
    check_bytes("identity output 0", out, gc, sizeof(gc));

    CHECK(sagebox_compose_output(&routing, &ports, inputs, 1, out) == SAGEBOX_N64_REPORT_BYTES,
          "output 1 length");
    check_bytes("identity output 1", out, n64, sizeof(n64));
}

static void test_an_unwired_or_unrouted_output_emits_nothing(void) {
    sagebox_routing_t routing;
    sagebox_routing_identity(&routing);
    const sagebox_ports_t ports = board();

    sagebox_input_state_t inputs[SAGEBOX_PORT_COUNT];
    memset(inputs, 0, sizeof(inputs));
    inputs[0].kind = SAGEBOX_KIND_GAMECUBE;
    inputs[0].present = 1;

    uint8_t out[SAGEBOX_MAX_REPORT_BYTES];
    CHECK(sagebox_compose_output(&routing, &ports, inputs, 2, out) == 0, "unwired output 2 emitted");
    CHECK(sagebox_compose_output(&routing, &ports, inputs, 3, out) == 0, "unwired output 3 emitted");

    // Nothing plugged into input 1, so output 1 must fall silent rather than
    // hand the console a neutral report that looks like a held controller.
    CHECK(sagebox_compose_output(&routing, &ports, inputs, 1, out) == 0,
          "output 1 emitted with its only source absent");
}

// --- Priority ---------------------------------------------------------------

static void test_priority_skips_an_absent_first_source(void) {
    sagebox_routing_t routing;
    sagebox_routing_identity(&routing);
    routing.outputs[1].merge = SAGEBOX_MERGE_PRIORITY;
    routing.outputs[1].sources[0] = 0;  // GameCube front port, first choice
    routing.outputs[1].sources[1] = 1;  // N64 front port, fallback
    const sagebox_ports_t ports = board();
    CHECK(sagebox_routing_validate(&routing, &ports) == 1, "two-source priority route rejected");

    sagebox_input_state_t inputs[SAGEBOX_PORT_COUNT];
    memset(inputs, 0, sizeof(inputs));
    inputs[0].kind = SAGEBOX_KIND_GAMECUBE;
    inputs[0].present = 0;  // nothing in the GameCube port
    inputs[1].kind = SAGEBOX_KIND_N64;
    inputs[1].present = 1;
    const uint8_t n64[SAGEBOX_N64_REPORT_BYTES] = {SAGEBOX_N64_A, 0, 0x20, 0};
    memcpy(inputs[1].report, n64, sizeof(n64));

    uint8_t out[SAGEBOX_MAX_REPORT_BYTES];
    CHECK(sagebox_compose_output(&routing, &ports, inputs, 1, out) == SAGEBOX_N64_REPORT_BYTES,
          "fallback source did not supply the report");
    check_bytes("priority fallback", out, n64, sizeof(n64));

    // Plug the first choice in and it takes over completely — not merged.
    inputs[0].present = 1;
    const uint8_t gc[SAGEBOX_GC_REPORT_BYTES] = {SAGEBOX_GC_B, 0x80, 128, 128, 128, 128, 0, 0};
    memcpy(inputs[0].report, gc, sizeof(gc));
    CHECK(sagebox_compose_output(&routing, &ports, inputs, 1, out) == SAGEBOX_N64_REPORT_BYTES,
          "first source did not supply the report");
    const uint8_t want[SAGEBOX_N64_REPORT_BYTES] = {SAGEBOX_N64_B, 0, 0, 0};
    check_bytes("priority first choice wins whole report", out, want, sizeof(want));
}

// --- Combine ----------------------------------------------------------------

static void test_combine_ors_buttons_and_keeps_the_largest_deflection(void) {
    sagebox_routing_t routing;
    sagebox_routing_identity(&routing);
    routing.outputs[1].merge = SAGEBOX_MERGE_COMBINE;
    routing.outputs[1].sources[0] = 0;
    routing.outputs[1].sources[1] = 1;
    const sagebox_ports_t ports = board();

    sagebox_input_state_t inputs[SAGEBOX_PORT_COUNT];
    memset(inputs, 0, sizeof(inputs));
    inputs[0].kind = SAGEBOX_KIND_GAMECUBE;
    inputs[0].present = 1;
    // B held, main stick hard right (128 + 100 -> a full +80 once scaled).
    const uint8_t gc[SAGEBOX_GC_REPORT_BYTES] = {SAGEBOX_GC_B, 0x80, 228, 128, 128, 128, 0, 0};
    memcpy(inputs[0].report, gc, sizeof(gc));
    inputs[1].kind = SAGEBOX_KIND_N64;
    inputs[1].present = 1;
    // A held, stick barely off centre, and pushed the other way.
    const uint8_t n64[SAGEBOX_N64_REPORT_BYTES] = {SAGEBOX_N64_A, 0, (uint8_t)(int8_t)-20, 0};
    memcpy(inputs[1].report, n64, sizeof(n64));

    uint8_t out[SAGEBOX_MAX_REPORT_BYTES];
    CHECK(sagebox_compose_output(&routing, &ports, inputs, 1, out) == SAGEBOX_N64_REPORT_BYTES,
          "combine length");
    const uint8_t want[SAGEBOX_N64_REPORT_BYTES] = {SAGEBOX_N64_A | SAGEBOX_N64_B, 0, 80, 0};
    check_bytes("combine across kinds", out, want, sizeof(want));

    // Order must not decide an axis: the bigger deflection wins either way.
    routing.outputs[1].sources[0] = 1;
    routing.outputs[1].sources[1] = 0;
    CHECK(sagebox_compose_output(&routing, &ports, inputs, 1, out) == SAGEBOX_N64_REPORT_BYTES,
          "combine length, reversed order");
    check_bytes("combine is order independent", out, want, sizeof(want));
}

static void test_combine_takes_the_larger_trigger_and_an_absent_source_contributes_nothing(void) {
    sagebox_routing_t routing;
    sagebox_routing_identity(&routing);
    routing.outputs[0].merge = SAGEBOX_MERGE_COMBINE;
    routing.outputs[0].sources[0] = 0;
    routing.outputs[0].sources[1] = 1;
    const sagebox_ports_t ports = board();
    CHECK(sagebox_routing_validate(&routing, &ports) == 1, "cross-kind combine route rejected");

    sagebox_input_state_t inputs[SAGEBOX_PORT_COUNT];
    memset(inputs, 0, sizeof(inputs));
    inputs[0].kind = SAGEBOX_KIND_GAMECUBE;
    inputs[0].present = 1;
    // Trigger half pressed, main stick pushed well left.
    const uint8_t gc[SAGEBOX_GC_REPORT_BYTES] = {0, 0x80, 40, 128, 128, 128, 100, 0};
    memcpy(inputs[0].report, gc, sizeof(gc));
    inputs[1].kind = SAGEBOX_KIND_N64;
    inputs[1].present = 1;
    // L held; translation gives it a fully depressed analog trigger.
    const uint8_t n64[SAGEBOX_N64_REPORT_BYTES] = {0, SAGEBOX_N64_L, 0, 0};
    memcpy(inputs[1].report, n64, sizeof(n64));

    uint8_t out[SAGEBOX_MAX_REPORT_BYTES];
    CHECK(sagebox_compose_output(&routing, &ports, inputs, 0, out) == SAGEBOX_GC_REPORT_BYTES,
          "combine onto a GameCube output");
    CHECK(out[SAGEBOX_GC_BYTE_L_ANALOG] == 255, "l_analog %u, wanted the larger of 100 and 255",
          out[SAGEBOX_GC_BYTE_L_ANALOG]);
    CHECK(out[SAGEBOX_GC_BYTE_STICK_X] == 40, "stick_x %u, wanted the GameCube deflection",
          out[SAGEBOX_GC_BYTE_STICK_X]);
    CHECK((out[SAGEBOX_GC_BYTE_BUTTONS1] & SAGEBOX_GC_RESERVED_ONE) != 0,
          "combine cleared the reserved one in byte 1");
    CHECK((out[SAGEBOX_GC_BYTE_BUTTONS0] & ~SAGEBOX_GC_BUTTONS0_MASK) == 0,
          "combine set a reserved bit in byte 0");

    // Unplug the N64 controller and its centred stick must not drag the
    // GameCube one back toward centre, nor its trigger back to zero.
    inputs[1].present = 0;
    CHECK(sagebox_compose_output(&routing, &ports, inputs, 0, out) == SAGEBOX_GC_REPORT_BYTES,
          "combine with one source absent");
    check_bytes("absent source contributes nothing", out, gc, sizeof(gc));
}

// --- Translation: GameCube controller -> N64 console -------------------------

static void test_gc_to_n64_buttons_and_c_stick(void) {
    uint8_t gc[SAGEBOX_GC_REPORT_BYTES];
    sagebox_gc_report_neutral(gc);
    gc[SAGEBOX_GC_BYTE_BUTTONS0] = SAGEBOX_GC_A | SAGEBOX_GC_X;
    gc[SAGEBOX_GC_BYTE_BUTTONS1] |= SAGEBOX_GC_Z | SAGEBOX_GC_DPAD_UP;
    gc[SAGEBOX_GC_BYTE_CSTICK_Y] = SAGEBOX_GC_CENTRE + 72;  // past the ±40 gate

    uint8_t n64[SAGEBOX_N64_REPORT_BYTES];
    sagebox_translate_gc_to_n64(gc, n64);

    const uint8_t want[SAGEBOX_N64_REPORT_BYTES] = {
        SAGEBOX_N64_A | SAGEBOX_N64_Z | SAGEBOX_N64_DPAD_UP,
        SAGEBOX_N64_C_RIGHT | SAGEBOX_N64_C_UP,  // X plus the C-stick
        0,
        0,
    };
    check_bytes("gc -> n64 buttons", n64, want, sizeof(want));
}

static void test_gc_to_n64_c_stick_gate_is_exclusive(void) {
    uint8_t gc[SAGEBOX_GC_REPORT_BYTES];
    uint8_t n64[SAGEBOX_N64_REPORT_BYTES];

    // Exactly ±40 is NOT beyond ±40. A resting C-stick with a little drift must
    // not hold a C button down for a whole race.
    sagebox_gc_report_neutral(gc);
    gc[SAGEBOX_GC_BYTE_CSTICK_X] = SAGEBOX_GC_CENTRE + SAGEBOX_GC_CSTICK_THRESHOLD;
    sagebox_translate_gc_to_n64(gc, n64);
    CHECK(n64[SAGEBOX_N64_BYTE_BUTTONS1] == 0, "C-stick at exactly +40 pressed 0x%02x",
          n64[SAGEBOX_N64_BYTE_BUTTONS1]);

    gc[SAGEBOX_GC_BYTE_CSTICK_X] = SAGEBOX_GC_CENTRE + SAGEBOX_GC_CSTICK_THRESHOLD + 1;
    sagebox_translate_gc_to_n64(gc, n64);
    CHECK(n64[SAGEBOX_N64_BYTE_BUTTONS1] == SAGEBOX_N64_C_RIGHT, "C-stick at +41 gave 0x%02x",
          n64[SAGEBOX_N64_BYTE_BUTTONS1]);

    sagebox_gc_report_neutral(gc);
    gc[SAGEBOX_GC_BYTE_CSTICK_Y] = SAGEBOX_GC_CENTRE - SAGEBOX_GC_CSTICK_THRESHOLD - 1;
    sagebox_translate_gc_to_n64(gc, n64);
    CHECK(n64[SAGEBOX_N64_BYTE_BUTTONS1] == SAGEBOX_N64_C_DOWN, "C-stick at -41 gave 0x%02x",
          n64[SAGEBOX_N64_BYTE_BUTTONS1]);
}

static void test_gc_to_n64_ignores_the_gamecube_reserved_bits(void) {
    // A controller that answers with the reserved bits set — or a bus glitch
    // that flips one — must not manufacture an N64 button out of them.
    uint8_t gc[SAGEBOX_GC_REPORT_BYTES];
    sagebox_gc_report_neutral(gc);
    gc[SAGEBOX_GC_BYTE_BUTTONS0] = 0xC0 | SAGEBOX_GC_ORIGIN;  // reserved pair plus origin
    gc[SAGEBOX_GC_BYTE_BUTTONS1] = SAGEBOX_GC_RESERVED_ONE;

    uint8_t n64[SAGEBOX_N64_REPORT_BYTES];
    sagebox_translate_gc_to_n64(gc, n64);
    const uint8_t want[SAGEBOX_N64_REPORT_BYTES] = {0, 0, 0, 0};
    check_bytes("gc reserved bits leaked into n64", n64, want, sizeof(want));
}

static void test_gc_to_n64_stick_scaling_and_clamping(void) {
    uint8_t gc[SAGEBOX_GC_REPORT_BYTES];
    uint8_t n64[SAGEBOX_N64_REPORT_BYTES];

    sagebox_gc_report_neutral(gc);
    sagebox_translate_gc_to_n64(gc, n64);
    CHECK((int8_t)n64[SAGEBOX_N64_BYTE_STICK_X] == 0, "centred stick became %d",
          (int)(int8_t)n64[SAGEBOX_N64_BYTE_STICK_X]);

    // 128 + 100 is a full GameCube deflection in practice, and maps to the
    // N64's full +80.
    gc[SAGEBOX_GC_BYTE_STICK_X] = SAGEBOX_GC_CENTRE + 100;
    gc[SAGEBOX_GC_BYTE_STICK_Y] = SAGEBOX_GC_CENTRE - 50;
    sagebox_translate_gc_to_n64(gc, n64);
    CHECK((int8_t)n64[SAGEBOX_N64_BYTE_STICK_X] == 80, "+100 became %d",
          (int)(int8_t)n64[SAGEBOX_N64_BYTE_STICK_X]);
    CHECK((int8_t)n64[SAGEBOX_N64_BYTE_STICK_Y] == -40, "-50 became %d",
          (int)(int8_t)n64[SAGEBOX_N64_BYTE_STICK_Y]);

    // The extremes of the byte overshoot ±80 and must clamp, never wrap: a
    // wrapped axis is a stick that snaps to the opposite corner.
    gc[SAGEBOX_GC_BYTE_STICK_X] = 255;
    gc[SAGEBOX_GC_BYTE_STICK_Y] = 0;
    sagebox_translate_gc_to_n64(gc, n64);
    CHECK((int8_t)n64[SAGEBOX_N64_BYTE_STICK_X] == 80, "255 became %d",
          (int)(int8_t)n64[SAGEBOX_N64_BYTE_STICK_X]);
    CHECK((int8_t)n64[SAGEBOX_N64_BYTE_STICK_Y] == -80, "0 became %d",
          (int)(int8_t)n64[SAGEBOX_N64_BYTE_STICK_Y]);
}

// --- Translation: N64 controller -> GameCube console -------------------------

static void test_n64_to_gc_full_report(void) {
    const uint8_t n64[SAGEBOX_N64_REPORT_BYTES] = {
        SAGEBOX_N64_A | SAGEBOX_N64_START,
        SAGEBOX_N64_C_LEFT | SAGEBOX_N64_L,
        (uint8_t)(int8_t)-80,
        (uint8_t)(int8_t)40,
    };

    uint8_t gc[SAGEBOX_GC_REPORT_BYTES];
    sagebox_translate_n64_to_gc(n64, gc);

    const uint8_t want[SAGEBOX_GC_REPORT_BYTES] = {
        SAGEBOX_GC_A | SAGEBOX_GC_START,             // X and Y stay clear
        SAGEBOX_GC_RESERVED_ONE | SAGEBOX_GC_L,      //
        SAGEBOX_GC_CENTRE - SAGEBOX_GC_CSTICK_RANGE, // -80 -> 28
        SAGEBOX_GC_CENTRE + 50,                      // +40 -> 178
        SAGEBOX_GC_CENTRE - SAGEBOX_GC_CSTICK_RANGE, // C-Left -> full left
        SAGEBOX_GC_CENTRE,                           //
        255,                                         // L is digital, so full travel
        0,
    };
    check_bytes("n64 -> gc", gc, want, sizeof(want));
}

static void test_n64_to_gc_sets_the_reserved_bits_the_console_demands(void) {
    uint8_t gc[SAGEBOX_GC_REPORT_BYTES];

    // Every N64 bit set, reserved ones included, is the worst case for bits
    // leaking into the GameCube frame.
    const uint8_t n64[SAGEBOX_N64_REPORT_BYTES] = {0xFF, 0xFF, 0, 0};
    sagebox_translate_n64_to_gc(n64, gc);

    CHECK((gc[SAGEBOX_GC_BYTE_BUTTONS0] & 0xC0) == 0, "byte 0 bits 7-6 are 0x%02x, must be zero",
          gc[SAGEBOX_GC_BYTE_BUTTONS0] & 0xC0);
    CHECK((gc[SAGEBOX_GC_BYTE_BUTTONS1] & SAGEBOX_GC_RESERVED_ONE) != 0,
          "byte 1 bit 7 is clear, the console reads that frame as garbage");

    // Opposed C buttons cancel rather than picking one.
    CHECK(gc[SAGEBOX_GC_BYTE_CSTICK_X] == SAGEBOX_GC_CENTRE, "C-left and C-right gave cstick_x %u",
          gc[SAGEBOX_GC_BYTE_CSTICK_X]);
    CHECK(gc[SAGEBOX_GC_BYTE_CSTICK_Y] == SAGEBOX_GC_CENTRE, "C-up and C-down gave cstick_y %u",
          gc[SAGEBOX_GC_BYTE_CSTICK_Y]);
}

static void test_n64_to_gc_stick_clamps_inside_the_byte(void) {
    uint8_t gc[SAGEBOX_GC_REPORT_BYTES];

    // An N64 stick can read past its nominal ±80. Scaled up by 100/80 that
    // overshoots the byte, so it must clamp — and never to 0, which a GameCube
    // console reads as a broken axis.
    const uint8_t high[SAGEBOX_N64_REPORT_BYTES] = {0, 0, 127, (uint8_t)(int8_t)-128};
    sagebox_translate_n64_to_gc(high, gc);
    CHECK(gc[SAGEBOX_GC_BYTE_STICK_X] == 255, "+127 became %u", gc[SAGEBOX_GC_BYTE_STICK_X]);
    CHECK(gc[SAGEBOX_GC_BYTE_STICK_Y] == 1, "-128 became %u", gc[SAGEBOX_GC_BYTE_STICK_Y]);

    const uint8_t centred[SAGEBOX_N64_REPORT_BYTES] = {0, 0, 0, 0};
    sagebox_translate_n64_to_gc(centred, gc);
    CHECK(gc[SAGEBOX_GC_BYTE_STICK_X] == SAGEBOX_GC_CENTRE, "centred x became %u",
          gc[SAGEBOX_GC_BYTE_STICK_X]);
    CHECK(gc[SAGEBOX_GC_BYTE_STICK_Y] == SAGEBOX_GC_CENTRE, "centred y became %u",
          gc[SAGEBOX_GC_BYTE_STICK_Y]);
}

static void test_translation_round_trips_a_neutral_report(void) {
    uint8_t gc[SAGEBOX_GC_REPORT_BYTES];
    uint8_t n64[SAGEBOX_N64_REPORT_BYTES];
    uint8_t back[SAGEBOX_GC_REPORT_BYTES];
    uint8_t neutral[SAGEBOX_GC_REPORT_BYTES];

    sagebox_gc_report_neutral(gc);
    sagebox_gc_report_neutral(neutral);
    sagebox_translate_gc_to_n64(gc, n64);
    sagebox_translate_n64_to_gc(n64, back);
    check_bytes("neutral gc -> n64 -> gc", back, neutral, sizeof(neutral));
}

// --- SET_ROUTING decoding and validation ------------------------------------

/** What the firmware does with a SET_ROUTING payload: decode, then validate. */
static int accepts(const uint8_t *payload, size_t len) {
    sagebox_routing_t routing;
    const sagebox_ports_t ports = board();
    if (!sagebox_routing_decode(payload, len, &routing)) return 0;
    return sagebox_routing_validate(&routing, &ports);
}

/** Identity as wire bytes, so a case can mutate one field of a valid payload. */
static void identity_payload(uint8_t *out) {
    sagebox_routing_t routing;
    sagebox_routing_identity(&routing);
    sagebox_routing_encode(&routing, out);
}

static void test_set_routing_accepts_a_well_formed_matrix(void) {
    uint8_t payload[SAGEBOX_ROUTING_PAYLOAD_BYTES];
    identity_payload(payload);
    CHECK(accepts(payload, sizeof(payload)) == 1, "identity payload rejected");

    // Both front ports onto the N64 cable, combined. Legal, and the headline
    // thing this feature exists for.
    payload[5] = SAGEBOX_MERGE_COMBINE;
    payload[6] = 0;
    payload[7] = 1;
    CHECK(accepts(payload, sizeof(payload)) == 1, "cross-kind combine payload rejected");
}

static void test_set_routing_rejects_a_wrong_length_payload(void) {
    uint8_t payload[SAGEBOX_ROUTING_PAYLOAD_BYTES + 1];
    identity_payload(payload);
    payload[SAGEBOX_ROUTING_PAYLOAD_BYTES] = 0;
    CHECK(accepts(payload, SAGEBOX_ROUTING_PAYLOAD_BYTES - 1) == 0, "19-byte payload accepted");
    CHECK(accepts(payload, SAGEBOX_ROUTING_PAYLOAD_BYTES + 1) == 0, "21-byte payload accepted");
    CHECK(accepts(payload, 0) == 0, "empty payload accepted");
}

static void test_set_routing_rejects_an_unknown_merge_mode(void) {
    uint8_t payload[SAGEBOX_ROUTING_PAYLOAD_BYTES];
    identity_payload(payload);
    payload[0] = 2;
    CHECK(accepts(payload, sizeof(payload)) == 0, "merge mode 2 accepted");
    payload[0] = 0xFF;
    CHECK(accepts(payload, sizeof(payload)) == 0, "merge mode 0xFF accepted");
}

static void test_set_routing_rejects_a_duplicate_source(void) {
    uint8_t payload[SAGEBOX_ROUTING_PAYLOAD_BYTES];
    identity_payload(payload);
    payload[1] = 0;
    payload[2] = 0;  // output 0 <- [0, 0]
    CHECK(accepts(payload, sizeof(payload)) == 0, "duplicate source accepted");
}

static void test_set_routing_rejects_an_unwired_port(void) {
    uint8_t payload[SAGEBOX_ROUTING_PAYLOAD_BYTES];

    identity_payload(payload);
    payload[1] = 2;  // output 0 <- input 2, which is not wired
    CHECK(accepts(payload, sizeof(payload)) == 0, "unwired source accepted");

    identity_payload(payload);
    payload[11] = 0;  // output 2, which is not wired, <- input 0
    CHECK(accepts(payload, sizeof(payload)) == 0, "source on an unwired output accepted");

    identity_payload(payload);
    payload[1] = 4;  // no such port at all
    CHECK(accepts(payload, sizeof(payload)) == 0, "source 4 accepted");
}

static void test_set_routing_rejects_a_hole_in_the_source_list(void) {
    uint8_t payload[SAGEBOX_ROUTING_PAYLOAD_BYTES];
    identity_payload(payload);
    payload[1] = SAGEBOX_SOURCE_NONE;
    payload[2] = 1;  // pad, then a real source
    CHECK(accepts(payload, sizeof(payload)) == 0, "a hole in the ordered source list was accepted");
}

static void test_a_rejected_payload_leaves_the_caller_s_routing_untouched(void) {
    sagebox_routing_t routing;
    sagebox_routing_identity(&routing);

    uint8_t payload[SAGEBOX_ROUTING_PAYLOAD_BYTES];
    identity_payload(payload);
    // Three good entries and a bad fourth: a decoder that wrote as it went
    // would have already changed outputs 0-2 before noticing.
    payload[15] = 9;
    CHECK(sagebox_routing_decode(payload, sizeof(payload), &routing) == 0,
          "bad fourth entry accepted");

    uint8_t encoded[SAGEBOX_ROUTING_PAYLOAD_BYTES];
    uint8_t identity[SAGEBOX_ROUTING_PAYLOAD_BYTES];
    sagebox_routing_encode(&routing, encoded);
    identity_payload(identity);
    check_bytes("rejected decode mutated the routing", encoded, identity, sizeof(identity));
}

// --- GET_ROUTING echo -------------------------------------------------------

static void test_get_routing_echoes_what_was_set(void) {
    const uint8_t payload[SAGEBOX_ROUTING_PAYLOAD_BYTES] = {
        SAGEBOX_MERGE_COMBINE,  1,    0,    0xFF, 0xFF,  //
        SAGEBOX_MERGE_PRIORITY, 0,    1,    0xFF, 0xFF,  //
        SAGEBOX_MERGE_COMBINE,  0xFF, 0xFF, 0xFF, 0xFF,  //
        SAGEBOX_MERGE_PRIORITY, 0xFF, 0xFF, 0xFF, 0xFF,
    };
    CHECK(accepts(payload, sizeof(payload)) == 1, "payload under test was rejected");

    sagebox_routing_t routing;
    CHECK(sagebox_routing_decode(payload, sizeof(payload), &routing) == 1, "decode failed");

    uint8_t encoded[SAGEBOX_ROUTING_PAYLOAD_BYTES];
    sagebox_routing_encode(&routing, encoded);
    check_bytes("GET_ROUTING echo", encoded, payload, sizeof(payload));
}

// --- Command framing for the new ids ----------------------------------------

static void test_the_new_command_ids_are_recognised_by_the_reader(void) {
    CHECK(sagebox_command_is_known(SAGEBOX_CMD_SET_ROUTING), "SET_ROUTING unknown");
    CHECK(sagebox_command_is_known(SAGEBOX_CMD_GET_ROUTING), "GET_ROUTING unknown");
    CHECK(sagebox_command_is_known(SAGEBOX_CMD_GET_PORTS), "GET_PORTS unknown");
    CHECK(!sagebox_command_is_known(0x06), "0x06 is known");

    CHECK(SAGEBOX_CMD_SET_ROUTING == 0x03, "SET_ROUTING id drifted");
    CHECK(SAGEBOX_CMD_GET_ROUTING == 0x04, "GET_ROUTING id drifted");
    CHECK(SAGEBOX_CMD_GET_PORTS == 0x05, "GET_PORTS id drifted");
}

static void test_a_twenty_byte_set_routing_command_survives_a_sync_byte_in_its_payload(void) {
    // SET_ROUTING carries the longest payload on this wire, and the reader sees
    // it as 20 opaque bytes. One of them equal to the command sync must be
    // carried through as payload, not mistaken for the start of a new command —
    // which would swallow the rest of this one and everything after it.
    //
    // The payload is therefore deliberately not a routable matrix; this checks
    // the reader, and `sagebox_routing_decode` is what judges the contents.
    uint8_t payload[SAGEBOX_ROUTING_PAYLOAD_BYTES];
    identity_payload(payload);
    payload[19] = SAGEBOX_COMMAND_SYNC;

    uint8_t wire[SAGEBOX_COMMAND_HEADER_BYTES + SAGEBOX_ROUTING_PAYLOAD_BYTES];
    wire[0] = SAGEBOX_COMMAND_SYNC;
    wire[1] = SAGEBOX_ROUTING_PAYLOAD_BYTES;
    wire[2] = SAGEBOX_CMD_SET_ROUTING;
    memcpy(wire + SAGEBOX_COMMAND_HEADER_BYTES, payload, sizeof(payload));

    sagebox_command_reader_t reader = {0};
    sagebox_command_t command = {0};
    int decoded = 0;
    for (size_t i = 0; i < sizeof(wire); i++) {
        if (sagebox_command_reader_push(&reader, wire[i], &command)) decoded++;
    }

    CHECK(decoded == 1, "%d commands decoded, wanted 1", decoded);
    CHECK(command.cmd == SAGEBOX_CMD_SET_ROUTING, "cmd 0x%02x", command.cmd);
    CHECK(command.payload_len == SAGEBOX_ROUTING_PAYLOAD_BYTES, "payload_len %u",
          command.payload_len);
    check_bytes("SET_ROUTING payload", command.payload, payload, sizeof(payload));
}

int main(void) {
    test_only_two_ports_of_each_side_are_wired();

    test_identity_is_straight_through_and_valid();
    test_identity_passes_each_controller_through_untouched();
    test_an_unwired_or_unrouted_output_emits_nothing();

    test_priority_skips_an_absent_first_source();

    test_combine_ors_buttons_and_keeps_the_largest_deflection();
    test_combine_takes_the_larger_trigger_and_an_absent_source_contributes_nothing();

    test_gc_to_n64_buttons_and_c_stick();
    test_gc_to_n64_c_stick_gate_is_exclusive();
    test_gc_to_n64_ignores_the_gamecube_reserved_bits();
    test_gc_to_n64_stick_scaling_and_clamping();

    test_n64_to_gc_full_report();
    test_n64_to_gc_sets_the_reserved_bits_the_console_demands();
    test_n64_to_gc_stick_clamps_inside_the_byte();
    test_translation_round_trips_a_neutral_report();

    test_set_routing_accepts_a_well_formed_matrix();
    test_set_routing_rejects_a_wrong_length_payload();
    test_set_routing_rejects_an_unknown_merge_mode();
    test_set_routing_rejects_a_duplicate_source();
    test_set_routing_rejects_an_unwired_port();
    test_set_routing_rejects_a_hole_in_the_source_list();
    test_a_rejected_payload_leaves_the_caller_s_routing_untouched();

    test_get_routing_echoes_what_was_set();

    test_the_new_command_ids_are_recognised_by_the_reader();
    test_a_twenty_byte_set_routing_command_survives_a_sync_byte_in_its_payload();

    printf("%d checks, %d failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
