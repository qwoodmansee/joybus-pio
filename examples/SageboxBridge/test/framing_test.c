// Host test for the Sagebox framing encoder.
//
// The firmware cannot be run without hardware, but the one thing that MUST be
// right before it is flashed is the byte layout: a firmware that encodes
// correct-looking-but-wrong frames produces an overlay showing buttons the
// runner never pressed, and nothing downstream reveals it.
//
// The assertions below are lifted from
// `imports/shared/integrations/sagebox/__tests__/sagebox-frame-factory.test.ts`
// in the SageRaces monorepo, which is the conformance suite for the reference
// encoder. Same vectors, same expected bytes.
//
// Build and run:  ./test/run-host-tests.sh
// Golden dump:    ./build/framing_test --emit-golden
//                 (feeds tests/test_frames.py on the Pi side)

#include "../sagebox_framing.h"

#include <stdio.h>
#include <string.h>

static int failures = 0;
static int checks = 0;

#define CHECK(cond, ...)                                       \
    do {                                                       \
        checks++;                                              \
        if (!(cond)) {                                         \
            failures++;                                        \
            printf("  FAIL %s:%d  ", __FILE__, __LINE__);      \
            printf(__VA_ARGS__);                               \
            printf("\n");                                      \
        }                                                      \
    } while (0)

/** The GameCube payload the reference suite uses: A released, sticks centred. */
static const uint8_t PAYLOAD[8] = {0x00, 0x80, 128, 128, 128, 128, 0, 0};

static void test_writes_the_documented_header_layout(void) {
    uint8_t out[64];
    const size_t n =
        sagebox_frame_encode(out, sizeof(out), 0x1234, 0x89abcdef, 2, PAYLOAD, sizeof(PAYLOAD));

    CHECK(n == SAGEBOX_FRAME_HEADER_BYTES + sizeof(PAYLOAD), "length %zu", n);
    CHECK(out[0] == SAGEBOX_FRAME_SYNC, "sync 0x%02x", out[0]);
    CHECK(out[1] == sizeof(PAYLOAD), "len %u", out[1]);
    // seq u16 little-endian
    CHECK(out[2] == 0x34, "seq lo 0x%02x", out[2]);
    CHECK(out[3] == 0x12, "seq hi 0x%02x", out[3]);
    // tUs u32 little-endian
    CHECK(out[4] == 0xef, "tUs[0] 0x%02x", out[4]);
    CHECK(out[5] == 0xcd, "tUs[1] 0x%02x", out[5]);
    CHECK(out[6] == 0xab, "tUs[2] 0x%02x", out[6]);
    CHECK(out[7] == 0x89, "tUs[3] 0x%02x", out[7]);
    CHECK(out[8] == 2, "port %u", out[8]);
    CHECK(memcmp(out + SAGEBOX_FRAME_HEADER_BYTES, PAYLOAD, sizeof(PAYLOAD)) == 0,
          "payload not copied verbatim");
}

static void test_refuses_a_payload_longer_than_the_length_byte(void) {
    uint8_t out[512];
    uint8_t big[256];
    memset(big, 0, sizeof(big));
    CHECK(sagebox_frame_encode(out, sizeof(out), 1, 1000, 0, big, sizeof(big)) == 0,
          "256-byte payload was accepted");
    // 255 is the largest that fits.
    CHECK(sagebox_frame_encode(out, sizeof(out), 1, 1000, 0, big, 255) ==
              SAGEBOX_FRAME_HEADER_BYTES + 255,
          "255-byte payload was refused");
}

static void test_refuses_a_buffer_that_is_too_small(void) {
    uint8_t out[16];
    CHECK(sagebox_frame_encode(out, SAGEBOX_FRAME_HEADER_BYTES + sizeof(PAYLOAD) - 1, 1, 1000, 0,
                               PAYLOAD, sizeof(PAYLOAD)) == 0,
          "wrote into a buffer that could not hold the frame");
}

static void test_wraps_the_sequence_number_at_16_bits(void) {
    uint8_t out[64];
    sagebox_frame_encode(out, sizeof(out), 0x10001, 1000, 0, PAYLOAD, sizeof(PAYLOAD));
    CHECK(out[2] == 0x01, "wrapped seq lo 0x%02x", out[2]);
    CHECK(out[3] == 0x00, "wrapped seq hi 0x%02x", out[3]);
}

static void test_writes_a_full_scale_microsecond_clock_unsigned(void) {
    uint8_t out[64];
    sagebox_frame_encode(out, sizeof(out), 1, 0xffffffffu, 0, PAYLOAD, sizeof(PAYLOAD));
    CHECK(out[4] == 0xff && out[5] == 0xff && out[6] == 0xff && out[7] == 0xff,
          "0xFFFFFFFF not written as four 0xFF bytes");
}

static void test_a_payload_byte_equal_to_the_sync_byte_is_copied_untouched(void) {
    // A GameCube stick sitting at 0xA5 is an ordinary reading. The encoder must
    // not escape it; the READER is what has to cope, by only accepting a
    // candidate frame that actually decodes.
    const uint8_t payload[8] = {0x00, 0x80, 0xa5, 0xa5, 128, 128, 0, 0};
    uint8_t out[64];
    sagebox_frame_encode(out, sizeof(out), 1, 1000, 0, payload, sizeof(payload));
    CHECK(out[SAGEBOX_FRAME_HEADER_BYTES + 2] == 0xa5, "0xA5 payload byte was altered");
    CHECK(out[SAGEBOX_FRAME_HEADER_BYTES + 3] == 0xa5, "0xA5 payload byte was altered");
}

static void test_n64_payload_is_four_bytes_gamecube_is_eight(void) {
    uint8_t out[64];
    const uint8_t n64[4] = {0x80, 0x00, 0x00, 0x00};
    CHECK(sagebox_frame_encode(out, sizeof(out), 1, 1000, 1, n64, sizeof(n64)) == 13,
          "N64 frame is not 13 bytes");
    CHECK(out[1] == 4, "N64 len byte is %u", out[1]);
    CHECK(sagebox_frame_encode(out, sizeof(out), 1, 1000, 0, PAYLOAD, sizeof(PAYLOAD)) == 17,
          "GameCube frame is not 17 bytes");
}

static void test_control_replies_use_the_same_layout(void) {
    uint8_t out[64];
    const uint8_t body[4] = {SAGEBOX_CMD_GET_STATUS, SAGEBOX_CMD_OK, SAGEBOX_PROFILE_OOT_ESS, 0x07};
    const size_t n =
        sagebox_frame_encode(out, sizeof(out), 9, 42, SAGEBOX_PORT_CONTROL, body, sizeof(body));
    CHECK(n == SAGEBOX_FRAME_HEADER_BYTES + sizeof(body), "control frame length %zu", n);
    CHECK(out[0] == SAGEBOX_FRAME_SYNC, "control frame does not use the input sync byte");
    CHECK(out[8] == SAGEBOX_PORT_CONTROL, "control port byte 0x%02x", out[8]);
}

// --- command reader ---------------------------------------------------------

static int push_all(sagebox_command_reader_t *reader, const uint8_t *bytes, size_t len,
                    sagebox_command_t *out) {
    int got = 0;
    for (size_t i = 0; i < len; i++) {
        if (sagebox_command_reader_push(reader, bytes[i], out)) got++;
    }
    return got;
}

static void test_reads_a_set_profile_command(void) {
    sagebox_command_reader_t reader = {0};
    sagebox_command_t cmd = {0};
    const uint8_t bytes[] = {SAGEBOX_COMMAND_SYNC, 1, SAGEBOX_CMD_SET_PROFILE,
                             SAGEBOX_PROFILE_SM64};

    CHECK(push_all(&reader, bytes, sizeof(bytes), &cmd) == 1, "command not decoded");
    CHECK(cmd.cmd == SAGEBOX_CMD_SET_PROFILE, "cmd 0x%02x", cmd.cmd);
    CHECK(cmd.payload_len == 1, "payload_len %u", cmd.payload_len);
    CHECK(cmd.payload[0] == SAGEBOX_PROFILE_SM64, "profile 0x%02x", cmd.payload[0]);
}

static void test_reads_a_zero_payload_command(void) {
    sagebox_command_reader_t reader = {0};
    sagebox_command_t cmd = {0};
    const uint8_t bytes[] = {SAGEBOX_COMMAND_SYNC, 0, SAGEBOX_CMD_GET_STATUS};

    CHECK(push_all(&reader, bytes, sizeof(bytes), &cmd) == 1, "command not decoded");
    CHECK(cmd.cmd == SAGEBOX_CMD_GET_STATUS, "cmd 0x%02x", cmd.cmd);
    CHECK(cmd.payload_len == 0, "payload_len %u", cmd.payload_len);
}

static void test_resynchronises_past_leading_garbage(void) {
    sagebox_command_reader_t reader = {0};
    sagebox_command_t cmd = {0};
    const uint8_t bytes[] = {0x11, 0x22, 0x33, SAGEBOX_COMMAND_SYNC, 1, SAGEBOX_CMD_SET_PROFILE,
                             SAGEBOX_PROFILE_OOT_ESS};

    CHECK(push_all(&reader, bytes, sizeof(bytes), &cmd) == 1, "command not decoded after garbage");
    CHECK(cmd.payload[0] == SAGEBOX_PROFILE_OOT_ESS, "profile 0x%02x", cmd.payload[0]);
}

static void test_a_sync_byte_with_an_unknown_command_does_not_desynchronise(void) {
    // 0x5A followed by something that is not a command id is an ordinary byte
    // pair, not a header. Swallowing its "payload" would eat the real command
    // that follows.
    sagebox_command_reader_t reader = {0};
    sagebox_command_t cmd = {0};
    const uint8_t bytes[] = {SAGEBOX_COMMAND_SYNC, 40, 0x99,  // not a command id
                             SAGEBOX_COMMAND_SYNC, 1,  SAGEBOX_CMD_SET_PROFILE,
                             SAGEBOX_PROFILE_SM64};

    CHECK(push_all(&reader, bytes, sizeof(bytes), &cmd) == 1, "real command was swallowed");
    CHECK(cmd.cmd == SAGEBOX_CMD_SET_PROFILE, "cmd 0x%02x", cmd.cmd);
    CHECK(cmd.payload[0] == SAGEBOX_PROFILE_SM64, "profile 0x%02x", cmd.payload[0]);
}

static void test_holds_a_partial_command_until_it_completes(void) {
    sagebox_command_reader_t reader = {0};
    sagebox_command_t cmd = {0};
    const uint8_t head[] = {SAGEBOX_COMMAND_SYNC, 1, SAGEBOX_CMD_SET_PROFILE};

    CHECK(push_all(&reader, head, sizeof(head), &cmd) == 0, "emitted before the payload arrived");
    CHECK(sagebox_command_reader_push(&reader, SAGEBOX_PROFILE_PASSTHROUGH, &cmd) == 1,
          "did not emit once the payload arrived");
    CHECK(cmd.payload[0] == SAGEBOX_PROFILE_PASSTHROUGH, "profile 0x%02x", cmd.payload[0]);
}

static void test_reads_two_back_to_back_commands(void) {
    sagebox_command_reader_t reader = {0};
    sagebox_command_t cmd = {0};
    const uint8_t bytes[] = {SAGEBOX_COMMAND_SYNC, 1, SAGEBOX_CMD_SET_PROFILE, SAGEBOX_PROFILE_SM64,
                             SAGEBOX_COMMAND_SYNC, 0, SAGEBOX_CMD_GET_STATUS};

    CHECK(push_all(&reader, bytes, sizeof(bytes), &cmd) == 2, "both commands not decoded");
    CHECK(cmd.cmd == SAGEBOX_CMD_GET_STATUS, "last cmd 0x%02x", cmd.cmd);
}

static void test_status_flags_are_seven_independent_bits(void) {
    // The flags byte is read by the Pi and by SageRaces. A bit that moves, or
    // one that overlaps another, reports a box state that was never true —
    // "bypassed" read as "remap implemented" is exactly the kind of quiet lie
    // this byte exists to prevent.
    CHECK(SAGEBOX_STATUS_FLAG_GC_PRESENT == 0x01, "gc present bit 0x%02x",
          SAGEBOX_STATUS_FLAG_GC_PRESENT);
    CHECK(SAGEBOX_STATUS_FLAG_N64_PRESENT == 0x02, "n64 present bit 0x%02x",
          SAGEBOX_STATUS_FLAG_N64_PRESENT);
    CHECK(SAGEBOX_STATUS_FLAG_BYPASSED == 0x04, "bypassed bit 0x%02x",
          SAGEBOX_STATUS_FLAG_BYPASSED);
    CHECK(SAGEBOX_STATUS_FLAG_PROFILE_REMAP == 0x08, "profile remap bit 0x%02x",
          SAGEBOX_STATUS_FLAG_PROFILE_REMAP);
    CHECK(SAGEBOX_STATUS_FLAG_PORT_FAULT == 0x10, "port fault bit 0x%02x",
          SAGEBOX_STATUS_FLAG_PORT_FAULT);
    CHECK(SAGEBOX_STATUS_FLAG_GC_CONSOLE_LINK == 0x20, "gc console link bit 0x%02x",
          SAGEBOX_STATUS_FLAG_GC_CONSOLE_LINK);
    CHECK(SAGEBOX_STATUS_FLAG_N64_CONSOLE_LINK == 0x40, "n64 console link bit 0x%02x",
          SAGEBOX_STATUS_FLAG_N64_CONSOLE_LINK);

    const unsigned all = SAGEBOX_STATUS_FLAG_GC_PRESENT | SAGEBOX_STATUS_FLAG_N64_PRESENT |
                         SAGEBOX_STATUS_FLAG_BYPASSED | SAGEBOX_STATUS_FLAG_PROFILE_REMAP |
                         SAGEBOX_STATUS_FLAG_PORT_FAULT | SAGEBOX_STATUS_FLAG_GC_CONSOLE_LINK |
                         SAGEBOX_STATUS_FLAG_N64_CONSOLE_LINK;
    CHECK(all == 0x7F, "flags overlap; combined 0x%02x", all);

    // The reply is still eight bytes, so a decoder that predates the new bit
    // and masks only the low three keeps working against a box that sets it.
    const uint8_t body[8] = {SAGEBOX_CMD_GET_STATUS, SAGEBOX_CMD_OK, SAGEBOX_PROFILE_PASSTHROUGH,
                             (uint8_t)all, 0x01, 0x00, 0x02, 0x00};
    uint8_t out[64];
    const size_t n =
        sagebox_frame_encode(out, sizeof(out), 1, 1000, SAGEBOX_PORT_CONTROL, body, sizeof(body));
    CHECK(n == SAGEBOX_FRAME_HEADER_BYTES + 8, "status reply grew to %zu bytes", n);
    CHECK((out[SAGEBOX_FRAME_HEADER_BYTES + 3] & 0x07) == 0x07,
          "the three original flags no longer read correctly");
}

static void test_profile_validation(void) {
    CHECK(sagebox_profile_is_valid(SAGEBOX_PROFILE_PASSTHROUGH), "passthrough rejected");
    CHECK(sagebox_profile_is_valid(SAGEBOX_PROFILE_SM64), "sm64 rejected");
    CHECK(!sagebox_profile_is_valid(3), "profile 3 accepted");
    CHECK(!sagebox_profile_is_valid(255), "profile 255 accepted");
}

// --- golden dump ------------------------------------------------------------

/**
 * Emit the exact bytes this encoder produces for a known set of frames, as one
 * hex line per case. `packages/joybus-bridge/tests/test_frames.py` pins the
 * same strings, so the Python reader is checked against the real C encoder and
 * not against a second reading of the spec.
 */
static void emit_golden(void) {
    struct {
        const char *name;
        uint32_t seq;
        uint32_t t_us;
        uint8_t port;
        const uint8_t *payload;
        size_t len;
    } cases[] = {
        {"gamecube-neutral", 0x1234, 0x89abcdef, 2, PAYLOAD, sizeof(PAYLOAD)},
        {"n64-a-pressed", 1, 987654, 1, (const uint8_t[]){0x80, 0x00, 0x00, 0x00}, 4},
        {"n64-stick-full-left", 2, 16667, 1, (const uint8_t[]){0x00, 0x00, 0xB0, 0x00}, 4},
        {"seq-wrap", 0x10001, 1000, 0, PAYLOAD, sizeof(PAYLOAD)},
        {"clock-max", 7, 0xffffffffu, 0, PAYLOAD, sizeof(PAYLOAD)},
        {"sync-byte-in-payload", 1, 1000, 0,
         (const uint8_t[]){0x00, 0x80, 0xa5, 0xa5, 128, 128, 0, 0}, 8},
        {"control-status-reply", 9, 42, SAGEBOX_PORT_CONTROL,
         (const uint8_t[]){SAGEBOX_CMD_GET_STATUS, SAGEBOX_CMD_OK, SAGEBOX_PROFILE_OOT_ESS, 0x07,
                           0x01, 0x00, 0x01, 0x00},
         8},
    };

    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        uint8_t out[300];
        const size_t n = sagebox_frame_encode(out, sizeof(out), cases[i].seq, cases[i].t_us,
                                              cases[i].port, cases[i].payload, cases[i].len);
        printf("%-22s ", cases[i].name);
        for (size_t b = 0; b < n; b++) printf("%02x", out[b]);
        printf("\n");
    }
}

int main(int argc, char **argv) {
    if (argc > 1 && strcmp(argv[1], "--emit-golden") == 0) {
        emit_golden();
        return 0;
    }

    test_writes_the_documented_header_layout();
    test_refuses_a_payload_longer_than_the_length_byte();
    test_refuses_a_buffer_that_is_too_small();
    test_wraps_the_sequence_number_at_16_bits();
    test_writes_a_full_scale_microsecond_clock_unsigned();
    test_a_payload_byte_equal_to_the_sync_byte_is_copied_untouched();
    test_n64_payload_is_four_bytes_gamecube_is_eight();
    test_control_replies_use_the_same_layout();

    test_reads_a_set_profile_command();
    test_reads_a_zero_payload_command();
    test_resynchronises_past_leading_garbage();
    test_a_sync_byte_with_an_unknown_command_does_not_desynchronise();
    test_holds_a_partial_command_until_it_completes();
    test_reads_two_back_to_back_commands();
    test_status_flags_are_seven_independent_bits();
    test_profile_validation();

    printf("%d checks, %d failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
