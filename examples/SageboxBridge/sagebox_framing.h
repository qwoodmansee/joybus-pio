// Sagebox serial framing — the Pico↔Pi wire format.
//
// This is a C port of `sagebox-frame-factory.ts` in the SageRaces monorepo
// (`imports/shared/integrations/sagebox/`), which is the contract. The layout
// is stated in prose in PROTOCOL.md next to it. If these two ever disagree the
// integration is broken, so the byte-level assertions in `test/framing_test.c`
// are lifted verbatim from that file's jest suite.
//
// Deliberately free of any pico-sdk dependency so the host test compiles it
// with the system compiler and checks the real encoder rather than a
// re-implementation of it.
#ifndef SAGEBOX_FRAMING_H
#define SAGEBOX_FRAMING_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// --- Pico -> Pi: input frames ----------------------------------------------

/** Frame start marker. See sagebox-frame-factory.ts for why 0xA5. */
#define SAGEBOX_FRAME_SYNC 0xA5u

/** sync u8 · len u8 · seq u16 · tUs u32 · port u8 */
#define SAGEBOX_FRAME_HEADER_BYTES 9u

/** `len` is one byte and counts payload bytes only. */
#define SAGEBOX_MAX_PAYLOAD_BYTES 0xFFu

/** Sequence numbers wrap here; gap arithmetic downstream is modulo this. */
#define SAGEBOX_SEQ_MODULUS 0x10000u

/**
 * Port id for a control reply rather than a controller poll.
 *
 * PROTOCOL.md fixes `port` at 0-3 for inputs and says nothing about replies,
 * because the command channel is left open. Reusing the input frame layout
 * with an out-of-band port keeps ONE framing on the wire: the reference
 * decoder parses a reply without choking, and the bridge routes on `port`
 * instead of needing a second resynchroniser. Control frames are consumed by
 * the bridge and never reach the websocket, so the 0-3 range the message
 * schema enforces is never violated.
 */
#define SAGEBOX_PORT_CONTROL 0xC0u

// --- Pi -> Pico: command frames ---------------------------------------------

/**
 * Command start marker, a deliberate nibble-swap of the frame sync so a
 * command can never be mistaken for an input frame by a reader that has lost
 * alignment. Commands only ever travel Pi -> Pico.
 */
#define SAGEBOX_COMMAND_SYNC 0x5Au

/** sync u8 · len u8 · cmd u8 */
#define SAGEBOX_COMMAND_HEADER_BYTES 3u

enum {
    /** payload: one profile id byte. */
    SAGEBOX_CMD_SET_PROFILE = 0x01,
    /** payload: empty. */
    SAGEBOX_CMD_GET_STATUS = 0x02,
    /**
     * payload: 20 bytes, the whole routing matrix — 4 outputs in output order,
     * `[merge, src0, src1, src2, src3]` each, unused source slots 0xFF.
     * See sagebox_routing.h for the field meanings.
     */
    SAGEBOX_CMD_SET_ROUTING = 0x03,
    /** payload: empty. Reply carries the ACTIVE matrix, not a requested one. */
    SAGEBOX_CMD_GET_ROUTING = 0x04,
    /** payload: empty. Reply carries the fixed port kinds. */
    SAGEBOX_CMD_GET_PORTS = 0x05,
};

/**
 * True for a command id this firmware knows.
 *
 * The command reader needs this to tell a header from an ordinary 0x5A in the
 * stream, so a new command that is not listed here is silently unreadable.
 * Keep it beside the enum, and add to both together.
 */
int sagebox_command_is_known(uint8_t cmd);

enum {
    SAGEBOX_CMD_OK = 0x00,
    SAGEBOX_CMD_ERR_UNKNOWN_COMMAND = 0x01,
    SAGEBOX_CMD_ERR_BAD_ARGUMENT = 0x02,
};

/**
 * Profile ids, ordered as `SAGEBOX_PROFILES` in sagebox-profile.ts. The
 * firmware only stores and echoes the value today — no remapping is applied,
 * so every profile currently behaves as passthrough on the wire.
 */
enum {
    SAGEBOX_PROFILE_PASSTHROUGH = 0x00,
    SAGEBOX_PROFILE_OOT_ESS = 0x01,
    SAGEBOX_PROFILE_SM64 = 0x02,
    SAGEBOX_PROFILE_COUNT = 0x03,
};

/** Bits in the status reply's flags byte. */
enum {
    SAGEBOX_STATUS_FLAG_GC_PRESENT = 1u << 0,
    SAGEBOX_STATUS_FLAG_N64_PRESENT = 1u << 1,
    SAGEBOX_STATUS_FLAG_BYPASSED = 1u << 2,
    /**
     * Set only when this firmware actually rewrites sticks for a profile.
     *
     * The profile has always been stored and echoed; for a long time nothing
     * remapped anything, so oot-ess and sm64 were byte-identical to
     * passthrough on the wire while SageRaces told runners their box was
     * rewriting stick values. That is wrong in the direction that gets relied
     * on — a runner declares an input assist they do not have, or a race bans
     * one that does nothing.
     *
     * The fact belongs on the wire rather than in a server-side constant,
     * because it is a fact about the BOX. A constant is only true for a fleet
     * where every box runs current firmware, and it silently becomes a lie the
     * moment one does not. Adding a bit to a byte that already exists keeps
     * the reply eight bytes, so a decoder that masks only the low three bits
     * is unaffected and an older box reports itself honestly as 0.
     */
    SAGEBOX_STATUS_FLAG_PROFILE_REMAP = 1u << 3,
};

/**
 * Encode one frame into `out`.
 *
 * `seq` is masked rather than rejected so a caller can hold a plain
 * incrementing counter and never think about the wrap, exactly as the
 * reference encoder does.
 *
 * @return bytes written, or 0 if the payload is too long for the length byte
 *         or the buffer is too small. Never writes past `out_cap`.
 */
size_t sagebox_frame_encode(uint8_t *out, size_t out_cap, uint32_t seq, uint32_t t_us,
                            uint8_t port, const uint8_t *payload, size_t payload_len);

/** One decoded Pi -> Pico command. */
typedef struct {
    uint8_t cmd;
    uint8_t payload_len;
    uint8_t payload[SAGEBOX_MAX_PAYLOAD_BYTES];
} sagebox_command_t;

/** Byte-at-a-time command reader. Zero-initialise before first use. */
typedef struct {
    uint8_t buf[SAGEBOX_COMMAND_HEADER_BYTES + SAGEBOX_MAX_PAYLOAD_BYTES];
    size_t len;
} sagebox_command_reader_t;

void sagebox_command_reader_reset(sagebox_command_reader_t *reader);

/**
 * Feed one received byte.
 *
 * Resynchronisation follows the same rule the frame reader uses: a candidate
 * header is only kept if what follows it actually decodes, and a byte that
 * cannot start one is dropped and the remainder re-scanned. A payload byte
 * equal to the command sync must not be able to desynchronise the reader.
 *
 * @return 1 when `out` holds a complete command, 0 otherwise.
 */
int sagebox_command_reader_push(sagebox_command_reader_t *reader, uint8_t byte,
                                sagebox_command_t *out);

/** True for a profile id the firmware knows. */
int sagebox_profile_is_valid(uint8_t profile);

#ifdef __cplusplus
}
#endif

#endif
