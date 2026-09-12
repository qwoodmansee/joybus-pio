#include "sagebox_framing.h"

#include <string.h>

size_t sagebox_frame_encode(uint8_t *out, size_t out_cap, uint32_t seq, uint32_t t_us,
                            uint8_t port, const uint8_t *payload, size_t payload_len) {
    if (payload_len > SAGEBOX_MAX_PAYLOAD_BYTES) return 0;

    const size_t total = SAGEBOX_FRAME_HEADER_BYTES + payload_len;
    if (out_cap < total) return 0;

    const uint16_t wrapped_seq = (uint16_t)(seq % SAGEBOX_SEQ_MODULUS);

    // Written a byte at a time rather than memcpy'd off a packed struct: the
    // frame is little-endian by contract, not by whatever the compiler happens
    // to be, and a host test compiled on a big-endian machine has to agree.
    out[0] = SAGEBOX_FRAME_SYNC;
    out[1] = (uint8_t)payload_len;
    out[2] = (uint8_t)(wrapped_seq & 0xFFu);
    out[3] = (uint8_t)((wrapped_seq >> 8) & 0xFFu);
    out[4] = (uint8_t)(t_us & 0xFFu);
    out[5] = (uint8_t)((t_us >> 8) & 0xFFu);
    out[6] = (uint8_t)((t_us >> 16) & 0xFFu);
    out[7] = (uint8_t)((t_us >> 24) & 0xFFu);
    out[8] = port;
    if (payload_len > 0) memcpy(out + SAGEBOX_FRAME_HEADER_BYTES, payload, payload_len);

    return total;
}

int sagebox_profile_is_valid(uint8_t profile) { return profile < SAGEBOX_PROFILE_COUNT; }

void sagebox_command_reader_reset(sagebox_command_reader_t *reader) { reader->len = 0; }

/** Drop the leading byte and keep whatever followed it, so it gets re-scanned. */
static void drop_leading_byte(sagebox_command_reader_t *reader) {
    if (reader->len == 0) return;
    reader->len -= 1;
    if (reader->len > 0) memmove(reader->buf, reader->buf + 1, reader->len);
}

/** Rescan the buffer from the front, discarding bytes that cannot start a command. */
static int reader_take(sagebox_command_reader_t *reader, sagebox_command_t *out) {
    while (reader->len > 0) {
        if (reader->buf[0] != SAGEBOX_COMMAND_SYNC) {
            drop_leading_byte(reader);
            continue;
        }
        if (reader->len < SAGEBOX_COMMAND_HEADER_BYTES) return 0;  // need more bytes

        const uint8_t payload_len = reader->buf[1];
        const uint8_t cmd = reader->buf[2];

        // An unrecognised command id means this 0x5A was an ordinary byte, not
        // a header. Drop it and re-scan rather than swallowing `payload_len`
        // bytes of whatever really follows.
        if (cmd != SAGEBOX_CMD_SET_PROFILE && cmd != SAGEBOX_CMD_GET_STATUS) {
            drop_leading_byte(reader);
            continue;
        }

        const size_t total = SAGEBOX_COMMAND_HEADER_BYTES + payload_len;
        if (reader->len < total) return 0;  // need more bytes

        out->cmd = cmd;
        out->payload_len = payload_len;
        if (payload_len > 0) memcpy(out->payload, reader->buf + SAGEBOX_COMMAND_HEADER_BYTES, payload_len);

        reader->len -= total;
        if (reader->len > 0) memmove(reader->buf, reader->buf + total, reader->len);
        return 1;
    }
    return 0;
}

int sagebox_command_reader_push(sagebox_command_reader_t *reader, uint8_t byte,
                                sagebox_command_t *out) {
    if (reader->len >= sizeof(reader->buf)) {
        // Cannot happen while the header is validated before the payload is
        // waited on, but a full buffer must never silently wedge the reader.
        drop_leading_byte(reader);
    }
    reader->buf[reader->len++] = byte;
    return reader_take(reader, out);
}
