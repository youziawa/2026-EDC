#include "lxs1_protocol.h"

#include <string.h>

size_t lxs1_encode(const lxs1_frame_t *frame, uint8_t *out, size_t capacity)
{
    size_t total;
    uint8_t length;

    if (frame == NULL || out == NULL || frame->data_len > LXS1_MAX_DATA) {
        return 0u;
    }
    length = (uint8_t)(3u + frame->data_len);
    total = 2u + 1u + (size_t)length + 2u;
    if (capacity < total) {
        return 0u;
    }

    out[0] = LXS1_SOF0;
    out[1] = LXS1_SOF1;
    out[2] = length;
    out[3] = frame->src;
    out[4] = frame->dst;
    out[5] = frame->msg_id;
    if (frame->data_len != 0u) {
        memcpy(&out[6], frame->data, frame->data_len);
    }
    out[6u + frame->data_len] = LXS1_EOF0;
    out[7u + frame->data_len] = LXS1_EOF1;
    return total;
}

void lxs1_parser_init(lxs1_parser_t *parser)
{
    if (parser != NULL) {
        memset(parser, 0, sizeof(*parser));
    }
}

static void discard_prefix(lxs1_parser_t *parser, size_t count)
{
    if (count >= parser->length) {
        parser->length = 0u;
        return;
    }
    memmove(parser->buffer, &parser->buffer[count], parser->length - count);
    parser->length -= count;
}

lxs1_parse_result_t lxs1_parser_push(
    lxs1_parser_t *parser, uint8_t byte, lxs1_frame_t *out_frame)
{
    size_t total;
    uint8_t declared_length;
    uint8_t data_len;

    if (parser == NULL || out_frame == NULL) {
        return LXS1_PARSE_BAD_FRAME;
    }
    if (parser->length >= sizeof(parser->buffer)) {
        parser->length = 0u;
        parser->overflow_errors++;
        return LXS1_PARSE_OVERFLOW;
    }
    parser->buffer[parser->length++] = byte;

    for (;;) {
        if (parser->length < 2u) {
            return LXS1_PARSE_NONE;
        }
        if (parser->buffer[0] != LXS1_SOF0 || parser->buffer[1] != LXS1_SOF1) {
            discard_prefix(parser, 1u);
            continue;
        }
        if (parser->length < 3u) {
            return LXS1_PARSE_NONE;
        }

        declared_length = parser->buffer[2];
        if (declared_length < 3u || declared_length > 3u + LXS1_MAX_DATA) {
            discard_prefix(parser, 1u);
            parser->format_errors++;
            return LXS1_PARSE_BAD_FRAME;
        }
        total = 2u + 1u + (size_t)declared_length + 2u;
        if (parser->length < total) {
            return LXS1_PARSE_NONE;
        }
        if (parser->buffer[total - 2u] != LXS1_EOF0 ||
            parser->buffer[total - 1u] != LXS1_EOF1) {
            discard_prefix(parser, 1u);
            parser->format_errors++;
            return LXS1_PARSE_BAD_FRAME;
        }

        data_len = (uint8_t)(declared_length - 3u);
        out_frame->src = parser->buffer[3];
        out_frame->dst = parser->buffer[4];
        out_frame->msg_id = parser->buffer[5];
        out_frame->data_len = data_len;
        if (data_len != 0u) {
            memcpy(out_frame->data, &parser->buffer[6], data_len);
        }
        discard_prefix(parser, total);
        parser->frames_ok++;
        return LXS1_PARSE_FRAME;
    }
}
