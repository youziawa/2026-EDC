#ifndef LXS1_PROTOCOL_H
#define LXS1_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LXS1_SOF0 0xAAu
#define LXS1_SOF1 0x55u
#define LXS1_EOF0 0x0Du
#define LXS1_EOF1 0x0Au
#define LXS1_MAX_DATA 64u
#define LXS1_MAX_FRAME (2u + 1u + 3u + LXS1_MAX_DATA + 2u)

enum {
    LXS1_NODE_FC = 0x01,
    LXS1_NODE_AIR_MCU = 0x02,
    LXS1_NODE_AIR_K230 = 0x03,
    LXS1_NODE_CAR_MCU = 0x04,
    LXS1_NODE_CAR_K230 = 0x05,
    LXS1_NODE_GROUND = 0x06,
    LXS1_NODE_BROADCAST = 0xFF,
};

enum lxs1_msg_id {
    LXS1_MSG_HELLO = 0x01,
    LXS1_MSG_HEARTBEAT = 0x02,
    LXS1_MSG_TASK_START = 0x10,
    LXS1_MSG_TASK_ABORT = 0x11,
    LXS1_MSG_TASK_STATE = 0x12,
    LXS1_MSG_FC_CMD = 0x20,
    LXS1_MSG_FC_STATE = 0x21,
    LXS1_MSG_FC_POSE = 0x22,
    LXS1_MSG_CAR_CMD = 0x30,
    LXS1_MSG_CAR_STATE = 0x31,
    LXS1_MSG_CAR_POSE = 0x32,
    LXS1_MSG_TRACK_EVENT = 0x33,
    LXS1_MSG_CAR_DIAGNOSTIC = 0x34,
    /* K230 -> aircraft F4 only; payload is pixel measurements, never forwarded. */
    LXS1_MSG_VISION_PIXEL = 0x40,
    LXS1_MSG_VISION_LANDMARK_RESERVED = 0x41,
    LXS1_MSG_VISION_LINE = 0x42,
    LXS1_MSG_VISION_DIAG = 0x43,
    LXS1_MSG_DROP_STATE = 0x50,
    LXS1_MSG_LAND_STATE = 0x51,
    LXS1_MSG_FAULT = 0x60,
};

typedef struct {
    uint8_t src;
    uint8_t dst;
    uint8_t msg_id;
    uint8_t data_len;
    uint8_t data[LXS1_MAX_DATA];
} lxs1_frame_t;

typedef struct {
    uint8_t buffer[LXS1_MAX_FRAME * 2u];
    size_t length;
    uint32_t frames_ok;
    uint32_t format_errors;
    uint32_t overflow_errors;
} lxs1_parser_t;

typedef enum {
    LXS1_PARSE_NONE = 0,
    LXS1_PARSE_FRAME = 1,
    LXS1_PARSE_BAD_FRAME = -1,
    LXS1_PARSE_OVERFLOW = -2,
} lxs1_parse_result_t;

/* Returns encoded length, or 0 when arguments/capacity are invalid. */
size_t lxs1_encode(const lxs1_frame_t *frame, uint8_t *out, size_t capacity);

void lxs1_parser_init(lxs1_parser_t *parser);

/* Feed one byte. The returned frame is valid only when result is FRAME. */
lxs1_parse_result_t lxs1_parser_push(
    lxs1_parser_t *parser, uint8_t byte, lxs1_frame_t *out_frame);

#ifdef __cplusplus
}
#endif

#endif
