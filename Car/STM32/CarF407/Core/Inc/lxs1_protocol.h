#ifndef LXS1_PROTOCOL_H
#define LXS1_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>

#define LXS1_SOF0                         0xAAU
#define LXS1_SOF1                         0x55U
#define LXS1_EOF0                         0x0DU
#define LXS1_EOF1                         0x0AU
#define LXS1_MAX_DATA                     64U
#define LXS1_MAX_FRAME                    72U

typedef enum
{
  LXS1_NODE_FC = 0x01U,
  LXS1_NODE_AIR_MCU = 0x02U,
  LXS1_NODE_AIR_K230 = 0x03U,
  LXS1_NODE_CAR_MCU = 0x04U,
  LXS1_NODE_CAR_K230 = 0x05U,
  LXS1_NODE_GROUND = 0x06U,
  LXS1_NODE_BROADCAST = 0xFFU
} Lxs1Node;

typedef enum
{
  LXS1_MSG_HELLO = 0x01U,
  LXS1_MSG_HEARTBEAT = 0x02U,
  LXS1_MSG_TASK_START = 0x10U,
  LXS1_MSG_TASK_ABORT = 0x11U,
  LXS1_MSG_TASK_STATE = 0x12U,
  LXS1_MSG_CAR_CMD = 0x30U,
  LXS1_MSG_CAR_STATE = 0x31U,
  LXS1_MSG_CAR_POSE = 0x32U,
  LXS1_MSG_TRACK_EVENT = 0x33U,
  LXS1_MSG_CAR_DIAGNOSTIC = 0x34U,
  LXS1_MSG_DROP_STATE = 0x50U,
  LXS1_MSG_LAND_STATE = 0x51U,
  LXS1_MSG_FAULT = 0x60U
} Lxs1MessageId;

typedef struct
{
  uint8_t src;
  uint8_t dst;
  uint8_t msg_id;
  uint8_t data_len;
  uint8_t data[LXS1_MAX_DATA];
} Lxs1Frame;

typedef struct
{
  uint8_t buffer[LXS1_MAX_FRAME * 2U];
  size_t length;
  uint32_t frames_ok;
  uint32_t format_errors;
  uint32_t overflow_errors;
} Lxs1Parser;

typedef enum
{
  LXS1_PARSE_NONE = 0,
  LXS1_PARSE_FRAME = 1,
  LXS1_PARSE_BAD_FRAME = -1,
  LXS1_PARSE_OVERFLOW = -2
} Lxs1ParseResult;

size_t Lxs1_Encode(const Lxs1Frame *frame,
                   uint8_t *output,
                   size_t capacity);
void Lxs1Parser_Init(Lxs1Parser *parser);
Lxs1ParseResult Lxs1Parser_Push(Lxs1Parser *parser,
                                uint8_t byte,
                                Lxs1Frame *output);

void Lxs1_PutU16(uint8_t *data, uint16_t value);
void Lxs1_PutI16(uint8_t *data, int16_t value);
void Lxs1_PutU32(uint8_t *data, uint32_t value);
uint16_t Lxs1_GetU16(const uint8_t *data);
uint32_t Lxs1_GetU32(const uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif /* LXS1_PROTOCOL_H */
