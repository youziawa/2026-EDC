#include "lxs1_protocol.h"

#include <string.h>

static void discard_prefix(Lxs1Parser *parser, size_t count)
{
  if (count >= parser->length)
  {
    parser->length = 0U;
    return;
  }

  memmove(parser->buffer, &parser->buffer[count],
          parser->length - count);
  parser->length -= count;
}

size_t Lxs1_Encode(const Lxs1Frame *frame,
                   uint8_t *output,
                   size_t capacity)
{
  size_t total;
  uint8_t declared_length;

  if ((frame == NULL) || (output == NULL) ||
      (frame->data_len > LXS1_MAX_DATA))
  {
    return 0U;
  }

  declared_length = (uint8_t)(3U + frame->data_len);
  total = 2U + 1U + (size_t)declared_length + 2U;
  if (capacity < total)
  {
    return 0U;
  }

  output[0] = LXS1_SOF0;
  output[1] = LXS1_SOF1;
  output[2] = declared_length;
  output[3] = frame->src;
  output[4] = frame->dst;
  output[5] = frame->msg_id;
  if (frame->data_len > 0U)
  {
    memcpy(&output[6], frame->data, frame->data_len);
  }
  output[6U + frame->data_len] = LXS1_EOF0;
  output[7U + frame->data_len] = LXS1_EOF1;
  return total;
}

void Lxs1Parser_Init(Lxs1Parser *parser)
{
  if (parser != NULL)
  {
    memset(parser, 0, sizeof(*parser));
  }
}

Lxs1ParseResult Lxs1Parser_Push(Lxs1Parser *parser,
                                uint8_t byte,
                                Lxs1Frame *output)
{
  size_t total;
  uint8_t declared_length;
  uint8_t data_length;

  if ((parser == NULL) || (output == NULL))
  {
    return LXS1_PARSE_BAD_FRAME;
  }

  if (parser->length >= sizeof(parser->buffer))
  {
    parser->length = 0U;
    parser->overflow_errors++;
    return LXS1_PARSE_OVERFLOW;
  }
  parser->buffer[parser->length++] = byte;

  for (;;)
  {
    if (parser->length < 2U)
    {
      return LXS1_PARSE_NONE;
    }
    if ((parser->buffer[0] != LXS1_SOF0) ||
        (parser->buffer[1] != LXS1_SOF1))
    {
      discard_prefix(parser, 1U);
      continue;
    }
    if (parser->length < 3U)
    {
      return LXS1_PARSE_NONE;
    }

    declared_length = parser->buffer[2];
    if ((declared_length < 3U) ||
        (declared_length > (3U + LXS1_MAX_DATA)))
    {
      discard_prefix(parser, 1U);
      parser->format_errors++;
      return LXS1_PARSE_BAD_FRAME;
    }

    total = 2U + 1U + (size_t)declared_length + 2U;
    if (parser->length < total)
    {
      return LXS1_PARSE_NONE;
    }
    if ((parser->buffer[total - 2U] != LXS1_EOF0) ||
        (parser->buffer[total - 1U] != LXS1_EOF1))
    {
      discard_prefix(parser, 1U);
      parser->format_errors++;
      return LXS1_PARSE_BAD_FRAME;
    }

    data_length = (uint8_t)(declared_length - 3U);
    output->src = parser->buffer[3];
    output->dst = parser->buffer[4];
    output->msg_id = parser->buffer[5];
    output->data_len = data_length;
    if (data_length > 0U)
    {
      memcpy(output->data, &parser->buffer[6], data_length);
    }
    discard_prefix(parser, total);
    parser->frames_ok++;
    return LXS1_PARSE_FRAME;
  }
}

void Lxs1_PutU16(uint8_t *data, uint16_t value)
{
  data[0] = (uint8_t)(value & 0xFFU);
  data[1] = (uint8_t)(value >> 8);
}

void Lxs1_PutI16(uint8_t *data, int16_t value)
{
  Lxs1_PutU16(data, (uint16_t)value);
}

void Lxs1_PutU32(uint8_t *data, uint32_t value)
{
  data[0] = (uint8_t)(value & 0xFFU);
  data[1] = (uint8_t)((value >> 8) & 0xFFU);
  data[2] = (uint8_t)((value >> 16) & 0xFFU);
  data[3] = (uint8_t)((value >> 24) & 0xFFU);
}

uint16_t Lxs1_GetU16(const uint8_t *data)
{
  return (uint16_t)((uint16_t)data[0] |
                    ((uint16_t)data[1] << 8));
}

uint32_t Lxs1_GetU32(const uint8_t *data)
{
  return (uint32_t)data[0] |
         ((uint32_t)data[1] << 8) |
         ((uint32_t)data[2] << 16) |
         ((uint32_t)data[3] << 24);
}
