#ifndef K230_VISION_H
#define K230_VISION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/*
 * K230每25 ms发送一次VISION_LINE。所有字段都对应lxs1_uart.py中的11字节DATA。
 * received_at_ms由F407在收到完整合法帧时填写，用于通信超时停车。
 */
typedef struct
{
  uint8_t valid;
  uint8_t lost_count;
  uint16_t confidence;
  int16_t lateral_error_mm;
  int16_t heading_error_deg;
  int16_t curvature;
  uint8_t marker_detected;
  uint32_t received_at_ms;
  uint32_t frame_count;
  uint32_t malformed_count;
} K230VisionData;

/* 启动USART3循环DMA接收；必须在MX_USART3_UART_Init()之后调用。 */
HAL_StatusTypeDef K230Vision_Init(void);

/* 放在主循环中持续调用，从DMA环形缓冲区取出新字节并解析固定19字节帧。 */
void K230Vision_Task(void);

/* 复制最新一帧；尚未收到任何合法帧时返回0。 */
uint8_t K230Vision_GetLatest(K230VisionData *data);

#ifdef __cplusplus
}
#endif

#endif /* K230_VISION_H */
