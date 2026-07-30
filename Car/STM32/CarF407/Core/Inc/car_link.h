#ifndef CAR_LINK_H
#define CAR_LINK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "lxs1_protocol.h"
#include "main.h"

typedef enum
{
  CAR_LINK_NONE = 0U,
  CAR_LINK_AIRCRAFT = (1U << 0),
  CAR_LINK_GROUND = (1U << 1),
  CAR_LINK_BOTH = CAR_LINK_AIRCRAFT | CAR_LINK_GROUND
} CarLinkMask;

typedef struct
{
  Lxs1Frame frame;
  CarLinkMask source_link;
} CarLinkReceivedFrame;

/*
 * aircraft_uart和ground_uart分别连接两块ECB02。未完成第二UART硬件配置时，
 * ground_uart允许暂时传NULL；协议和任务逻辑仍可在飞机链路上运行。
 */
HAL_StatusTypeDef CarLink_Init(UART_HandleTypeDef *aircraft_uart,
                               UART_HandleTypeDef *ground_uart);
void CarLink_Task(void);
uint8_t CarLink_Send(CarLinkMask links, const Lxs1Frame *frame);
uint8_t CarLink_Receive(CarLinkReceivedFrame *received);
uint8_t CarLink_IsOnline(CarLinkMask link);

/* 由统一HAL回调调用；业务解析仍在CarLink_Task主循环中完成。 */
void CarLink_OnUartError(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* CAR_LINK_H */
