#include "car_link.h"

#include <string.h>

#define CAR_LINK_COUNT                    2U
#define CAR_LINK_RX_RING_SIZE             128U
#define CAR_LINK_TX_QUEUE_SIZE            8U
#define CAR_LINK_RX_FRAME_QUEUE_SIZE      4U
#define CAR_LINK_ONLINE_TIMEOUT_MS        2000U

typedef struct
{
  UART_HandleTypeDef *uart;
  Lxs1Parser parser;
  uint8_t rx_byte;
  uint8_t rx_ring[CAR_LINK_RX_RING_SIZE];
  volatile uint16_t rx_head;
  volatile uint16_t rx_tail;
  uint8_t tx_data[CAR_LINK_TX_QUEUE_SIZE][LXS1_MAX_FRAME];
  uint8_t tx_length[CAR_LINK_TX_QUEUE_SIZE];
  volatile uint8_t tx_head;
  volatile uint8_t tx_tail;
  volatile uint8_t tx_busy;
  volatile uint8_t restart_rx;
  uint32_t last_rx_ms;
} CarLinkContext;

static CarLinkContext links[CAR_LINK_COUNT];
static CarLinkReceivedFrame rx_frames[CAR_LINK_RX_FRAME_QUEUE_SIZE];
static uint8_t rx_frame_head;
static uint8_t rx_frame_tail;

static CarLinkMask index_to_mask(uint8_t index)
{
  return (index == 0U) ? CAR_LINK_AIRCRAFT : CAR_LINK_GROUND;
}

static uint8_t ring_next(uint8_t value, uint8_t capacity)
{
  value++;
  return (value >= capacity) ? 0U : value;
}

static uint16_t rx_ring_next(uint16_t value)
{
  value++;
  return (value >= CAR_LINK_RX_RING_SIZE) ? 0U : value;
}

static HAL_StatusTypeDef start_receive(CarLinkContext *link)
{
  if (link->uart == NULL)
  {
    return HAL_OK;
  }
  return HAL_UART_Receive_IT(link->uart, &link->rx_byte, 1U);
}

static void start_next_transmit(CarLinkContext *link)
{
  uint8_t tail;

  if ((link->uart == NULL) || (link->tx_busy != 0U) ||
      (link->tx_tail == link->tx_head))
  {
    return;
  }

  tail = link->tx_tail;
  link->tx_busy = 1U;
  if (HAL_UART_Transmit_IT(link->uart,
                           link->tx_data[tail],
                           link->tx_length[tail]) != HAL_OK)
  {
    link->tx_busy = 0U;
  }
}

static uint8_t enqueue_transmit(CarLinkContext *link,
                                const uint8_t *data,
                                uint8_t length)
{
  uint8_t next;
  uint32_t primask;

  if ((link->uart == NULL) || (length == 0U) ||
      (length > LXS1_MAX_FRAME))
  {
    return 0U;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  next = ring_next(link->tx_head, CAR_LINK_TX_QUEUE_SIZE);
  if (next == link->tx_tail)
  {
    if (primask == 0U)
    {
      __enable_irq();
    }
    return 0U;
  }

  memcpy(link->tx_data[link->tx_head], data, length);
  link->tx_length[link->tx_head] = length;
  link->tx_head = next;
  if (primask == 0U)
  {
    __enable_irq();
  }
  return 1U;
}

static void enqueue_received(uint8_t link_index,
                             const Lxs1Frame *frame)
{
  uint8_t next = ring_next(rx_frame_head,
                           CAR_LINK_RX_FRAME_QUEUE_SIZE);

  if (next == rx_frame_tail)
  {
    return;
  }
  rx_frames[rx_frame_head].frame = *frame;
  rx_frames[rx_frame_head].source_link = index_to_mask(link_index);
  rx_frame_head = next;
}

HAL_StatusTypeDef CarLink_Init(UART_HandleTypeDef *aircraft_uart,
                               UART_HandleTypeDef *ground_uart)
{
  uint8_t index;

  memset(links, 0, sizeof(links));
  memset(rx_frames, 0, sizeof(rx_frames));
  rx_frame_head = 0U;
  rx_frame_tail = 0U;
  links[0].uart = aircraft_uart;
  links[1].uart = ground_uart;

  for (index = 0U; index < CAR_LINK_COUNT; index++)
  {
    Lxs1Parser_Init(&links[index].parser);
    if (start_receive(&links[index]) != HAL_OK)
    {
      return HAL_ERROR;
    }
  }
  return HAL_OK;
}

void CarLink_Task(void)
{
  uint8_t index;
  Lxs1Frame frame;

  for (index = 0U; index < CAR_LINK_COUNT; index++)
  {
    CarLinkContext *link = &links[index];

    if (link->uart == NULL)
    {
      continue;
    }
    if (link->restart_rx != 0U)
    {
      link->restart_rx = 0U;
      (void)HAL_UART_AbortReceive(link->uart);
      (void)start_receive(link);
    }

    while (link->rx_tail != link->rx_head)
    {
      uint8_t byte = link->rx_ring[link->rx_tail];
      link->rx_tail = rx_ring_next(link->rx_tail);
      if (Lxs1Parser_Push(&link->parser, byte, &frame) ==
          LXS1_PARSE_FRAME)
      {
        link->last_rx_ms = HAL_GetTick();
        if ((frame.dst == LXS1_NODE_CAR_MCU) ||
            (frame.dst == LXS1_NODE_BROADCAST))
        {
          enqueue_received(index, &frame);
        }
      }
    }
    start_next_transmit(link);
  }
}

uint8_t CarLink_Send(CarLinkMask selected_links,
                     const Lxs1Frame *frame)
{
  uint8_t encoded[LXS1_MAX_FRAME];
  size_t length;
  uint8_t sent = 0U;
  uint8_t index;

  length = Lxs1_Encode(frame, encoded, sizeof(encoded));
  if ((length == 0U) || (length > 255U))
  {
    return 0U;
  }

  for (index = 0U; index < CAR_LINK_COUNT; index++)
  {
    CarLinkMask mask = index_to_mask(index);
    if (((selected_links & mask) != 0U) &&
        (enqueue_transmit(&links[index], encoded,
                          (uint8_t)length) != 0U))
    {
      sent |= (uint8_t)mask;
    }
  }
  return sent;
}

uint8_t CarLink_Receive(CarLinkReceivedFrame *received)
{
  if ((received == NULL) || (rx_frame_tail == rx_frame_head))
  {
    return 0U;
  }

  *received = rx_frames[rx_frame_tail];
  rx_frame_tail = ring_next(rx_frame_tail,
                            CAR_LINK_RX_FRAME_QUEUE_SIZE);
  return 1U;
}

uint8_t CarLink_IsOnline(CarLinkMask selected_link)
{
  uint8_t index;
  uint32_t now_ms = HAL_GetTick();

  for (index = 0U; index < CAR_LINK_COUNT; index++)
  {
    if ((selected_link & index_to_mask(index)) != 0U)
    {
      if ((links[index].uart == NULL) ||
          ((now_ms - links[index].last_rx_ms) >
           CAR_LINK_ONLINE_TIMEOUT_MS))
      {
        return 0U;
      }
    }
  }
  return 1U;
}

void CarLink_OnUartError(UART_HandleTypeDef *huart)
{
  uint8_t index;

  if (huart == NULL)
  {
    return;
  }
  for (index = 0U; index < CAR_LINK_COUNT; index++)
  {
    if ((links[index].uart != NULL) &&
        (links[index].uart->Instance == huart->Instance))
    {
      links[index].restart_rx = 1U;
    }
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  uint8_t index;

  for (index = 0U; index < CAR_LINK_COUNT; index++)
  {
    CarLinkContext *link = &links[index];
    if ((link->uart != NULL) &&
        (link->uart->Instance == huart->Instance))
    {
      uint16_t next = rx_ring_next(link->rx_head);
      if (next != link->rx_tail)
      {
        link->rx_ring[link->rx_head] = link->rx_byte;
        link->rx_head = next;
      }
      if (HAL_UART_Receive_IT(link->uart, &link->rx_byte, 1U) !=
          HAL_OK)
      {
        link->restart_rx = 1U;
      }
      return;
    }
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  uint8_t index;

  for (index = 0U; index < CAR_LINK_COUNT; index++)
  {
    CarLinkContext *link = &links[index];
    if ((link->uart != NULL) &&
        (link->uart->Instance == huart->Instance))
    {
      link->tx_tail = ring_next(link->tx_tail,
                                CAR_LINK_TX_QUEUE_SIZE);
      link->tx_busy = 0U;
      return;
    }
  }
}
