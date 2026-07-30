#include "k230_vision.h"

#include "car_link.h"
#include "usart.h"

#include <string.h>

/*
 * K230 LXS1 VISION_LINE固定帧：
 * AA 55 | 0E | 05 | 04 | 42 | DATA(11) | 0D 0A
 *
 * 按题目约定不使用校验和或CRC，因此解析器会严格检查长度、地址、消息号、
 * 帧头帧尾及字段范围，尽量降低串口错位被当成控制量的概率。
 */
#define K230_DMA_BUFFER_SIZE              128U
#define K230_FRAME_SIZE                   19U
#define K230_FRAME_HEAD_0                 0xAAU
#define K230_FRAME_HEAD_1                 0x55U
#define K230_FRAME_DATA_LENGTH            0x0EU
#define K230_SOURCE_ADDRESS               0x05U
#define CAR_DESTINATION_ADDRESS           0x04U
#define MSG_VISION_LINE                   0x42U
#define K230_FRAME_TAIL_0                 0x0DU
#define K230_FRAME_TAIL_1                 0x0AU

static uint8_t dma_buffer[K230_DMA_BUFFER_SIZE];
static uint8_t frame_buffer[K230_FRAME_SIZE];
static uint16_t dma_read_index;
static uint8_t frame_index;
static volatile uint8_t restart_required;
static uint8_t initialized;
static K230VisionData latest;

static uint16_t read_u16_le(const uint8_t *data)
{
  return (uint16_t)((uint16_t)data[0] |
                    ((uint16_t)data[1] << 8));
}

static int16_t read_i16_le(const uint8_t *data)
{
  return (int16_t)read_u16_le(data);
}

static uint8_t frame_is_valid(const uint8_t *frame)
{
  if ((frame[0] != K230_FRAME_HEAD_0) ||
      (frame[1] != K230_FRAME_HEAD_1) ||
      (frame[2] != K230_FRAME_DATA_LENGTH) ||
      (frame[3] != K230_SOURCE_ADDRESS) ||
      (frame[4] != CAR_DESTINATION_ADDRESS) ||
      (frame[5] != MSG_VISION_LINE) ||
      (frame[17] != K230_FRAME_TAIL_0) ||
      (frame[18] != K230_FRAME_TAIL_1))
  {
    return 0U;
  }

  /* valid只能为0/1，confidence必须符合K230定义的0～1000范围。 */
  if ((frame[6] > 1U) ||
      (read_u16_le(&frame[8]) > 1000U) ||
      (frame[16] > 1U))
  {
    return 0U;
  }

  return 1U;
}

static void publish_frame(const uint8_t *frame)
{
  latest.valid = frame[6];
  latest.lost_count = frame[7];
  latest.confidence = read_u16_le(&frame[8]);
  latest.lateral_error_mm = read_i16_le(&frame[10]);
  latest.heading_error_deg = read_i16_le(&frame[12]);
  latest.curvature = read_i16_le(&frame[14]);
  latest.marker_detected = frame[16];
  latest.received_at_ms = HAL_GetTick();
  latest.frame_count++;
}

/*
 * 流式状态机允许帧从DMA缓冲区任意位置开始，也能在干扰后重新寻找AA 55。
 */
static void parse_byte(uint8_t value)
{
  if (frame_index == 0U)
  {
    if (value == K230_FRAME_HEAD_0)
    {
      frame_buffer[0] = value;
      frame_index = 1U;
    }
    return;
  }

  if (frame_index == 1U)
  {
    if (value == K230_FRAME_HEAD_1)
    {
      frame_buffer[1] = value;
      frame_index = 2U;
    }
    else if (value == K230_FRAME_HEAD_0)
    {
      /* 连续AA时保留后一字节作为新帧头。 */
      frame_buffer[0] = value;
    }
    else
    {
      frame_index = 0U;
    }
    return;
  }

  frame_buffer[frame_index++] = value;
  if (frame_index < K230_FRAME_SIZE)
  {
    return;
  }

  if (frame_is_valid(frame_buffer))
  {
    publish_frame(frame_buffer);
  }
  else
  {
    latest.malformed_count++;
  }

  /*
   * 当前19字节已经消费完。下一字节重新找帧头；固定帧尾0D 0A不会与AA冲突。
   */
  frame_index = 0U;
}

static HAL_StatusTypeDef start_dma_receive(void)
{
  dma_read_index = 0U;
  memset(dma_buffer, 0, sizeof(dma_buffer));

  if (HAL_UART_Receive_DMA(&huart3, dma_buffer,
                           K230_DMA_BUFFER_SIZE) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /*
   * 本模块在主循环中读取DMA写指针，不依赖半满中断。关闭HT可减少40 Hz通信
   * 之外的无意义中断；循环DMA的TC中断仍由HAL正常处理。
   */
  __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
  return HAL_OK;
}

HAL_StatusTypeDef K230Vision_Init(void)
{
  memset(&latest, 0, sizeof(latest));
  frame_index = 0U;
  restart_required = 0U;
  initialized = 0U;

  if (huart3.hdmarx == NULL)
  {
    return HAL_ERROR;
  }

  if (start_dma_receive() != HAL_OK)
  {
    return HAL_ERROR;
  }

  initialized = 1U;
  return HAL_OK;
}

void K230Vision_Task(void)
{
  uint16_t dma_write_index;

  if (initialized == 0U)
  {
    return;
  }

  if (restart_required != 0U)
  {
    restart_required = 0U;
    (void)HAL_UART_AbortReceive(&huart3);
    if (start_dma_receive() != HAL_OK)
    {
      initialized = 0U;
      return;
    }
    frame_index = 0U;
  }

  dma_write_index =
      (uint16_t)(K230_DMA_BUFFER_SIZE -
                 __HAL_DMA_GET_COUNTER(huart3.hdmarx));
  if (dma_write_index >= K230_DMA_BUFFER_SIZE)
  {
    dma_write_index = 0U;
  }

  while (dma_read_index != dma_write_index)
  {
    parse_byte(dma_buffer[dma_read_index]);
    dma_read_index++;
    if (dma_read_index >= K230_DMA_BUFFER_SIZE)
    {
      dma_read_index = 0U;
    }
  }
}

uint8_t K230Vision_GetLatest(K230VisionData *data)
{
  if ((data == NULL) || (latest.frame_count == 0U))
  {
    return 0U;
  }

  *data = latest;
  return 1U;
}

/*
 * USART3发生溢出、帧错误或噪声错误时只在中断中置标志。DMA停止与重启放在
 * 主循环执行，避免在中断中调用复杂HAL流程。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if ((huart != NULL) && (huart->Instance == USART3))
  {
    restart_required = 1U;
  }
  else
  {
    CarLink_OnUartError(huart);
  }
}
