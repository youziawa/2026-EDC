#include "telecom.h"
#include "usart.h"
#include "location.h"
#include "lxs1_protocol.h"
#include "decision.h"

/*
 * USART6：SLAM原协议长度。
 */
#define RX1_FRAME_LEN 10U
#define SLAM_POSE_TIMEOUT_MS 300U

/*
 * USART2：地面站旧协议长度。
 * USART2保留给地面站，不再解析小车LXS1帧。
 */
#define RX2_FRAME_LEN 10U
#define USART2_RX_RING_SIZE 128U
#define USART2_RX_RING_MASK (USART2_RX_RING_SIZE - 1U)
#define USART2_MAX_BYTES_PER_UPDATE 64U

/* UART5：小车LXS1链路。 */
#define UART5_RX_RING_SIZE 128U
#define UART5_RX_RING_MASK (UART5_RX_RING_SIZE - 1U)
#define UART5_MAX_BYTES_PER_UPDATE 64U

/*
 * USART3环形接收缓冲区。
 *
 * 大小必须是2的整数次幂，
 * 因为下面使用位运算代替取模。
 */
#define USART3_RX_RING_SIZE 128U
#define USART3_RX_RING_MASK (USART3_RX_RING_SIZE - 1U)

/*
 * 每次Telecom_Update最多处理64个字节，
 * 避免串口持续收到数据时主循环被长期占用。
 */
#define USART3_MAX_BYTES_PER_UPDATE 64U

/* K230实际发送的VISION_TARGET固定数据长度。 */
#define VISION_TARGET_DATA_LEN 12U

/* 视觉结果最大有效时间。 */
#define VISION_TIMEOUT_MS 1200U

/* K230 TRACK frame minimum confidence, 0..1000. */
#define VISION_MIN_CONFIDENCE 450U

/*
 * K230每秒发送一次心跳。
 * 连续2500ms没有收到合法心跳或视觉帧时，认为链路断开。
 */
#define K230_LINK_TIMEOUT_MS 2500U

/* Retry the K230 detection-start command until visual takeover succeeds. */
#define K230_VISION_START_RETRY_MS 1000U

/* Retry until CAR_STATE confirms that the car left its waiting speed mode. */
#define CAR_ACCEL_RETRY_MS 200U
#define CAR_CMD_ACCELERATE 1U
#define CAR_ACCEL_SPEED_MM_S 150U
#define CAR_SPEED_MODE_WAITING_FOR_AIRCRAFT 4U
#define CAR_STATE_DATA_LEN 13U
#define CAR_STATE_TIMEOUT_MS 500U

/*
 * USART1：凌霄旧匿名0x05高度帧。
 *
 * AA FF 05 09
 * ALT_FU[4]
 * HIGH[4]
 * ALT_STA[1]
 * SC AC
 */
#define USART1_HIGH_FRAME_LEN  15U
#define USART1_HIGH_DATA_LEN   9U
#define USART1_HIGH_TIMEOUT_MS 300U

/*
 * USART1环形缓冲区。
 * 大小必须是2的整数次幂。
 */
#define USART1_RX_RING_SIZE 128U
#define USART1_RX_RING_MASK (USART1_RX_RING_SIZE - 1U)

/* 每次Telecom_Update最多处理64个USART1字节。 */
#define USART1_MAX_BYTES_PER_UPDATE 64U

/*
 * 通信层高度合理范围，不是飞机目标高度范围。
 */
#define USART1_HIGH_MIN_CM 0L
#define USART1_HIGH_MAX_CM 500L

/*
 * USART6接收缓存。
 */
static uint8_t rx1_buf[RX1_FRAME_LEN];
static uint8_t rx1_index = 0U;

/*
 * USART2接收缓存。
 */
static uint8_t rx2_buf[RX2_FRAME_LEN];
static uint8_t rx2_index = 0U;
static uint8_t usart2_rx_byte = 0U;
static uint8_t usart2_rx_ring[USART2_RX_RING_SIZE];
static volatile uint16_t usart2_rx_head = 0U;
static volatile uint16_t usart2_rx_tail = 0U;
static volatile uint8_t usart2_parser_reset_requested = 0U;
static volatile uint32_t usart2_rx_overflow_count = 0U;
static volatile uint32_t usart2_rx_error_count = 0U;
static volatile uint32_t usart2_rx_byte_count = 0U;

/* UART5独立接收小车LXS1数据，不与USART2地面站数据混用。 */
static uint8_t uart5_rx_byte = 0U;
static uint8_t uart5_rx_ring[UART5_RX_RING_SIZE];
static volatile uint16_t uart5_rx_head = 0U;
static volatile uint16_t uart5_rx_tail = 0U;
static volatile uint8_t uart5_parser_reset_requested = 0U;
static volatile uint32_t uart5_rx_overflow_count = 0U;
static volatile uint32_t uart5_rx_error_count = 0U;
static volatile uint32_t uart5_rx_byte_count = 0U;
static volatile uint32_t uart5_task_start_count = 0U;
static volatile uint32_t uart5_bad_task_count = 0U;
static lxs1_parser_t uart5_parser;
static lxs1_frame_t uart5_frame;

/*
 * USART1每次中断接收一个字节。
 */
static uint8_t usart1_rx_byte = 0U;

/*
 * USART1单生产者、单消费者环形缓冲区：
 * head由串口中断更新；
 * tail由主循环更新。
 */
static uint8_t usart1_rx_ring[USART1_RX_RING_SIZE];
static volatile uint16_t usart1_rx_head = 0U;
static volatile uint16_t usart1_rx_tail = 0U;

/*
 * 中断与主循环共享的状态。
 */
static volatile uint8_t usart1_parser_reset_requested = 0U;
static volatile uint32_t usart1_rx_overflow_count = 0U;
static volatile uint32_t usart1_rx_error_count = 0U;

/*
 * 可在Keil Watch窗口观察的统计变量。
 */
static volatile uint32_t usart1_good_frame_count = 0U;
static volatile uint32_t usart1_bad_format_count = 0U;
static volatile uint32_t usart1_bad_checksum_count = 0U;
static volatile uint32_t usart1_bad_payload_count = 0U;

/*
 * 凌霄固定15字节高度帧解析器。
 */
typedef struct
{
    uint8_t frame[USART1_HIGH_FRAME_LEN];
    uint8_t index;
} Usart1HighParser_t;

static Usart1HighParser_t usart1_high_parser;

/*
 * USART3每次中断接收一个字节。
 */
static uint8_t usart3_rx_byte = 0U;

/*
 * USART3环形缓冲区。
 *
 * head由中断更新；
 * tail由主循环更新。
 */
static uint8_t usart3_rx_ring[USART3_RX_RING_SIZE];
static volatile uint16_t usart3_rx_head = 0U;
static volatile uint16_t usart3_rx_tail = 0U;

/*
 * 串口调试计数。
 * 可在Keil的Watch窗口中观察。
 */
static volatile uint32_t usart3_rx_overflow_count = 0U;
static volatile uint32_t usart3_rx_error_count = 0U;

/*
 * 串口错误发生后，由中断置1，
 * 主循环负责真正重置LXS1解析器。
 */
static volatile uint8_t usart3_parser_reset_requested = 0U;

/* USART3独立的LXS1解析器和输出帧。 */
static lxs1_parser_t usart3_parser;
static lxs1_frame_t usart3_frame;

/*
 * 最近一次收到合法K230帧的时间。
 * 心跳帧和视觉帧都会更新。
 */
static uint32_t usart3_last_k230_tick = 0U;

/* 是否曾经收到过合法K230帧。 */
static uint8_t usart3_k230_seen = 0U;

/*
 * USART3调试计数，可以在Keil Watch窗口观察。
 */
static volatile uint32_t usart3_heartbeat_count = 0U;
static volatile uint32_t usart3_target_count = 0U;
static volatile uint32_t usart3_bad_payload_count = 0U;
static volatile uint32_t usart3_rx_byte_count = 0U;
/* decision.c中的任务变量。 */

/* USART6的SLAM位姿。 */
PoseData_t pose_data = {0};

/* USART3视觉目标结果。 */
VisionTargetData_t vision_target = {0};

/* USART3 TX storage must remain valid until HAL_UART_Transmit_IT completes. */
static uint8_t usart3_vision_start_tx_buf[LXS1_MAX_FRAME];
static uint32_t usart3_vision_start_last_tick = 0U;

/* UART5 asynchronous TX storage and retry state for the car command. */
static uint8_t uart5_car_cmd_tx_buf[LXS1_MAX_FRAME];
static uint32_t uart5_car_cmd_last_tick = 0U;
static uint8_t uart5_car_cmd_run_id = 0U;
static uint8_t uart5_car_cmd_send_count = 0U;
static uint8_t uart5_car_accel_acknowledged = 0U;

/* USART1接收到的凌霄实际激光高度。 */
FlightHeightData_t flight_height = {0};
CarStatusData_t car_status = {0};

/*
 * 为了兼容transmit.c现有代码，
 * 继续保留board和camera_number。
 */
uint8_t board = 0U;
uint8_t camera_number = 0U;

/*
 * 从小端字节流读取uint16。
 */
static uint16_t Telecom_ReadU16LE(const uint8_t *data)
{
    return (uint16_t)(
        ((uint16_t)data[0]) |
        ((uint16_t)data[1] << 8));
}

/*
 * 从小端字节流读取int16。
 */
static int16_t Telecom_ReadS16LE(const uint8_t *data)
{
    return (int16_t)Telecom_ReadU16LE(data);
}

/*
 * 从小端字节流读取int32。
 */
static int32_t Telecom_ReadS32LE(const uint8_t *data)
{
    uint32_t raw;

    raw  =  (uint32_t)data[0];
    raw |= ((uint32_t)data[1] << 8);
    raw |= ((uint32_t)data[2] << 16);
    raw |= ((uint32_t)data[3] << 24);

    return (int32_t)raw;
}

static uint32_t Telecom_ReadU32LE(const uint8_t *data)
{
    uint32_t value;

    value  =  (uint32_t)data[0];
    value |= ((uint32_t)data[1] << 8);
    value |= ((uint32_t)data[2] << 16);
    value |= ((uint32_t)data[3] << 24);
    return value;
}

uint8_t Telecom_IsPoseValid(void)
{
    if (pose_data.valid == 0U)
    {
        return 0U;
    }

    if ((uint32_t)(HAL_GetTick() - pose_data.rx_tick) >
        SLAM_POSE_TIMEOUT_MS)
    {
        return 0U;
    }

    return 1U;
}

/*
 * 当前字节不符合预期时重新同步。
 *
 * 如果当前字节本身是0xAA，则将其保留为下一帧帧头。
 */
static void Telecom_Usart1ParserResync(uint8_t byte)
{
    usart1_high_parser.index = 0U;

    if (byte == 0xAAU)
    {
        usart1_high_parser.frame[0] = byte;
        usart1_high_parser.index = 1U;
    }
}

/*
 * 判断凌霄实际高度是否仍然有效。
 */
uint8_t Telecom_IsFlightHeightValid(void)
{
    uint32_t elapsed_ms;

    if (flight_height.valid == 0U)
    {
        return 0U;
    }

    /*
     * unsigned减法可以正确处理HAL_GetTick自然溢出。
     */
    elapsed_ms = (uint32_t)(
        HAL_GetTick() - flight_height.rx_tick);

    if (elapsed_ms > USART1_HIGH_TIMEOUT_MS)
    {
        return 0U;
    }

    return 1U;
}

/*
 * 安全取得一份实际激光高度。
 */
uint8_t Telecom_GetFlightHeightCm(int32_t *height_cm)
{
    if (height_cm == NULL)
    {
        return 0U;
    }

    if (Telecom_IsFlightHeightValid() == 0U)
    {
        return 0U;
    }

    *height_cm = flight_height.height_cm;
    return 1U;
}

/*
 * 向固定15字节高度帧解析器送入一个字节。
 *
 * 帧格式：
 * AA FF 05 09
 * 00 00 00 00
 * H0 H1 H2 H3
 * 01
 * SC AC
 *
 * 本函数只能由主循环调用，不能放入串口中断。
 */
static void Telecom_Usart1ParserPush(uint8_t byte)
{
    uint8_t i;
    uint8_t sc;
    uint8_t ac;
    uint8_t last_byte;
    int32_t height_cm;

    /* 等待帧头AA。 */
    if (usart1_high_parser.index == 0U)
    {
        if (byte == 0xAAU)
        {
            usart1_high_parser.frame[0] = byte;
            usart1_high_parser.index = 1U;
        }

        return;
    }

    /* 检查目标地址FF。 */
    if (usart1_high_parser.index == 1U)
    {
        if (byte != 0xFFU)
        {
            usart1_bad_format_count++;
            Telecom_Usart1ParserResync(byte);
            return;
        }

        usart1_high_parser.frame[1] = byte;
        usart1_high_parser.index = 2U;
        return;
    }

    /* 检查高度帧ID 05。 */
    if (usart1_high_parser.index == 2U)
    {
        if (byte != 0x05U)
        {
            usart1_bad_format_count++;
            Telecom_Usart1ParserResync(byte);
            return;
        }

        usart1_high_parser.frame[2] = byte;
        usart1_high_parser.index = 3U;
        return;
    }

    /* 检查DATA长度必须为9。 */
    if (usart1_high_parser.index == 3U)
    {
        if (byte != USART1_HIGH_DATA_LEN)
        {
            usart1_bad_format_count++;
            Telecom_Usart1ParserResync(byte);
            return;
        }

        usart1_high_parser.frame[3] = byte;
        usart1_high_parser.index = 4U;
        return;
    }

    /*
     * 收集剩余DATA、SC和AC。
     */
    usart1_high_parser.frame[usart1_high_parser.index] =
        byte;

    usart1_high_parser.index++;

    if (usart1_high_parser.index <
        USART1_HIGH_FRAME_LEN)
    {
        return;
    }

    last_byte =
        usart1_high_parser.frame[
            USART1_HIGH_FRAME_LEN - 1U];

    /*
     * 双累加校验。
     * frame[0]～frame[12]参与校验；
     * frame[13]是SC；
     * frame[14]是AC。
     */
    sc = 0U;
    ac = 0U;

    for (i = 0U; i < 13U; i++)
    {
        sc += usart1_high_parser.frame[i];
        ac += sc;
    }

    if ((sc != usart1_high_parser.frame[13]) ||
        (ac != usart1_high_parser.frame[14]))
    {
        /*
         * 校验失败时不覆盖当前高度。
         * 原高度最多保留300ms，随后自动失效。
         */
        usart1_bad_checksum_count++;
    }
    else
    {
        /*
         * ALT_STA在frame[12]。
         * 凌霄发送函数固定填1。
         */
        if (usart1_high_parser.frame[12] != 1U)
        {
            flight_height.valid = 0U;
            usart1_bad_payload_count++;
        }
        else
        {
            /*
             * HIGH位于frame[8]～frame[11]，
             * 即ALT_ADD字段，s32小端。
             */
            height_cm = Telecom_ReadS32LE(
                &usart1_high_parser.frame[8]);

            if ((height_cm < USART1_HIGH_MIN_CM) ||
                (height_cm > USART1_HIGH_MAX_CM))
            {
                /*
                 * 收到带正确校验但明显异常的高度，
                 * 立即将高度标记为无效。
                 */
                flight_height.valid = 0U;
                usart1_bad_payload_count++;
            }
            else
            {
                /*
                 * 先写高度和时间，最后置valid。
                 */
                flight_height.valid = 0U;
                flight_height.height_cm = height_cm;
                flight_height.rx_tick = HAL_GetTick();
                flight_height.valid = 1U;

                usart1_good_frame_count++;
            }
        }
    }

    /*
     * 一帧处理结束，重新寻找帧头。
     * 如果最后一个字节碰巧是AA，将其保留用于重新同步。
     */
    Telecom_Usart1ParserResync(last_byte);
}

/*
 * 在主循环中处理USART1环形缓冲区。
 */
static void Telecom_UpdateUsart1(void)
{
    uint8_t byte;
    uint16_t processed;
    uint32_t saved_primask;

    /*
     * 串口错误或环形缓冲区溢出后，
     * 丢弃残缺内容并重置高度帧解析器。
     */
    if (usart1_parser_reset_requested != 0U)
    {
        /*
         * 保存进入函数前的中断状态。
         */
        saved_primask = __get_PRIMASK();
        __disable_irq();

        usart1_rx_tail = usart1_rx_head;
        usart1_parser_reset_requested = 0U;

        if (saved_primask == 0U)
        {
            __enable_irq();
        }

        usart1_high_parser.index = 0U;
    }

    processed = 0U;

    /*
     * 每轮最多处理64字节，避免长期占用主循环。
     */
    while ((usart1_rx_tail != usart1_rx_head) &&
           (processed <
            USART1_MAX_BYTES_PER_UPDATE))
    {
        byte = usart1_rx_ring[usart1_rx_tail];

        usart1_rx_tail = (uint16_t)(
            (usart1_rx_tail + 1U) &
            USART1_RX_RING_MASK);

        processed++;

        Telecom_Usart1ParserPush(byte);
    }

    /*
     * 超过300ms没有合法高度，主动清除有效标志。
     */
    if ((flight_height.valid != 0U) &&
        (Telecom_IsFlightHeightValid() == 0U))
    {
        flight_height.valid = 0U;
    }
}

/*
 * 处理一帧完整的USART3 LXS1消息。
 *
 * K230实际发送两种消息：
 * HEARTBEAT     MSG=0x02，DATA长度=0；
 * VISION_TARGET MSG=0x40，DATA长度=12。
 */
static void Telecom_HandleUsart3Frame(const lxs1_frame_t *frame)
{
    const uint8_t *data;
    uint32_t now_tick;

    if (frame == NULL)
    {
        return;
    }

    /* USART3只接受天空K230发送的数据。 */
    if (frame->src != LXS1_NODE_AIR_K230)
    {
        return;
    }

    /* 只接受发给飞机F407或广播地址的帧。 */
    if ((frame->dst != LXS1_NODE_AIR_MCU) &&
        (frame->dst != LXS1_NODE_BROADCAST))
    {
        return;
    }

    now_tick = HAL_GetTick();

    /*
     * K230心跳帧：
     * AA 55 03 03 02 02 0D 0A
     */
    if (frame->msg_id == LXS1_MSG_HEARTBEAT)
    {
        if (frame->data_len != 0U)
        {
            usart3_bad_payload_count++;
            return;
        }

        usart3_last_k230_tick = now_tick;
        usart3_k230_seen = 1U;
        usart3_heartbeat_count++;
        return;
    }

    /* 其他未知消息不参与平台识别。 */
    if (frame->msg_id != LXS1_MSG_VISION_TARGET)
    {
        return;
    }

    /* K230的VISION_TARGET数据区必须正好12字节。 */
    if (frame->data_len != VISION_TARGET_DATA_LEN)
    {
        usart3_bad_payload_count++;
        return;
    }

    /*
     * 长度正确的视觉帧也能证明K230链路在线。
     * 注意：链路在线不会延长旧视觉坐标的1200ms有效期。
     */
    usart3_last_k230_tick = now_tick;
    usart3_k230_seen = 1U;
    usart3_target_count++;

    data = frame->data;

    /*
     * Actual K230 DATA layout:
     * data[0]       valid
     * data[1]       target_kind, fixed at 2
     * data[2..3]    confidence, uint16 little-endian
     * data[4..5]    dx_px, int16 little-endian
     * data[6..7]    dy_px, int16 little-endian
     * data[8..9]    radius_px, int16 little-endian
     * data[10..11]  vision_mode, int16 little-endian
     *
     * Invalid K230 frames deliberately contain zero coordinates. Keep the
     * most recent valid pixel measurement for diagnostics, but set valid=0
     * immediately so stale coordinates can never drive horizontal control.
     */
    vision_target.target_kind = data[1];

    vision_target.confidence =
        Telecom_ReadU16LE(&data[2]);

    vision_target.vision_mode =
        Telecom_ReadS16LE(&data[10]);

    vision_target.rx_tick = now_tick;
    vision_target.valid = 0U;

    /* Reject malformed target metadata before control sees it. */
    if (vision_target.target_kind !=
        VISION_TARGET_KIND_PLATFORM)
    {
        return;
    }

    if (vision_target.confidence > 1000U)
    {
        return;
    }

    if ((vision_target.vision_mode < VISION_MODE_ACQUIRE) ||
        (vision_target.vision_mode > VISION_MODE_LOST))
    {
        return;
    }

    if (data[0] == 1U)
    {
        vision_target.dx_px =
            Telecom_ReadS16LE(&data[4]);

        vision_target.dy_px =
            Telecom_ReadS16LE(&data[6]);

        vision_target.radius_px =
            Telecom_ReadS16LE(&data[8]);

        if ((vision_target.vision_mode == VISION_MODE_TRACK) &&
            (vision_target.radius_px > 0))
        {
            vision_target.valid = 1U;
        }
    }

    /*
     * 这份K230平台协议不包含board_id和camera_id。
     * 保留旧全局变量只是为了不破坏transmit.c编译。
     */
    board = 0U;
    camera_number = 0U;
}

/*
 * 判断当前视觉结果是否仍然有效。
 *
 * K230不发送age_ms，因此只使用F407本地接收时间。
 */
uint8_t Telecom_IsVisionValid(void)
{
    uint32_t elapsed_ms;

    if (vision_target.valid == 0U)
    {
        return 0U;
    }

    if (vision_target.target_kind !=
        VISION_TARGET_KIND_PLATFORM)
    {
        return 0U;
    }

    if (vision_target.vision_mode != VISION_MODE_TRACK)
    {
        return 0U;
    }

    if ((vision_target.confidence < VISION_MIN_CONFIDENCE) ||
        (vision_target.confidence > 1000U))
    {
        return 0U;
    }

    if (vision_target.radius_px <= 0)
    {
        return 0U;
    }

    elapsed_ms =
        (uint32_t)(HAL_GetTick() - vision_target.rx_tick);

    if (elapsed_ms > VISION_TIMEOUT_MS)
    {
        return 0U;
    }

    return 1U;
}

/*
 * 判断天空K230通信链路是否在线。
 *
 * 心跳或视觉帧都可以维持链路在线状态；
 * 该状态与视觉目标的1200ms有效期相互独立。
 */
uint8_t Telecom_IsK230Online(void)
{
    uint32_t elapsed_ms;

    if (usart3_k230_seen == 0U)
    {
        return 0U;
    }

    elapsed_ms =
        (uint32_t)(HAL_GetTick() -
                   usart3_last_k230_tick);

    if (elapsed_ms > K230_LINK_TIMEOUT_MS)
    {
        return 0U;
    }

    return 1U;
}

uint32_t Telecom_GetK230RxByteCount(void)
{
    return usart3_rx_byte_count;
}

uint32_t Telecom_GetK230ParsedFrameCount(void)
{
    return usart3_parser.frames_ok;
}

uint32_t Telecom_GetK230TargetFrameCount(void)
{
    return usart3_target_count;
}

uint32_t Telecom_GetK230HeartbeatCount(void)
{
    return usart3_heartbeat_count;
}

uint32_t Telecom_GetK230UartErrorCount(void)
{
    return usart3_rx_error_count;
}

uint32_t Telecom_GetK230BadPayloadCount(void)
{
    return usart3_bad_payload_count;
}

static void Telecom_HandleUart5Frame(const lxs1_frame_t *frame)
{
    if (frame == NULL)
    {
        return;
    }

    if ((frame->src != LXS1_NODE_CAR_MCU) ||
        ((frame->dst != LXS1_NODE_AIR_MCU) &&
         (frame->dst != LXS1_NODE_BROADCAST)))
    {
        return;
    }

    /* CarF407 DATA: run_id, task_mode, normal_speed, action_speed. */
    if (frame->msg_id == LXS1_MSG_TASK_START)
    {
        if ((frame->data_len != 6U) ||
            ((frame->data[1] != AIR_TASK_DROP) &&
             (frame->data[1] != AIR_TASK_DYNAMIC_LANDING)))
        {
            uart5_bad_task_count++;
            return;
        }

        uart5_task_start_count++;
        Decision_OnTaskStart(frame->data[0], frame->data[1]);
    }
    else if (frame->msg_id == LXS1_MSG_TASK_ABORT)
    {
        if (frame->data_len < 1U)
        {
            uart5_bad_task_count++;
            return;
        }

        Decision_OnTaskAbort(frame->data[0]);
    }
    else if (frame->msg_id == LXS1_MSG_CAR_STATE)
    {
        /*
         * CarF407 CAR_STATE DATA begins with run_id, state, speed_mode.
         * speed_mode 4 means it is still waiting for the aircraft catch.
         * Any later mode for the same run is an idempotent acceleration ACK.
         */
        if (frame->data_len == CAR_STATE_DATA_LEN)
        {
            car_status.valid = 0U;
            car_status.run_id = frame->data[0];
            car_status.state = frame->data[1];
            car_status.speed_mode = frame->data[2];
            car_status.line_valid = frame->data[3];
            car_status.speed_mm_s = Telecom_ReadU16LE(&frame->data[4]);
            car_status.fault = Telecom_ReadU16LE(&frame->data[6]);
            car_status.path_mm = Telecom_ReadU32LE(&frame->data[8]);
            car_status.finish_marker = frame->data[12];
            car_status.rx_tick = HAL_GetTick();
            car_status.valid = 1U;
        }

        if ((frame->data_len >= 3U) &&
            (frame->data[0] == uart5_car_cmd_run_id) &&
            (uart5_car_cmd_send_count != 0U) &&
            (frame->data[2] != CAR_SPEED_MODE_WAITING_FOR_AIRCRAFT))
        {
            uart5_car_accel_acknowledged = 1U;
        }
    }
}

/* Keep the old FE coordinate parser isolated from the LXS1 parser. */
static void Telecom_ProcessUsart2LegacyByte(uint8_t byte)
{
    if (rx2_index == 0U)
    {
        if (byte != 0xFEU)
        {
            return;
        }
    }

    rx2_buf[rx2_index++] = byte;
    if (rx2_index < RX2_FRAME_LEN)
    {
        return;
    }

    rx2_index = 0U;

    if ((rx2_buf[0] == 0xFEU) &&
        (rx2_buf[3] == 0xFFU))
    {
        decision = 1;
        target_location.x = (int16_t)(
            ((uint16_t)rx2_buf[1] << 8) |
            rx2_buf[2]);
    }

    if ((rx2_buf[0] == 0xFEU) &&
        (rx2_buf[6] == 0xFFU))
    {
        decision = 2;
        target_location.y = (int16_t)(
            ((uint16_t)rx2_buf[4] << 8) |
            rx2_buf[5]);
    }
}

uint32_t Telecom_GetCarRxByteCount(void)
{
    return uart5_rx_byte_count;
}

uint32_t Telecom_GetCarTaskStartCount(void)
{
    return uart5_task_start_count;
}

uint32_t Telecom_GetCarBadTaskCount(void)
{
    return uart5_bad_task_count;
}

uint32_t Telecom_GetCarUartErrorCount(void)
{
    return uart5_rx_error_count;
}

uint8_t Telecom_GetCarAccelerationSendCount(void)
{
    return uart5_car_cmd_send_count;
}

uint8_t Telecom_IsCarAccelerationAcknowledged(void)
{
    return uart5_car_accel_acknowledged;
}

uint8_t Telecom_IsCarStatusValid(void)
{
    if ((car_status.valid == 0U) ||
        ((uint32_t)(HAL_GetTick() - car_status.rx_tick) >
         CAR_STATE_TIMEOUT_MS))
    {
        return 0U;
    }
    return 1U;
}

const CarStatusData_t *Telecom_GetCarStatus(void)
{
    return &car_status;
}

/*
 * 初始化USART1凌霄高度接收。
 */
void Telecom_USART1_Init(void)
{
    usart1_rx_head = 0U;
    usart1_rx_tail = 0U;
    usart1_parser_reset_requested = 0U;

    usart1_rx_overflow_count = 0U;
    usart1_rx_error_count = 0U;
    usart1_good_frame_count = 0U;
    usart1_bad_format_count = 0U;
    usart1_bad_checksum_count = 0U;
    usart1_bad_payload_count = 0U;

    usart1_high_parser.index = 0U;

    flight_height.height_cm = 0;
    flight_height.rx_tick = HAL_GetTick();
    flight_height.valid = 0U;

    /*
     * 启动USART1第一个字节的中断接收。
     */
    (void)HAL_UART_Receive_IT(
        &huart1,
        &usart1_rx_byte,
        1U);
}

/*
 * 初始化USART6的SLAM接收。
 * 保持原有逐字节中断接收方式。
 */
void Telecom_USART6_Init(void)
{
    rx1_index = 0U;
    pose_data.x = 0;
    pose_data.y = 0;
    pose_data.z = 0;
    pose_data.yaw = 0;
    pose_data.rx_tick = HAL_GetTick();
    pose_data.valid = 0U;

    (void)HAL_UART_Receive_IT(
        &huart6,
        &rx1_buf[rx1_index],
        1U);
}

/*
 * 初始化USART2的地面站接收。
 * 当前继续保留FE旧协议。
 */
void Telecom_USART2_Init(void)
{
    rx2_index = 0U;
    usart2_rx_head = 0U;
    usart2_rx_tail = 0U;
    usart2_parser_reset_requested = 0U;
    usart2_rx_overflow_count = 0U;
    usart2_rx_error_count = 0U;
    usart2_rx_byte_count = 0U;

    (void)HAL_UART_Receive_IT(
        &huart2,
        &usart2_rx_byte,
        1U);
}

/* 初始化UART5的小车LXS1接收。 */
void Telecom_UART5_Init(void)
{
    uart5_rx_head = 0U;
    uart5_rx_tail = 0U;
    uart5_parser_reset_requested = 0U;
    uart5_rx_overflow_count = 0U;
    uart5_rx_error_count = 0U;
    uart5_rx_byte_count = 0U;
    uart5_task_start_count = 0U;
    uart5_bad_task_count = 0U;
    uart5_car_cmd_last_tick = 0U;
    uart5_car_cmd_run_id = 0U;
    uart5_car_cmd_send_count = 0U;
    uart5_car_accel_acknowledged = 0U;
    car_status.valid = 0U;
    car_status.rx_tick = HAL_GetTick();
    lxs1_parser_init(&uart5_parser);

    (void)HAL_UART_Receive_IT(
        &huart5,
        &uart5_rx_byte,
        1U);
}

/*
 * 初始化USART3的LXS1视觉通信。
 */
void Telecom_USART3_Init(void)
{
    usart3_rx_head = 0U;
    usart3_rx_tail = 0U;

    usart3_rx_overflow_count = 0U;
    usart3_rx_error_count = 0U;
    usart3_parser_reset_requested = 0U;

    usart3_k230_seen = 0U;
    usart3_last_k230_tick = HAL_GetTick();
    usart3_heartbeat_count = 0U;
    usart3_target_count = 0U;
    usart3_bad_payload_count = 0U;
    usart3_rx_byte_count = 0U;

    board = 0U;
    camera_number = 0U;

    vision_target.valid = 0U;
    vision_target.target_kind =
        VISION_TARGET_KIND_PLATFORM;
    vision_target.confidence = 0U;
    vision_target.dx_px = 0;
    vision_target.dy_px = 0;
    vision_target.radius_px = 0;
    vision_target.vision_mode = VISION_MODE_LOST;
    vision_target.rx_tick = HAL_GetTick();
    usart3_vision_start_last_tick = 0U;

    /*
     * 初始化LXS1状态机。
     */
    lxs1_parser_init(&usart3_parser);

    /*
     * 启动USART3第一个字节接收。
     */
    (void)HAL_UART_Receive_IT(
        &huart3,
        &usart3_rx_byte,
        1U);
}

void Telecom_RequestVisionStart(void)
{
    lxs1_frame_t frame;
    size_t tx_len;
    uint32_t now_tick = HAL_GetTick();

    if ((uint32_t)(now_tick - usart3_vision_start_last_tick) <
        K230_VISION_START_RETRY_MS)
    {
        return;
    }

    if (huart3.gState != HAL_UART_STATE_READY)
    {
        return;
    }

    frame.src = LXS1_NODE_AIR_MCU;
    frame.dst = LXS1_NODE_AIR_K230;
    frame.msg_id = LXS1_MSG_TASK_START;
    frame.data_len = 1U;
    frame.data[0] = VISION_TARGET_KIND_PLATFORM;

    tx_len = lxs1_encode(
        &frame,
        usart3_vision_start_tx_buf,
        sizeof(usart3_vision_start_tx_buf));

    if ((tx_len != 0U) &&
        (HAL_UART_Transmit_IT(
             &huart3,
             usart3_vision_start_tx_buf,
             (uint16_t)tx_len) == HAL_OK))
    {
        usart3_vision_start_last_tick = now_tick;
    }
}

void Telecom_RequestCarAccelerate(void)
{
    lxs1_frame_t frame;
    size_t tx_len;
    uint8_t run_id = Decision_GetRunId();
    uint32_t now_tick = HAL_GetTick();

    if ((run_id == 0U) || (Decision_IsTaskActive() == 0U))
    {
        return;
    }

    if (uart5_car_cmd_run_id != run_id)
    {
        uart5_car_cmd_run_id = run_id;
        uart5_car_cmd_send_count = 0U;
        uart5_car_cmd_last_tick = 0U;
        uart5_car_accel_acknowledged = 0U;
    }

    if ((uart5_car_accel_acknowledged != 0U) ||
        ((uint32_t)(now_tick - uart5_car_cmd_last_tick) < CAR_ACCEL_RETRY_MS) ||
        (huart5.gState != HAL_UART_STATE_READY))
    {
        return;
    }

    frame.src = LXS1_NODE_AIR_MCU;
    frame.dst = LXS1_NODE_CAR_MCU;
    frame.msg_id = LXS1_MSG_CAR_CMD;
    frame.data_len = 4U;
    frame.data[0] = run_id;
    frame.data[1] = CAR_CMD_ACCELERATE;
    frame.data[2] = (uint8_t)(CAR_ACCEL_SPEED_MM_S & 0xFFU);
    frame.data[3] = (uint8_t)(CAR_ACCEL_SPEED_MM_S >> 8);

    tx_len = lxs1_encode(
        &frame, uart5_car_cmd_tx_buf, sizeof(uart5_car_cmd_tx_buf));
    if ((tx_len != 0U) &&
        (HAL_UART_Transmit_IT(&huart5, uart5_car_cmd_tx_buf,
                             (uint16_t)tx_len) == HAL_OK))
    {
        uart5_car_cmd_last_tick = now_tick;
        if (uart5_car_cmd_send_count < 0xFFU)
        {
            uart5_car_cmd_send_count++;
        }
    }
}

/*
 * HAL串口接收完成回调。
 *
 * USART1、USART2、USART3、UART5、USART6共用该回调，
 * 根据串口实例分别处理。
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        uint16_t next_head;

        /*
         * USART1中断中只搬运字节。
         * 不解析协议、不打印、不延时。
         */
        next_head = (uint16_t)(
            (usart1_rx_head + 1U) &
            USART1_RX_RING_MASK);

        if (next_head != usart1_rx_tail)
        {
            usart1_rx_ring[usart1_rx_head] =
                usart1_rx_byte;

            /*
             * 字节写完以后再更新head。
             */
            usart1_rx_head = next_head;
        }
        else
        {
            /*
             * 环形缓冲区已满，丢弃本字节，
             * 并要求主循环重置解析器。
             */
            usart1_rx_overflow_count++;
            usart1_parser_reset_requested = 1U;
        }

        /*
         * 立即接收USART1下一个字节。
         * USART1的TX/RX相互独立，不影响原控制帧发送。
         */
        (void)HAL_UART_Receive_IT(
            &huart1,
            &usart1_rx_byte,
            1U);
    }
    else if (huart->Instance == USART6)
    {
        /*
         * USART6：SLAM原协议。
         */
        rx1_index++;

        if ((rx1_index == 1U) &&
            (rx1_buf[0] != 0xAAU))
        {
            rx1_index = 0U;
        }

        if (rx1_index >= RX1_FRAME_LEN)
        {
            rx1_index = 0U;

            if ((rx1_buf[0] == 0xAAU) &&
                (rx1_buf[9] == 0x0AU))
            {
                /*
                 * USART6原协议为大端int16。
                 */
                pose_data.valid = 0U;

                pose_data.x = (int16_t)(
                    ((uint16_t)rx1_buf[1] << 8) |
                    rx1_buf[2]);

                pose_data.y = (int16_t)(
                    ((uint16_t)rx1_buf[3] << 8) |
                    rx1_buf[4]);

                pose_data.z = (int16_t)(
                    ((uint16_t)rx1_buf[5] << 8) |
                    rx1_buf[6]);

                pose_data.yaw = (int16_t)(
                    ((uint16_t)rx1_buf[7] << 8) |
                    rx1_buf[8]);

                pose_data.rx_tick = HAL_GetTick();
                pose_data.valid = 1U;
            }
        }

        /*
         * 继续接收USART6下一个字节。
         */
        (void)HAL_UART_Receive_IT(
            &huart6,
            &rx1_buf[rx1_index],
            1U);
    }
    else if (huart->Instance == USART2)
    {
        /*
         * USART2：地面站FE旧协议。
         */
        uint16_t next_head;

        usart2_rx_byte_count++;

        next_head = (uint16_t)(
            (usart2_rx_head + 1U) & USART2_RX_RING_MASK);

        if (next_head != usart2_rx_tail)
        {
            usart2_rx_ring[usart2_rx_head] = usart2_rx_byte;
            usart2_rx_head = next_head;
        }
        else
        {
            usart2_rx_overflow_count++;
            usart2_parser_reset_requested = 1U;
        }

        /*
         * 继续接收USART2下一个字节。
         */
        (void)HAL_UART_Receive_IT(
            &huart2,
            &usart2_rx_byte,
            1U);
    }
    else if (huart->Instance == UART5)
    {
        uint16_t next_head;

        uart5_rx_byte_count++;

        /* 中断中只搬运小车字节，LXS1解析放在主循环。 */
        next_head = (uint16_t)(
            (uart5_rx_head + 1U) & UART5_RX_RING_MASK);

        if (next_head != uart5_rx_tail)
        {
            uart5_rx_ring[uart5_rx_head] = uart5_rx_byte;
            uart5_rx_head = next_head;
        }
        else
        {
            uart5_rx_overflow_count++;
            uart5_parser_reset_requested = 1U;
        }

        (void)HAL_UART_Receive_IT(
            &huart5,
            &uart5_rx_byte,
            1U);
    }
    else if (huart->Instance == USART3)
    {
        uint16_t next_head;

        usart3_rx_byte_count++;

        /*
         * USART3中断内只存储字节。
         * 不在中断中解析LXS1，也不打印。
         */
        next_head = (uint16_t)(
            (usart3_rx_head + 1U) &
            USART3_RX_RING_MASK);

        if (next_head != usart3_rx_tail)
        {
            usart3_rx_ring[usart3_rx_head] =
                usart3_rx_byte;

            /*
             * 数据写入完成后再更新head。
             */
            usart3_rx_head = next_head;
        }
        else
        {
            /*
             * 环形缓冲区已满，丢弃当前字节。
             */
            usart3_rx_overflow_count++;
            usart3_parser_reset_requested = 1U;
        }

        /*
         * 立即挂接USART3下一个字节。
         */
        (void)HAL_UART_Receive_IT(
            &huart3,
            &usart3_rx_byte,
            1U);
    }
}

/*
 * HAL串口错误回调。
 *
 * USART1、USART2、USART3和UART5分别恢复各自接收链路。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        usart1_rx_error_count++;
        usart1_parser_reset_requested = 1U;

        /*
         * ORE会终止当前中断接收，
         * 需要重新挂接USART1接收。
         */
        if ((huart->ErrorCode &
             HAL_UART_ERROR_ORE) != 0U)
        {
            (void)HAL_UART_Receive_IT(
                &huart1,
                &usart1_rx_byte,
                1U);
        }
    }
    else if (huart->Instance == USART2)
    {
        usart2_rx_error_count++;
        usart2_parser_reset_requested = 1U;

        if ((huart->ErrorCode & HAL_UART_ERROR_ORE) != 0U)
        {
            (void)HAL_UART_Receive_IT(
                &huart2,
                &usart2_rx_byte,
                1U);
        }
    }
    else if (huart->Instance == UART5)
    {
        uart5_rx_error_count++;
        uart5_parser_reset_requested = 1U;

        if ((huart->ErrorCode & HAL_UART_ERROR_ORE) != 0U)
        {
            (void)HAL_UART_Receive_IT(
                &huart5,
                &uart5_rx_byte,
                1U);
        }
    }
    else if (huart->Instance == USART3)
    {
        usart3_rx_error_count++;
        usart3_parser_reset_requested = 1U;

        /*
         * ORE是阻塞型接收错误。
         * HAL在进入该回调前已经结束当前接收，
         * 因此需要重新启动接收。
         *
         * FE、NE、PE属于非阻塞错误时，
         * HAL仍会继续当前接收，不需要重复挂接。
         */
        if ((huart->ErrorCode & HAL_UART_ERROR_ORE) != 0U)
        {
            (void)HAL_UART_Receive_IT(
                &huart3,
                &usart3_rx_byte,
                1U);
        }
    }
}

/*
 * 主循环通信处理函数。
 */
void Telecom_Update(void)
{
    uint8_t byte;
    uint16_t processed;
    lxs1_parse_result_t result;

    /*
     * 先处理凌霄实际高度。
     */
    Telecom_UpdateUsart1();

    /* USART2只处理地面站FE旧协议。 */
    if (usart2_parser_reset_requested != 0U)
    {
        __disable_irq();
        usart2_rx_tail = usart2_rx_head;
        usart2_parser_reset_requested = 0U;
        __enable_irq();

        rx2_index = 0U;
    }

    processed = 0U;
    while ((usart2_rx_tail != usart2_rx_head) &&
           (processed < USART2_MAX_BYTES_PER_UPDATE))
    {
        byte = usart2_rx_ring[usart2_rx_tail];
        usart2_rx_tail = (uint16_t)(
            (usart2_rx_tail + 1U) & USART2_RX_RING_MASK);
        processed++;

        Telecom_ProcessUsart2LegacyByte(byte);
    }

    /* UART5只处理小车TASK_START/TASK_ABORT LXS1帧。 */
    if (uart5_parser_reset_requested != 0U)
    {
        __disable_irq();
        uart5_rx_tail = uart5_rx_head;
        uart5_parser_reset_requested = 0U;
        __enable_irq();

        lxs1_parser_init(&uart5_parser);
    }

    processed = 0U;
    while ((uart5_rx_tail != uart5_rx_head) &&
           (processed < UART5_MAX_BYTES_PER_UPDATE))
    {
        byte = uart5_rx_ring[uart5_rx_tail];
        uart5_rx_tail = (uint16_t)(
            (uart5_rx_tail + 1U) & UART5_RX_RING_MASK);
        processed++;

        result = lxs1_parser_push(
            &uart5_parser, byte, &uart5_frame);
        if (result == LXS1_PARSE_FRAME)
        {
            Telecom_HandleUart5Frame(&uart5_frame);
        }
    }

    /*
     * 串口出错或环形缓冲区溢出后，
     * 丢弃残缺字节并重置解析器。
     */
    if (usart3_parser_reset_requested != 0U)
    {
        /*
         * 短暂关中断，避免复制head过程中发生变化。
         */
        __disable_irq();

        usart3_rx_tail = usart3_rx_head;
        usart3_parser_reset_requested = 0U;

        __enable_irq();

        lxs1_parser_init(&usart3_parser);
    }

    processed = 0U;

    /*
     * 每次最多处理64字节。
     * 因此即使USART3持续收到数据，
     * 这里也不会永久占用主循环。
     */
    while ((usart3_rx_tail != usart3_rx_head) &&
           (processed < USART3_MAX_BYTES_PER_UPDATE))
    {
        byte = usart3_rx_ring[usart3_rx_tail];

        usart3_rx_tail = (uint16_t)(
            (usart3_rx_tail + 1U) &
            USART3_RX_RING_MASK);

        processed++;

        result = lxs1_parser_push(
            &usart3_parser,
            byte,
            &usart3_frame);

        if (result == LXS1_PARSE_FRAME)
        {
            Telecom_HandleUsart3Frame(
                &usart3_frame);
        }
    }

    /*
     * 视觉数据超过1200ms后主动失效。
     */
    if ((vision_target.valid != 0U) &&
        (Telecom_IsVisionValid() == 0U))
    {
        vision_target.valid = 0U;
        board = 0U;
        camera_number = 0U;
    }
}

/*
 * 初始化所有通信接口。
 */
void Telecom_Init(void)
{
    /* USART1：凌霄实际激光高度。 */
    Telecom_USART1_Init();

    Telecom_USART6_Init();
    Telecom_USART2_Init();
    Telecom_UART5_Init();
    Telecom_USART3_Init();
}
