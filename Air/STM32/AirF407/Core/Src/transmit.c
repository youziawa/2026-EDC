#include "transmit.h"
#include "usart.h"
#include "telecom.h"
#include "location.h"
#include "move.h"
#include "decision.h"
#include <stdio.h>

// 位姿帧: FB + x/y/z/yaw(各2字节) + 0D
#define POSE_FRAME_LEN 10U

// 降落标识帧: FD 01 0D
#define MISSION_DONE_FRAME_LEN 3U

// 位姿发送周期: 50ms (20Hz)
#define TRANSMIT_POSE_PERIOD_MS 50U

/* Fast two-stage launch, followed by a mandatory 3 s hover at 125 cm. */
#define TAKEOFF_LOW_TARGET_CM 70
#define TAKEOFF_LOW_HOLD_MS   500U
#define TAKEOFF_RAMP_MS       750U
#define TAKEOFF_FINAL_HOVER_MS 3000U


// 位姿发送模式: 0=正常位姿, 1=固定测试位姿(未启用分支会被预处理裁掉)
#define TRANSMIT_POSE_TEST_MODE 0U

// 位姿输出格式: 0=二进制帧(FB + 4*int16 + 0D), 1=ASCII调试帧("FB,x,y,z,yaw\r\n")
#define TRANSMIT_POSE_ASCII_MODE 0U

// 降落标识发送超时(ms), 仅用于USART1阻塞发送
#define MISSION_DONE_TX_TIMEOUT_MS 20U

// 1=允许发送降落标识, 0=关闭降落标识发送
#define TRANSMIT_ENABLE_MISSION_DONE 1U

// USART2自定义发送帧最大长度, 后续可按协议调整
#define USART2_CUSTOM_FRAME_MAX_LEN 32U

#define USART2_POSE_DEBUG_PERIOD_MS 250U
#define USART2_DEBUG_TEXT_MAX_LEN 256U

static uint8_t pose_tx_buf[POSE_FRAME_LEN];                       // 位姿帧发送包
static uint8_t usart2_custom_tx_buf[USART2_CUSTOM_FRAME_MAX_LEN]; // USART2自定义发送缓存

extern target_location_t target_location;


// ASCII模式下的位姿发送包，此时为屏蔽状态
#if (TRANSMIT_POSE_ASCII_MODE == 1U)
static uint8_t pose_ascii_tx_buf[48U];
#endif
static const uint8_t mission_done_buf[MISSION_DONE_FRAME_LEN] = {0xFD, 0x01, 0x0D};

// 最近一次位姿发送时刻, 用于控制发送周期
static uint32_t pose_last_tick = 0U;

uint8_t board1 = 0; // 占位值，代表棋盘面ABCD
uint8_t board2 = 0; // 占位值，代表棋盘格123456

static void PointChoose(void);

/*
 * Before TASK_START, keep Lingxiao's altitude target at ground level.  A run
 * commands 70 cm briefly, ramps continuously to the 125 cm waypoint height,
 * then holds that height while Move keeps horizontal navigation locked.
 */
static int16_t Transmit_GetCommandedHeightCm(void)
{
    uint32_t elapsed_ms;
    int32_t final_height_cm;
    int32_t ramp_height_cm;

    if (Decision_IsTaskActive() == 0U)
    {
        return 0;
    }

    /* Use Move's clock so altitude phases and XY release share one origin. */
    elapsed_ms = Move_GetLaunchElapsedMs();
    if (elapsed_ms < TAKEOFF_LOW_HOLD_MS)
    {
        return TAKEOFF_LOW_TARGET_CM;
    }

    final_height_cm = Location_GetCurrentWaypointZ();
    if (elapsed_ms < (TAKEOFF_LOW_HOLD_MS + TAKEOFF_RAMP_MS))
    {
        elapsed_ms -= TAKEOFF_LOW_HOLD_MS;
        ramp_height_cm = TAKEOFF_LOW_TARGET_CM +
            ((final_height_cm - TAKEOFF_LOW_TARGET_CM) *
             (int32_t)elapsed_ms / (int32_t)TAKEOFF_RAMP_MS);
        return (int16_t)ramp_height_cm;
    }

    return (int16_t)final_height_cm;
}

static const char *Transmit_TakeoffPhaseName(void)
{
    uint32_t elapsed_ms;

    if (Decision_IsTaskActive() == 0U)
    {
        return "READY";
    }

    elapsed_ms = Move_GetLaunchElapsedMs();
    if (elapsed_ms < TAKEOFF_LOW_HOLD_MS)
    {
        return "TAKEOFF_70";
    }
    if (elapsed_ms < (TAKEOFF_LOW_HOLD_MS + TAKEOFF_RAMP_MS))
    {
        return "CLIMB_125";
    }
    if (elapsed_ms < (TAKEOFF_LOW_HOLD_MS + TAKEOFF_RAMP_MS +
                      TAKEOFF_FINAL_HOVER_MS))
    {
        return "HOVER_3S";
    }
    return "ROUTE";
}

// 通用中断发送入口定义: 由调用方指定串口句柄(huart1/huart2)
static uint8_t Transmit_StartRawIT(UART_HandleTypeDef *huart, const uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef status;

    if (huart->gState != HAL_UART_STATE_READY)
    {
        return 0U;
    }

    status = HAL_UART_Transmit_IT(huart, (uint8_t *)buf, len);

    return (uint8_t)(status == HAL_OK ? 1U : 0U);
}

//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////

#if (TRANSMIT_POSE_TEST_MODE == 0U)
// 打包控制帧: x/y/z/yaw字段发送Move输出(-1/0/1)
static void Transmit_PackPoseFrame(void)
{
    const MoveCmd_t *cmd = Move_GetLastCmd();
    int16_t target_height_cm = Transmit_GetCommandedHeightCm();

    pose_tx_buf[0] = 0xFB;
    pose_tx_buf[1] = (uint8_t)((cmd->dx >> 8) & 0xFF);
    pose_tx_buf[2] = (uint8_t)(cmd->dx & 0xFF);
    pose_tx_buf[3] = (uint8_t)((cmd->dy >> 8) & 0xFF);
    pose_tx_buf[4] = (uint8_t)(cmd->dy & 0xFF);
    pose_tx_buf[5] = (uint8_t)((target_height_cm >> 8) & 0xFF);
    pose_tx_buf[6] = (uint8_t)(target_height_cm & 0xFF);
    pose_tx_buf[7] = (uint8_t)((cmd->dyaw >> 8) & 0xFF);
    pose_tx_buf[8] = (uint8_t)(cmd->dyaw & 0xFF);
    pose_tx_buf[9] = 0x0D;
}
#endif

#if (TRANSMIT_POSE_TEST_MODE == 1U)
// 固定测试帧: 帧头帧尾与位姿帧一致, 数据区固定为1/2/3/4。
static void Transmit_PackPoseTestFrame(void)
{

    PointChoose();

    pose_tx_buf[0] = 0xFB;
    pose_tx_buf[1] = (uint8_t)((pose_data.x >> 8) & 0xFF);
    pose_tx_buf[2] = (uint8_t)(pose_data.x & 0xFF);
    pose_tx_buf[3] = (uint8_t)((pose_data.y >> 8) & 0xFF);
    pose_tx_buf[4] = (uint8_t)(pose_data.y & 0xFF);
    pose_tx_buf[5] = (uint8_t)((pose_data.z >> 8) & 0xFF);
    pose_tx_buf[6] = (uint8_t)(pose_data.z & 0xFF);
    pose_tx_buf[7] = (uint8_t)((pose_data.yaw >> 8) & 0xFF);
    pose_tx_buf[8] = (uint8_t)(pose_data.yaw & 0xFF);
    pose_tx_buf[9] = 0x0D;
}

#endif

// 发送模块初始化: 复位统计
void Transmit_Init(void)
{
    pose_last_tick = HAL_GetTick();
}

// 位姿和降落标识均发往凌霄USART1链路
void Transmit_SendPose(void)
{
#if (TRANSMIT_POSE_ASCII_MODE == 1U)
    const MoveCmd_t *cmd = Move_GetLastCmd();
    int len = snprintf((char *)pose_ascii_tx_buf,
                       (unsigned int)sizeof(pose_ascii_tx_buf),
                       "FB,%d,%d,%d,%d\r\n",
                       cmd->dx,
                       cmd->dy,
                       Transmit_GetCommandedHeightCm(),
                       cmd->dyaw);

    if (len > 0)
    {
        (void)Transmit_StartRawIT(&huart1, pose_ascii_tx_buf, (uint16_t)len);
    }
#else
#if (TRANSMIT_POSE_TEST_MODE == 1U)
    Transmit_PackPoseTestFrame();
#else
    Transmit_PackPoseFrame();
#endif

    (void)Transmit_StartRawIT(&huart1, pose_tx_buf, (uint16_t)sizeof(pose_tx_buf));
#endif
}

// 降落标识发送走USART1，UART5专用于小车LXS1链路
uint8_t Transmit_SendMissionDone(void)
{
#if (TRANSMIT_ENABLE_MISSION_DONE == 0U)
    return 0U;
#else
    HAL_StatusTypeDef status;

    if (huart1.gState != HAL_UART_STATE_READY)
    {
        return 0U;
    }

    // 为排查FD 01 0D完整性，降落标识使用USART1阻塞发送，确保3字节连续发出。
    status = HAL_UART_Transmit(&huart1, (uint8_t *)mission_done_buf, (uint16_t)sizeof(mission_done_buf), MISSION_DONE_TX_TIMEOUT_MS);

    if (status == HAL_OK)
    {
        return 1U;
    }

    return 0U;
#endif
}

// 检查并处理降落标识待发送状态
void Transmit_ProcessMissionDone(void)
{
#if (TRANSMIT_ENABLE_MISSION_DONE == 0U)
    return;
#else
    if (Location_GetMissionDonePending() == 0U)
    {
        return;
    }

    if (Transmit_SendMissionDone() == 1U)
    {
        Location_SetMissionDonePending(0U);
    }
#endif
}

// 周期调度入口: 先处理降落标识, 再按50ms发位姿
void Transmit_Update(void)
{
    uint32_t now_tick = HAL_GetTick();

    // 先发降落标识, 避免任务完成状态被延后
    Transmit_ProcessMissionDone();

    if ((uint32_t)(now_tick - pose_last_tick) >= TRANSMIT_POSE_PERIOD_MS)
    {
        pose_last_tick = now_tick;
        Transmit_SendPose();
    }
}

////////////////////////////////////////////////////////////////////////////////

// 构建USART2发送帧: 在此填写具体协议数据, 返回实际发送长度(字节)

void PointChoose(void)
{
    if (board >= 1 && board <= 6)
    {
        board1 = 'A';

        if (board >= 1 && board <= 3)
        {
            board2 = 7 - board;
        }
        else if (board >= 4 && board <= 6)
        {
            board2 = board - 3;
        }
    }
    else if (board >= 7 && board <= 12)
    {
        board1 = 'B';

        if (board >= 7 && board <= 9)
        {
            board2 = board - 6;
        }
        else if (board >= 10 && board <= 12)
        {
            board2 = 16 - board;
        }
    }
    else if (board >= 13 && board <= 18)
    {
        board1 = 'C';

        if (board >= 13 && board <= 15)
        {
            board2 = 19 - board;
        }
        else if (board >= 16 && board <= 18)
        {
            board2 = board - 15;
        }
    }
    else if (board >= 19 && board <= 24)
    {
        board1 = 'D';

        if (board >= 19 && board <= 21)
        {
            board2 = board - 18;
        }
        else if (board >= 22 && board <= 24)
        {
            board2 = 28 - board;
        }
    }
}

static uint16_t Transmit_SkytoLandUsart2Frame(uint8_t *buf, uint16_t max_len)
{
    if (max_len < 5U)
    {
        return 0U;
    }

    PointChoose();

    buf[0] = 0xFE;
    buf[1] = (uint8_t)((board1 >= 'A' && board1 <= 'D') ? (board1 - 'A' + 0x0AU) : 0U);
    buf[2] = (uint8_t)board2;
    buf[3] = (uint8_t)camera_number;
    buf[4] = 0xFF;

    return 5U;
}

// USART2自定义发送入口: 打包后通过中断发送, 串口忙或无有效数据时返回0
uint8_t Transmit_SendCustomUsart2(void)
{
    uint16_t tx_len = Transmit_SkytoLandUsart2Frame(usart2_custom_tx_buf,
                                                    (uint16_t)sizeof(usart2_custom_tx_buf));

    if (tx_len == 0U)
    {
        return 0U;
    }

    return Transmit_StartRawIT(&huart2, usart2_custom_tx_buf, tx_len);
}

static const char *Transmit_TaskName(uint8_t mode)
{
    if (mode == AIR_TASK_DROP) return "DROP";
    if (mode == AIR_TASK_DYNAMIC_LANDING) return "LAND";
    return "NONE";
}

static const char *Transmit_VisionModeName(int16_t mode)
{
    if (mode == VISION_MODE_ACQUIRE) return "ACQUIRE";
    if (mode == VISION_MODE_TRACK) return "TRACK";
    if (mode == VISION_MODE_LOST) return "LOST";
    return "UNKNOWN";
}

static const char *Transmit_CarStateName(uint8_t state)
{
    static const char *const names[] = {
        "READY", "STARTING", "NORMAL", "ACTION_SLOW", "LOST_HOLD",
        "REACQUIRE", "FINISH", "DONE", "ABORT", "FAULT"
    };
    return (state < (uint8_t)(sizeof(names) / sizeof(names[0]))) ?
        names[state] : "UNKNOWN";
}

static const char *Transmit_ControlModeName(const MoveVisionDebug_t *vision,
                                            uint8_t vision_valid)
{
    if (vision->following != 0U)
    {
        return (vision_valid != 0U) ? "VISION_FOLLOW" : "VISION_HOLD";
    }
    return (vision->armed != 0U) ? "VISION_SEARCH" : "WAYPOINT";
}

/* USART2/XCOM: rotate compact single-line pages to avoid long BLE packets. */
void Transmit_UpdateUsart2(void)
{
    static uint32_t last_time = 0U;
    static uint8_t page = 0U;
    static char text[USART2_DEBUG_TEXT_MAX_LEN];
    const MoveVisionDebug_t *vision;
    const MoveCmd_t *cmd;
    const CarStatusData_t *car;
    uint32_t now_tick;
    int32_t height_cm;
    uint8_t height_valid;
    uint8_t pose_valid;
    uint8_t vision_valid;
    uint8_t k230_online;
    uint8_t car_online;
    int len;

    now_tick = HAL_GetTick();
    if ((uint32_t)(now_tick - last_time) < USART2_POSE_DEBUG_PERIOD_MS)
    {
        return;
    }
    if (huart2.gState != HAL_UART_STATE_READY)
    {
        return;
    }

    last_time = now_tick;
    vision = Move_GetVisionDebug();
    cmd = Move_GetLastCmd();
    car = Telecom_GetCarStatus();
    pose_valid = Telecom_IsPoseValid();
    vision_valid = Telecom_IsVisionValid();
    k230_online = Telecom_IsK230Online();
    car_online = Telecom_IsCarStatusValid();
    height_valid = Telecom_GetFlightHeightCm(&height_cm);
    if (height_valid == 0U) height_cm = -1;

    switch (page)
    {
    case 0U:
        len = snprintf(
            text, sizeof(text),
            "[SYS t=%lu] run=%u task=%s active=%u launch=%s wp=%u ctrl=%s "
            "pose=%u xyz=%d,%d,%d yaw=%d cmd=%d,%d,z%d,%d h=%ld/%u\r\n",
            (unsigned long)now_tick,
            (unsigned int)Decision_GetRunId(),
            Transmit_TaskName(Decision_GetTaskMode()),
            (unsigned int)Decision_IsTaskActive(),
            Transmit_TakeoffPhaseName(),
            (unsigned int)Location_GetCurrentWaypointIndex(),
            Transmit_ControlModeName(vision, vision_valid),
            (unsigned int)pose_valid,
            (int)pose_data.x, (int)pose_data.y, (int)pose_data.z,
            (int)pose_data.yaw,
            (int)cmd->dx, (int)cmd->dy,
            (int)Transmit_GetCommandedHeightCm(), (int)cmd->dyaw,
            (long)height_cm, (unsigned int)height_valid);
        break;

    case 1U:
        len = snprintf(
            text, sizeof(text),
            "[VIS t=%lu] link=%u valid=%u mode=%s conf=%u px=%d,%d,r%d "
            "err=%.1f,%.1f arm=%u follow=%u fine=%u\r\n",
            (unsigned long)now_tick,
            (unsigned int)k230_online, (unsigned int)vision_valid,
            Transmit_VisionModeName(vision_target.vision_mode),
            (unsigned int)vision_target.confidence,
            (int)vision_target.dx_px, (int)vision_target.dy_px,
            (int)vision_target.radius_px,
            (double)vision->filtered_body_x_cm,
            (double)vision->filtered_body_y_cm,
            (unsigned int)vision->armed,
            (unsigned int)vision->following,
            (unsigned int)vision->small_range);
        break;

    case 2U:
        len = snprintf(
            text, sizeof(text),
            "[CAR t=%lu] link=%u run=%u state=%s speed=%u path=%lu "
            "line=%u fault=%04X wait=%u finish=%u accel=%u/%u\r\n",
            (unsigned long)now_tick,
            (unsigned int)car_online, (unsigned int)car->run_id,
            Transmit_CarStateName(car->state),
            (unsigned int)car->speed_mm_s, (unsigned long)car->path_mm,
            (unsigned int)car->line_valid, (unsigned int)car->fault,
            (unsigned int)(car->speed_mode == 4U),
            (unsigned int)car->finish_marker,
            (unsigned int)Telecom_GetCarAccelerationSendCount(),
            (unsigned int)Telecom_IsCarAccelerationAcknowledged());
        break;

    default:
        len = snprintf(
            text, sizeof(text),
            "[COM t=%lu] k230_rx=%lu ok=%lu target=%lu hb=%lu err=%lu bad=%lu "
            "car_rx=%lu task=%lu err=%lu bad=%lu\r\n",
            (unsigned long)now_tick,
            (unsigned long)Telecom_GetK230RxByteCount(),
            (unsigned long)Telecom_GetK230ParsedFrameCount(),
            (unsigned long)Telecom_GetK230TargetFrameCount(),
            (unsigned long)Telecom_GetK230HeartbeatCount(),
            (unsigned long)Telecom_GetK230UartErrorCount(),
            (unsigned long)Telecom_GetK230BadPayloadCount(),
            (unsigned long)Telecom_GetCarRxByteCount(),
            (unsigned long)Telecom_GetCarTaskStartCount(),
            (unsigned long)Telecom_GetCarUartErrorCount(),
            (unsigned long)Telecom_GetCarBadTaskCount());
        break;
    }

    if (len > 0)
    {
        if (len >= (int)sizeof(text)) len = (int)sizeof(text) - 1;
        if (Transmit_StartRawIT(&huart2, (uint8_t *)text, (uint16_t)len) != 0U)
        {
            page = (uint8_t)((page + 1U) & 0x03U);
        }
    }
}
////////////////////////////////////////////////////////////////////




