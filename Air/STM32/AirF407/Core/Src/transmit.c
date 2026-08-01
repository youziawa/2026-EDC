#include "transmit.h"
#include "usart.h"
#include "telecom.h"
#include "location.h"
#include "move.h"
#include "decision.h"
#include "lxs1_protocol.h"
#include <stdio.h>

// 位姿帧: FB + x/y/z/yaw(各2字节) + 0D
#define POSE_FRAME_LEN 10U

// 降落标识帧: FD 01 0D
#define MISSION_DONE_FRAME_LEN 3U

// 位姿发送周期: 50ms (20Hz)
#define TRANSMIT_POSE_PERIOD_MS 50U

/* Lingxiao直接起飞到125 cm，用约3 s起飞并在该高度保持3 s。 */
#define LINGXIAO_TAKEOFF_TARGET_CM 125
#define LINGXIAO_TAKEOFF_MS        3000U
#define TAKEOFF_FINAL_HOVER_MS     3000U
#define TAKEOFF_TOTAL_MS           \
    (LINGXIAO_TAKEOFF_MS + TAKEOFF_FINAL_HOVER_MS)


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

/* 地面站蓝牙链路：10 Hz发送飞机X/Y坐标。 */
#define USART2_GROUND_POSE_PERIOD_MS 100U

static uint8_t pose_tx_buf[POSE_FRAME_LEN];                       // 位姿帧发送包
static uint8_t usart2_custom_tx_buf[USART2_CUSTOM_FRAME_MAX_LEN]; // USART2自定义发送缓存

extern target_location_t target_location;


// ASCII模式下的位姿发送包，此时为屏蔽状态
#if (TRANSMIT_POSE_ASCII_MODE == 1U)
static uint8_t pose_ascii_tx_buf[48U];
#endif
static uint8_t mission_done_buf[MISSION_DONE_FRAME_LEN] = {0xFD, 0x01, 0x0D};

// 最近一次位姿发送时刻, 用于控制发送周期
static uint32_t pose_last_tick = 0U;

uint8_t board1 = 0; // 占位值，代表棋盘面ABCD
uint8_t board2 = 0; // 占位值，代表棋盘格123456

#if (TRANSMIT_POSE_TEST_MODE == 1U)
static void PointChoose(void);
#endif

/* 起飞阶段始终发送125 cm，6 s结束后恢复发送当前航点高度。 */
static int16_t Transmit_GetCommandedHeightCm(void)
{
    return Decision_GetCommandedHeightCm();
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
    uint8_t action;

    if (huart1.gState != HAL_UART_STATE_READY)
    {
        return 0U;
    }

    action = Location_GetMissionDonePending();
    if ((action < FLIGHT_ACTION_LAND_HOME) ||
        (action > FLIGHT_ACTION_RETAKEOFF))
    {
        return 0U;
    }

    mission_done_buf[1] = action;

    /* Send FD action 0D as one blocking write so all three bytes stay intact. */
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

#if (TRANSMIT_POSE_TEST_MODE == 1U)
static void PointChoose(void)
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
#endif

static uint16_t Transmit_SkytoLandUsart2Frame(uint8_t *buf, uint16_t max_len)
{
    lxs1_frame_t frame;
    size_t tx_len;
    int16_t x_cm = pose_data.x;
    int16_t y_cm = pose_data.y;

    frame.src = LXS1_NODE_AIR_MCU;
    frame.dst = LXS1_NODE_GROUND;
    frame.msg_id = LXS1_MSG_FC_POSE;
    frame.data_len = 4U;

    /* FC_POSE payload: int16 little-endian x_cm, y_cm. */
    frame.data[0] = (uint8_t)((uint16_t)x_cm & 0xFFU);
    frame.data[1] = (uint8_t)(((uint16_t)x_cm >> 8) & 0xFFU);
    frame.data[2] = (uint8_t)((uint16_t)y_cm & 0xFFU);
    frame.data[3] = (uint8_t)(((uint16_t)y_cm >> 8) & 0xFFU);

    tx_len = lxs1_encode(&frame, buf, max_len);
    return (uint16_t)tx_len;
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

/* USART2/蓝牙：只向地面站发送LXS1 FC_POSE坐标帧。 */
void Transmit_UpdateUsart2(void)
{
    static uint32_t last_time = 0U;
    uint32_t now_tick;

    now_tick = HAL_GetTick();
    if ((uint32_t)(now_tick - last_time) < USART2_GROUND_POSE_PERIOD_MS)
    {
        return;
    }

    if (Transmit_SendCustomUsart2() != 0U)
    {
        last_time = now_tick;
    }
}
////////////////////////////////////////////////////////////////////




