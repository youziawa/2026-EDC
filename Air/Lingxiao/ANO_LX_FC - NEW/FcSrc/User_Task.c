#include "User_Task.h"
#include "Drv_RcIn.h"
#include "LX_FC_Fun.h"
#include "LX_FC_State.h"
#include "ANO_LX.h"
#include "control.h"
#include "Drv_Uart.h"
#include "uartstdio.h"

#define USER_MISSION_STEP_LAND_HOME      10
#define USER_MISSION_STEP_PLATFORM_LAND 11
#define USER_MISSION_STEP_PLATFORM_WAIT 12
#define USER_MISSION_STEP_PLATFORM_REARM 13
#define FLIGHT_ACTION_LAND_HOME           1U
#define FLIGHT_ACTION_LAND_PLATFORM       2U
#define FLIGHT_ACTION_RETAKEOFF           3U
#define HEIGHT_MIN_CM  10
#define HEIGHT_MAX_CM  150
#define TAKEOFF_TARGET_CM             125
#define TAKEOFF_SETTLE_WINDOW_MS     5980
#define TAKEOFF_TRIGGER_MIN_CM        100
#define REARM_THROTTLE_MIN_HOLD_MS     500U
#define REARM_UNLOCK_RETRY_MS           300U
// 这组变量保存“最近一次收到的有效位姿”。
static s16 latest_pose_x = 0;
static s16 latest_pose_y = 0;
static s16 latest_pose_z = 0;
static s16 latest_pose_yaw = 0;
static u8 latest_pose_ready = 0;
static s16 input_z = 0;

s16 vz = 0; // 供调试用，看看keep_high函数输出的高度值

extern s32 HIGH; // 当前高度反馈

void UserTask_UpdateExtPose(void)
{
    s16 x = 0, y = 0, z = 0, yaw = 0;

    // 只在有新帧时刷新缓存，没新帧就保持上次值
    if (DrvUart2ReadPose(&x, &y, &z, &yaw))
    {
        latest_pose_x = x;
        latest_pose_y = y;
        // latest_pose_z = z;
        latest_pose_yaw = yaw;
        latest_pose_ready = 1;
        latest_pose_z = z;
    }
}

void UserTask_OneKeyCmd(void)
{
    //////////////////////////////////////////////////////////////////////
    // 一键起飞/降落例程
    //////////////////////////////////////////////////////////////////////
    // 用静态变量记录一键起飞/降落指令已经执行。
    static u8 one_key_mission_f = 0;
    static u8 mission_step = 0;
    static u16 time_dly_cnt_ms = 0;
    static u8 mission_done_evt = 0;
    static u16 auto_unlock_retry_ms = 0U;
    static u8 auto_unlock_confirmed = 0U;

    // 在用户任务中实时刷新位姿缓存，后续逻辑直接读取 latest_pose_*.
    UserTask_UpdateExtPose();

    /*
     * This task runs at 50 Hz. Keep F407's measured-height sample fresh so
     * Task 2 lowers one 5 cm step only after the previous step is reached.
     */
    ANO_DT_LX_Send_High_To_F407();
    PID_Init(); // 初始化PID参数

    // 判断有遥控信号才执行
    if (rc_in.fail_safe == 0)
    {
				
        if ((latest_pose_z >= HEIGHT_MIN_CM) &&(latest_pose_z <= HEIGHT_MAX_CM))
        {
            input_z = latest_pose_z;
        }

        // 判断通道6是否为高值，高值解锁
        if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 1700 && rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 2200)
        {

            /*
             * CH6可以在准备阶段提前置高，但只有F4在TASK_START后把Z从0
             * 切换到125 cm，才真正启动本次一键起飞，防止流程提前跑完。
             */
            if ((one_key_mission_f == 0U) &&
                (latest_pose_ready != 0U) &&
                (latest_pose_z >= TAKEOFF_TRIGGER_MIN_CM))
            {
                one_key_mission_f = 1U;
                mission_step = 0U;
                time_dly_cnt_ms = 0U;
                /* 清除上一次任务残留的实时速度，起飞期间不叠加旧控制量。 */
                TMD_Val_Zero();
            }

            mission_done_evt = DrvUart5ReadMissionDone();
            if (mission_done_evt == FLIGHT_ACTION_LAND_HOME)
            {
                one_key_mission_f = 1U;
                mission_step = USER_MISSION_STEP_LAND_HOME;
                auto_unlock_retry_ms = 0U;
                auto_unlock_confirmed = 0U;
                ANO_LX_SetThrottleMinOverride(0U);
            }
            else if ((mission_done_evt == FLIGHT_ACTION_LAND_PLATFORM) &&
                     (mission_step == 3U))
            {
                /* F407 has already tracked the deck down to about 10 cm. */
                one_key_mission_f = 1U;
                mission_step = USER_MISSION_STEP_PLATFORM_LAND;
                auto_unlock_retry_ms = 0U;
                auto_unlock_confirmed = 0U;
                ANO_LX_SetThrottleMinOverride(0U);
            }
            else if ((mission_done_evt == FLIGHT_ACTION_RETAKEOFF) &&
                     (mission_step == USER_MISSION_STEP_PLATFORM_WAIT))
            {
                /*
                 * F407 sends this only after touchdown plus the full 5 s
                 * wait. First force the protocol throttle to minimum, then
                 * keep requesting unlock until FC status confirms success.
                 */
                one_key_mission_f = 1U;
                mission_step = USER_MISSION_STEP_PLATFORM_REARM;
                time_dly_cnt_ms = 0U;
                auto_unlock_retry_ms = 0U;
                auto_unlock_confirmed = 0U;
                ANO_LX_SetThrottleMinOverride(1U);
                TMD_Val_Zero();
                UARTprintf("REARM THR_MIN\r\n");
            }

            // 执行任务
            if (one_key_mission_f == 1)
            {
                switch (mission_step)
                {
                case 0:
                {
                    /* Initial launch remains manual. */
                    if (fc_sta.unlock_sta == 0U)
                    {
                        break;
                    }

                    auto_unlock_retry_ms = 0U;
                    auto_unlock_confirmed = 0U;
                    ANO_LX_SetThrottleMinOverride(0U);
                    if (fc_sta.fc_mode_sta != 3U)
                    {
                        (void)LX_Change_Mode(3U);
                        break;
                    }

                    time_dly_cnt_ms = 0U;
                    mission_step = 1U;
                }
                /* 立即落入case 1，本周期就发出125 cm一键起飞指令。 */
                case 1: // 一键起飞
                {
                    /*
                     * 严格沿用参考工程的写法：仅当指令成功进入发送队列时
                     * 才推进步骤。发送成功后立即离开本步骤，禁止重复触发
                     * 一键起飞，否则飞控会反复接管油门并重启起飞流程。
                     */
                    mission_step += OneKey_Takeoff(TAKEOFF_TARGET_CM);
                }
                break;
                case 2:
                {
                    /* 一键起飞发出后总计等待约6秒（含爬升及稳定悬停）。 */
                    if (time_dly_cnt_ms < TAKEOFF_SETTLE_WINDOW_MS)
                    {
                        time_dly_cnt_ms += 20;
                    }
                    else
                    {
                        time_dly_cnt_ms = 0;
                        mission_step += 1;
                        UARTprintf("200\r\n"); // 悬停中
                      
                    }
                }
                break;
                case 3:
                {
                     u8 height_ok;

                    (void)LX_Change_Mode(3U);

                    if (latest_pose_ready == 0U)
                    {
                        TMD_Val_Zero();
                        break;
                    }

                    /*
                    * F4的Z直接作为keep_high目标高度。
                    */
                    if ((latest_pose_z >= HEIGHT_MIN_CM) &&
                        (latest_pose_z <= HEIGHT_MAX_CM))
                    {
                        input_z = latest_pose_z;
                    }

                    /*
                    * keep_high只修改垂直速度TMD_Val[2]。
                    */
                    height_ok = keep_high((u16)input_z);

                    /*
                    * 无论高度有没有到，都继续执行XY和Yaw。
                    * 这样从第二航点进入第三航点时，
                    * 飞机才能一边向X=50飞，一边向50cm下降。
                    */
                    TMD_Val[0] = latest_pose_x;
                    TMD_Val[1] = latest_pose_y;
                    TMD_Val[3] = latest_pose_yaw;

                    UARTprintf(
                        "target_z=%d, HIGH=%d, ok=%d, "
                        "vx=%d, vy=%d, vz=%d\r\n",
                        input_z,
                        (int)HIGH,
                        height_ok,
                        TMD_Val[0],
                        TMD_Val[1],
                        TMD_Val[2]);
                }
                break;
                case USER_MISSION_STEP_LAND_HOME:
                {
                    TMD_Val_Zero(); // 进入case10，降落前先把控制量清零
                    OneKey_Land();  // 一键降落
                }
                break;
                case USER_MISSION_STEP_PLATFORM_LAND:
                {
                    /* Only the final centimetres use one-key landing. */
                    TMD_Val_Zero();
                    if (OneKey_Land() != 0U)
                    {
                        mission_step = USER_MISSION_STEP_PLATFORM_WAIT;
                    }
                }
                break;
                case USER_MISSION_STEP_PLATFORM_WAIT:
                {
                    /* Motors remain landed/locked until F407 sends action 3. */
                    TMD_Val_Zero();
                    ANO_LX_SetThrottleMinOverride(0U);
                }
                break;
                case USER_MISSION_STEP_PLATFORM_REARM:
                {
                    /*
                     * Reproduce the safe manual rearm sequence through the
                     * protocol: throttle minimum -> confirmed unlock ->
                     * program mode -> the same 125 cm one-key takeoff.
                     */
                    TMD_Val_Zero();
                    ANO_LX_SetThrottleMinOverride(1U);

                    if (time_dly_cnt_ms < REARM_THROTTLE_MIN_HOLD_MS)
                    {
                        time_dly_cnt_ms += 20U;
                        break;
                    }

                    if (fc_sta.unlock_sta == 0U)
                    {
                        auto_unlock_confirmed = 0U;
                        if (auto_unlock_retry_ms < REARM_UNLOCK_RETRY_MS)
                        {
                            auto_unlock_retry_ms += 20U;
                        }
                        else if (FC_Unlock() != 0U)
                        {
                            auto_unlock_retry_ms = 0U;
                            UARTprintf("REARM UNLOCK_REQ\r\n");
                        }
                        break;
                    }

                    if (auto_unlock_confirmed == 0U)
                    {
                        auto_unlock_confirmed = 1U;
                        UARTprintf("REARM UNLOCK_OK\r\n");
                    }

                    if (fc_sta.fc_mode_sta != 3U)
                    {
                        (void)LX_Change_Mode(3U);
                        break;
                    }

                    if (OneKey_Takeoff(TAKEOFF_TARGET_CM) != 0U)
                    {
                        ANO_LX_SetThrottleMinOverride(0U);
                        auto_unlock_retry_ms = 0U;
                        auto_unlock_confirmed = 0U;
                        time_dly_cnt_ms = 0U;
                        mission_step = 2U;
                        UARTprintf("REARM TAKEOFF_125\r\n");
                    }
                }
                break;
                }
            }
        }
        else // 低值，维持上锁不变
        {
            one_key_mission_f = 0; // 复位标记，以便再次执行
            mission_step = 0;
            time_dly_cnt_ms = 0;
            latest_pose_ready = 0U;
            auto_unlock_retry_ms = 0U;
            auto_unlock_confirmed = 0U;
            ANO_LX_SetThrottleMinOverride(0U);
            UARTprintf("000\r\n"); // 未解锁
        }
    }
    ////////////////////////////////////////////////////////////////////////
}
