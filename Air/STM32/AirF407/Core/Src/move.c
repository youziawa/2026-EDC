#include "move.h"
#include "location.h"
#include "telecom.h"
#include "decision.h"

extern PoseData_t pose_data;
extern target_location_t target_location;

uint16_t cur_waypoint_index;

#define YAW_PERIOD 360
#define MOVE_CTRL_PERIOD_MS 20
#define ARRIVAL_CONFIRM_TICKS 25 // 需要连续25次（约500ms）满足到达条件才确认到达
#define ARRIVAL_HOVER_MS 1500U  // 到点确认后悬停时长(1s)
#define MOVE_ENABLE_HOVER 0U    // 1=到点悬停, 0=不悬停
#define ARRIVAL_ENTER_POS_TOL 10//到点位置容忍度(cm), 误差小于等于该值时认为到达
#define ARRIVAL_EXIT_POS_TOL 15
#define ARRIVAL_YAW_TOL 8
#define PID_SCALE 100 // 该结构体定义了PID控制器的参数和状态变量
/* 位置控制速度规划 */
#define MOVE_CRUISE_ERR_CM 20
#define MOVE_CRUISE_SPEED 20//起飞后先右再前的追车航线速度
#define MOVE_RETURN_SPEED 25//抛物后返回起飞点的速度
#define VISION_CHASE_SPEED 22//视觉追车速度；限制重捕获时的大幅修正
#define MOVE_APPROACH_MIN_SPEED 5
// 平滑斜率限制（每周期最大变化量，与pid.slew_limit一致）
#define SMOOTH_SLEW_LIMIT 2
/* yaw进入允许范围后，再稳定200ms才确认方向已经改变 */
#define MOVE_TURN_CONFIRM_TICKS 10U
/* TASK_START后直接起飞至125 cm并保持3 s，6 s后释放水平航线。 */
#define TASK_TAKEOFF_SETTLE_MS 6000U
#define TASK_ROUTE_FIRST_WAYPOINT_INDEX 1U

/*
 * K230 pixel-to-body-coordinate calibration.
 *
 * Body frame used by this project:
 *   +X = aircraft nose
 *   +Y = aircraft left
 *
 * Current camera mounting:
 *   image right = aircraft rear
 *   image down  = aircraft right
 */
#define VISION_CAMERA_FX_PX 365.0f
#define VISION_CAMERA_FY_PX 365.0f

#define VISION_CAMERA_OFFSET_X_CM  0.0f
/* Flight-test neutral value: the previous -1.6 cm bias held the aircraft right. */
#define VISION_CAMERA_OFFSET_Y_CM  0.0f

/*
 * 固定高度视觉换算参数。
 *
 * 飞机按固定135 cm高度飞行，K230只发送像素偏移。
 * F407使用固定比例计算水平相对位置，不使用实时高度参与换算。
 *
 * 如果135 cm指的是摄像头镜头到目标平面的距离，保持135.0f。
 */
#define VISION_FIXED_TARGET_DISTANCE_CM 125.0f

/*
 * 水平误差最大限制，避免异常像素产生过大的速度指令。
 */
#define VISION_ERROR_LIMIT_CM 80.0f
//我写的以下，现成pid和巡航速度
#define VISION_FINE_ENTER_CM 10
#define VISION_FINE_EXIT_CM  15
/*
 * 1: a valid K230 TRACK flag may take over at any mission waypoint.
 * 0: the mission must call Move_VisionFollowArm() first.
 */
#define VISION_FOLLOW_AUTO_ARM 0U

/*
 * Index 1 is the rightward waypoint (0,-40), while index 2 is the first
 * forward waypoint (85,-40).  Visual acquisition may only start after the
 * rightward waypoint has been completed and the car has announced this run.
 */
#define VISION_ARM_MIN_WAYPOINT_INDEX 2U

/* K230已完成圆环+十字验证；再确认2帧即可接管，避免低帧率下追车过晚。 */
#define VISION_TAKEOVER_CONFIRM_FRAMES 2U

/* 红点真正进入中心区域并稳定后，才允许小车从低速加速。 */
#define VISION_ACCEL_CENTER_ERR_CM 15
#define VISION_ACCEL_CONFIRM_FRAMES 5U

/* Moving-target velocity feed-forward for low-error visual tracking. */
#define VISION_VELOCITY_ALPHA 0.15f
#define VISION_VELOCITY_LIMIT_CM_S 8.0f
#define VISION_VELOCITY_MIN_DT_MS 30U
#define VISION_VELOCITY_MAX_DT_MS 200U

/*
 * 视觉接管后允许短暂漏检。窗口内继续保持最后一条安全水平指令；
 * 超时后才清零水平指令并重置视觉PID。
 */
#define VISION_LOSS_BUFFER_MS 300U

typedef struct
{
    int16_t deadband;   // 输入死区, 小于等于该值的误差被视为0
    int16_t out_limit;  // 输出限幅, PID输出被限制在[-out_limit, out_limit]
    int16_t slew_limit; // 输出斜率限制, PID输出每步变化不超过该值
    int32_t i_limit;    // 积分限幅, 积分项被限制在[-i_limit, i_limit]
    int32_t kp;         // 比例增益
    int32_t ki;         // 积分增益
    int32_t kd;         // 微分增益
    int32_t i_acc;      // 积分累加器
    int32_t d_filt;     // 微分滤波器
    int16_t prev_err;   // 上次误差, 用于计算微分项
    int16_t prev_out;   // 上次输出, 用于斜率限制
} AxisPid_t;

static MoveCmd_t move_cmd;
static uint32_t move_last_tick;
static uint8_t move_launch_run_id;
static uint32_t move_launch_start_tick;
static uint8_t move_launch_active;
static uint8_t move_arrival_handled; // 是否已处理当前航点的到达事件, 用于避免重复触发
static uint8_t move_arrival_confirm_ticks;
#if (MOVE_ENABLE_HOVER == 1U)
static uint8_t move_hover_active; // 悬停计时是否激活
static uint32_t move_hover_start_tick;
#endif
static uint16_t move_prev_waypoint_index; // 记录上次航点索引, 用于检测航点切换
static AxisPid_t pid_x;
static AxisPid_t pid_y;
static AxisPid_t pid_yaw;
static AxisPid_t pid_vision_x;
static AxisPid_t pid_vision_y;

/*
 * The mission arms visual control. The first valid K230 TRACK frame
 * latches XY takeover. After takeover, target loss causes horizontal
 * hold instead of an unsafe jump back to the old waypoint.
 */
static uint8_t vision_follow_armed;
static uint8_t vision_follow_latched;
static uint8_t vision_have_error;
static uint8_t vision_track_confirm_count;
static uint8_t vision_car_accel_notified;
static uint8_t vision_accel_center_confirm_count;
static uint32_t vision_confirm_last_rx_tick;

/* 1=视觉精细范围，0=视觉巡航追赶范围 */
static uint8_t vision_small_range;

static uint32_t vision_last_rx_tick;
static uint32_t vision_last_valid_tick;
static float vision_err_x_filt_cm;
static float vision_err_y_filt_cm;
static float vision_target_vx_cm_s;
static float vision_target_vy_cm_s;
static float vision_motion_prev_err_x_cm;
static float vision_motion_prev_err_y_cm;
static uint32_t vision_motion_last_tick;
static uint8_t vision_motion_valid;
static MoveVisionDebug_t vision_debug;

/* 最后一次已经确认完成的方向 */
static MoveHeading_t move_current_heading;

/* 当前准备转到的方向 */
static MoveHeading_t move_pending_heading;

static uint8_t move_current_heading_valid;
static uint8_t move_pending_heading_valid;
static uint8_t move_turning;
static uint8_t move_turn_confirm_ticks;

static int16_t abs16(int16_t v)
{
    if (v < 0)
    {
        return (int16_t)(-v);
    }
    return v;
}

static int16_t yaw_shortest_diff(int16_t target, int16_t current) // 寻找最小角度转动
{
    int32_t diff = (int32_t)target - (int32_t)current;

    while (diff > (YAW_PERIOD / 2))
    {
        diff -= YAW_PERIOD;
    }

    while (diff < -(YAW_PERIOD / 2))
    {
        diff += YAW_PERIOD;
    }

    return (int16_t)diff;
}

static void MovePid_Reset(AxisPid_t *pid) // 重置PID控制器的状态变量, 但不修改参数
{
    // 切换航点时调用, 避免积分和微分项对新航点产生不良影响
    pid->i_acc = 0;    // 积分累加器清零
    pid->d_filt = 0;   // 微分滤波器清零
    pid->prev_err = 0; // 上次误差清零
    pid->prev_out = 0; // 上次输出清零
}

static int16_t MovePid_Step(AxisPid_t *pid, int16_t error,
                            uint8_t use_position_speed,
                            int16_t position_speed) // 根据当前误差计算PID输出, 包括死区处理、积分累加与限幅、微分滤波、斜率限制等
{
    int32_t p_term; // 比例项
    int32_t i_term; // 积分项
    int32_t d_now;  // 当前微分项, 用于计算滤波后的微分项
    int32_t output; // PID总输出
    int32_t delta;  // 输出变化量, 用于斜率限制

    if (abs16(error) <= pid->deadband) // 死区处理，当误差绝对值小于等于死区时，视为0
    {
        error = 0;
    }

    if (error == 0)
    {
        pid->i_acc = pid->i_acc * 63 / 64; // 误差为0时积分累加器衰减
    }
    else
    {
        pid->i_acc += error;
        if (pid->i_acc > pid->i_limit)
        {
            pid->i_acc = pid->i_limit;
        }
        if (pid->i_acc < -(pid->i_limit))
        {
            pid->i_acc = -(pid->i_limit);
        }
    }

    p_term = pid->kp * error;                                    // 比例项直接与当前误差成正比
    i_term = pid->ki * pid->i_acc;                               // 积分项与积分累加器成正比
    d_now = pid->kd * ((int32_t)error - (int32_t)pid->prev_err); // 微分项与当前误差与上次误差之差成正比

    // 一阶低通抑制D项对采样噪声的放大
    pid->d_filt = ((pid->d_filt * 3) + d_now) / 4;

    // 原始输出，后需经过限幅和斜率限制处理
    output = (p_term + i_term + pid->d_filt) / PID_SCALE;
    /*
 * X/Y位置速度规划：
 * 误差大于20cm时巡航；
 * 误差在10～20cm时至少保持5cm/s；
 * 误差进入10cm后恢复原PID精细控制。
 */
    if (use_position_speed != 0U)
    {
        if (abs16(error) > MOVE_CRUISE_ERR_CM)
        {
            if (error > 0)
            {
                output = position_speed;
            }
            else
            {
                output = -position_speed;
            }
        }
        else if ((abs16(error) > ARRIVAL_ENTER_POS_TOL) &&
                (abs16((int16_t)output) < MOVE_APPROACH_MIN_SPEED))
        {
            if (error > 0)
            {
                output = MOVE_APPROACH_MIN_SPEED;
            }
            else
            {
                output = -MOVE_APPROACH_MIN_SPEED;
            }
        }
    }
    // 输出限幅
    if (output > pid->out_limit)
    {
        output = pid->out_limit;
    }
    if (output < -(pid->out_limit))
    {
        output = -(pid->out_limit);
    }

    // 斜率限制
    delta = output - pid->prev_out;
    if (delta > pid->slew_limit)
    {
        delta = pid->slew_limit;
    }
    if (delta < -(pid->slew_limit))
    {
        delta = -(pid->slew_limit);
    }

    // 经最大输出限制和斜率限制后的最终输出
    output = pid->prev_out + delta;

    // 当误差较小且输出也较小时，认为已经基本到达目标，可以将输出置零以避免微小振荡
    if ((error == 0) && (abs16((int16_t)output) <= pid->slew_limit))
    {
        output = 0;
    }

    pid->prev_err = error;           // 更新上次误差
    pid->prev_out = (int16_t)output; // 更新上次输出

    return (int16_t)output; // 返回当前PID输出
}
/*pid每架飞机不同，根据现象去调*/
//目前速度给的是30厘米每秒 PID(未调)因为机架的结构相同所以没调
static void MovePid_Init(void) // 初始化PID控制器的参数
{
    pid_x.deadband = 3;
    pid_x.out_limit = 25;
    pid_x.slew_limit = 1;
    pid_x.i_limit = 100; // 增大积分限幅，消除静差
    pid_x.kp = 14;
    pid_x.ki = 1;
    pid_x.kd = 40; // 微分适当减小，抑制震荡
    MovePid_Reset(&pid_x);

    // Y轴相同
    // pid_y = pid_x;
    pid_y.deadband = 3;
    pid_y.out_limit = 25;
    pid_y.slew_limit = 1;
    pid_y.i_limit = 100;
    pid_y.kp = 18;
    pid_y.ki = 1;
    pid_y.kd = 30;
    MovePid_Reset(&pid_y);
    pid_yaw.deadband = 5;
    pid_yaw.out_limit = 70;
    pid_yaw.slew_limit = 3;
    pid_yaw.i_limit = 400;
    pid_yaw.kp = 15;
    pid_yaw.ki = 1;
    pid_yaw.kd = 30;
    MovePid_Reset(&pid_yaw);

/*
 * 视觉控制直接复制已经验证稳定的普通位置PID参数。
 * 使用独立PID实例，避免与航点PID共用积分和历史误差。
 */
pid_vision_x = pid_x;

/* 视觉前后方向最大限制为15cm/s */
pid_vision_x.out_limit = VISION_CHASE_SPEED;

MovePid_Reset(&pid_vision_x);


pid_vision_y = pid_y;

/* 视觉左右方向最大限制为15cm/s */
pid_vision_y.out_limit = VISION_CHASE_SPEED;

MovePid_Reset(&pid_vision_y);

    //只有X轴和Y轴的pid，轴在凌霄上面
}
static uint8_t Move_YawToHeading(int16_t yaw_deg,
                                 MoveHeading_t *heading)
{
    int32_t yaw = yaw_deg;

    while (yaw < 0)
    {
        yaw += 360;
    }

    while (yaw >= 360)
    {
        yaw -= 360;
    }

    switch (yaw)
    {
    case 0:
        *heading = MOVE_HEADING_0;
        return 1U;

    case 90:
        *heading = MOVE_HEADING_90;
        return 1U;

    case 180:
        *heading = MOVE_HEADING_180;
        return 1U;

    case 270:
        *heading = MOVE_HEADING_270;
        return 1U;

    default:
        return 0U;
    }
}

static void Move_MapErrorToBody(int16_t map_x,
                                int16_t map_y,
                                MoveHeading_t heading,
                                int16_t *body_x,
                                int16_t *body_y)
{
    switch (heading)
    {
    case MOVE_HEADING_0:
        *body_x = map_x;
        *body_y = map_y;
        break;

    case MOVE_HEADING_90:
        *body_x = map_y;
        *body_y = (int16_t)(-map_x);
        break;

    case MOVE_HEADING_180:
        *body_x = (int16_t)(-map_x);
        *body_y = (int16_t)(-map_y);
        break;

    case MOVE_HEADING_270:
        *body_x = (int16_t)(-map_y);
        *body_y = map_x;
        break;

    default:
        *body_x = 0;
        *body_y = 0;
        break;
    }
}
/* 立即停止水平运动 */
static void Move_StopHorizontal(void)
{
    move_cmd.dx = 0;
    move_cmd.dy = 0;
}

/* Round and clamp a vision position error before the integer PID. */
static int16_t Move_VisionErrorToInt16(float error_cm)
{
    if (error_cm > VISION_ERROR_LIMIT_CM)
    {
        error_cm = VISION_ERROR_LIMIT_CM;
    }
    else if (error_cm < -VISION_ERROR_LIMIT_CM)
    {
        error_cm = -VISION_ERROR_LIMIT_CM;
    }

    if (error_cm >= 0.0f)
    {
        return (int16_t)(error_cm + 0.5f);
    }

    return (int16_t)(error_cm - 0.5f);
}

static float Move_ClampVisionVelocity(float velocity_cm_s)
{
    if (velocity_cm_s > VISION_VELOCITY_LIMIT_CM_S)
    {
        return VISION_VELOCITY_LIMIT_CM_S;
    }
    if (velocity_cm_s < -VISION_VELOCITY_LIMIT_CM_S)
    {
        return -VISION_VELOCITY_LIMIT_CM_S;
    }
    return velocity_cm_s;
}

static int16_t Move_ClampVisionCommand(int32_t command_cm_s)
{
    if (command_cm_s > VISION_CHASE_SPEED)
    {
        return VISION_CHASE_SPEED;
    }
    if (command_cm_s < -VISION_CHASE_SPEED)
    {
        return -VISION_CHASE_SPEED;
    }
    return (int16_t)command_cm_s;
}

static void Move_ResetVisionMotionEstimate(void)
{
    vision_target_vx_cm_s = 0.0f;
    vision_target_vy_cm_s = 0.0f;
    vision_motion_prev_err_x_cm = 0.0f;
    vision_motion_prev_err_y_cm = 0.0f;
    vision_motion_last_tick = 0U;
    vision_motion_valid = 0U;
}

/*
 * Safe state after target loss, a stale frame, or invalid height.
 * Keep visual ownership latched and command zero XY. Z is untouched:
 * transmit.c continues sending the current waypoint Z to keep_high().
 */
static void Move_VisionHoldHorizontal(void)
{
    Move_StopHorizontal();
    move_cmd.dyaw = 0;

    MovePid_Reset(&pid_vision_x);
    MovePid_Reset(&pid_vision_y);

    vision_have_error = 0U;
    vision_small_range = 0U;
    vision_last_valid_tick = 0U;

    vision_err_x_filt_cm = 0.0f;
    vision_err_y_filt_cm = 0.0f;
    Move_ResetVisionMotionEstimate();

    vision_debug.small_range = 0U;

    vision_debug.following = vision_follow_latched;
    vision_debug.conversion_valid = 0U;
    vision_debug.laser_height_cm = 0;
    vision_debug.target_distance_cm = 0.0f;
    vision_debug.raw_body_x_cm = 0.0f;
    vision_debug.raw_body_y_cm = 0.0f;
    vision_debug.filtered_body_x_cm = 0.0f;
    vision_debug.filtered_body_y_cm = 0.0f;
    vision_debug.camera_fx_px = VISION_CAMERA_FX_PX;
    vision_debug.camera_fy_px = VISION_CAMERA_FY_PX;
    vision_debug.camera_offset_x_cm =
        VISION_CAMERA_OFFSET_X_CM;
    vision_debug.camera_offset_y_cm =
        VISION_CAMERA_OFFSET_Y_CM;
    vision_debug.source_rx_tick = 0U;
}

/*
 * Call once when the mission enters the car-following stage.
 * Waypoint control continues until K230 publishes a valid TRACK frame.
 */
void Move_VisionFollowArm(void)
{
    vision_follow_armed = 1U;
    vision_follow_latched = 0U;
    vision_have_error = 0U;
    vision_small_range = 0U;
    vision_last_rx_tick = 0U;
    vision_last_valid_tick = 0U;
    vision_track_confirm_count = 0U;
    vision_car_accel_notified = 0U;
    vision_accel_center_confirm_count = 0U;

    /*
     * 丢弃解锁前已经收到的识别结果，只统计解锁后到达的新帧。
     */
    vision_confirm_last_rx_tick = vision_target.rx_tick;

    vision_err_x_filt_cm = 0.0f;
    vision_err_y_filt_cm = 0.0f;
    Move_ResetVisionMotionEstimate();

    MovePid_Reset(&pid_vision_x);
    MovePid_Reset(&pid_vision_y);

    vision_debug.armed = 1U;
    vision_debug.following = 0U;
    vision_debug.conversion_valid = 0U;
    vision_debug.small_range = 0U;
    vision_debug.small_range = 0U;
}

/* End visual following and allow waypoint control on the next cycle. */
void Move_VisionFollowStop(void)
{
    vision_follow_armed = 0U;
    vision_follow_latched = 0U;
    vision_have_error = 0U;
    vision_small_range = 0U;
    vision_last_rx_tick = 0U;
    vision_last_valid_tick = 0U;
    vision_track_confirm_count = 0U;
    vision_car_accel_notified = 0U;
    vision_accel_center_confirm_count = 0U;
    vision_confirm_last_rx_tick = 0U;
    vision_err_x_filt_cm = 0.0f;
    vision_err_y_filt_cm = 0.0f;
    Move_ResetVisionMotionEstimate();

    Move_StopHorizontal();
    move_cmd.dyaw = 0;

    MovePid_Reset(&pid_vision_x);
    MovePid_Reset(&pid_vision_y);
    MovePid_Reset(&pid_x);
    MovePid_Reset(&pid_y);
    MovePid_Reset(&pid_yaw);

    move_arrival_confirm_ticks = 0U;
    move_arrival_handled = 0U;
    move_prev_waypoint_index = 0xFFFFU;
#if (MOVE_ENABLE_HOVER == 1U)
    move_hover_active = 0U;
    move_hover_start_tick = 0U;
#endif

    vision_debug.armed = 0U;
    vision_debug.following = 0U;
    vision_debug.conversion_valid = 0U;
    vision_debug.laser_height_cm = 0;
    vision_debug.target_distance_cm = 0.0f;
    vision_debug.raw_body_x_cm = 0.0f;
    vision_debug.raw_body_y_cm = 0.0f;
    vision_debug.filtered_body_x_cm = 0.0f;
    vision_debug.filtered_body_y_cm = 0.0f;
    vision_debug.camera_fx_px = VISION_CAMERA_FX_PX;
    vision_debug.camera_fy_px = VISION_CAMERA_FY_PX;
    vision_debug.camera_offset_x_cm =
        VISION_CAMERA_OFFSET_X_CM;
    vision_debug.camera_offset_y_cm =
        VISION_CAMERA_OFFSET_Y_CM;
    vision_debug.source_rx_tick = 0U;
}

uint8_t Move_IsVisionFollowing(void)
{
    return vision_follow_latched;
}

/*
 * Return 0 when normal waypoint control should continue.
 * Return 1 when visual control owns XY and Move_Update() must return.
 */
static uint8_t Move_UpdateVisionFollow(void)
{
    float target_distance_cm;
    float raw_err_x_cm;
    float raw_err_y_cm;
    int16_t err_x_cm;
    int16_t err_y_cm;
    int16_t pid_cmd_x_cm_s;
    int16_t pid_cmd_y_cm_s;
    int16_t feedforward_x_cm_s;
    int16_t feedforward_y_cm_s;
    uint8_t vision_valid;
    uint8_t use_cruise_speed;
    uint32_t now_tick;
    uint32_t motion_dt_ms;
    float measured_vx_cm_s;
    float measured_vy_cm_s;

    vision_debug.armed = vision_follow_armed;
    vision_debug.following = vision_follow_latched;
    now_tick = HAL_GetTick();

    if (vision_follow_armed == 0U)
    {
        vision_debug.conversion_valid = 0U;
        return 0U;
    }

    if (vision_car_accel_notified != 0U)
    {
        Telecom_RequestCarAccelerate();
    }

    vision_valid = Telecom_IsVisionValid();

    /*
     * Before takeover, keep flying the waypoint route. Only new valid
     * TRACK frames received after arming count toward takeover.
     */
    if (vision_follow_latched == 0U)
    {
        /* K230 stays idle after boot and only starts detecting near B. */
        Telecom_RequestVisionStart();

        if (vision_valid == 0U)
        {
            vision_track_confirm_count = 0U;
            return 0U;
        }

        if (vision_confirm_last_rx_tick != vision_target.rx_tick)
        {
            vision_confirm_last_rx_tick = vision_target.rx_tick;

            if (vision_track_confirm_count <
                VISION_TAKEOVER_CONFIRM_FRAMES)
            {
                vision_track_confirm_count++;
            }
        }

        if (vision_track_confirm_count <
            VISION_TAKEOVER_CONFIRM_FRAMES)
        {
            return 0U;
        }

        vision_follow_latched = 1U;
        vision_have_error = 0U;
        vision_small_range = 0U;
        vision_last_rx_tick = 0U;
        vision_last_valid_tick = 0U;
        vision_debug.following = 1U;
        vision_debug.small_range = 0U;

        /* Detection at the image edge is only acquisition, not a catch. */
        vision_accel_center_confirm_count = 0U;

        Move_StopHorizontal();
        move_cmd.dyaw = 0;

        /*
         * Remove old waypoint integrator state so a later return to
         * coordinate control cannot produce a sudden command.
         */
        MovePid_Reset(&pid_x);
        MovePid_Reset(&pid_y);
        MovePid_Reset(&pid_yaw);
        MovePid_Reset(&pid_vision_x);
        MovePid_Reset(&pid_vision_y);
        Move_ResetVisionMotionEstimate();

        move_arrival_confirm_ticks = 0U;
#if (MOVE_ENABLE_HOVER == 1U)
        move_hover_active = 0U;
        move_hover_start_tick = 0U;
#endif
    }

    /*
     * Once K230 has taken over, never resume an old waypoint merely
     * because frames are lost. Bridge a short dropout with the last
     * safe command; stop and reset only after the bounded grace time.
     */
    if (vision_valid == 0U)
    {
        if ((vision_have_error != 0U) &&
            ((uint32_t)(now_tick - vision_last_valid_tick) <=
             VISION_LOSS_BUFFER_MS))
        {
            /*
             * Keep move_cmd.dx/dy and the visual PID state unchanged.
             * The decision layer still sees Telecom_IsVisionValid()==0,
             * so a buffered/stale target can never trigger payload drop.
             */
            vision_debug.conversion_valid = 0U;
            return 1U;
        }

        Move_VisionHoldHorizontal();
        return 1U;
    }

   /*
 * 飞机按固定135 cm高度运行。
 * K230只提供像素偏移，水平位置换算不依赖实时高度。
 */
target_distance_cm =
    VISION_FIXED_TARGET_DISTANCE_CM;

    /*
     * K230 updates more slowly than Move_Update(). Process each visual
     * frame once so the I term is not accumulated repeatedly. Hold the
     * last safe command between new frames.
     */
    if ((vision_have_error != 0U) &&
        (vision_last_rx_tick == vision_target.rx_tick))
    {
        return 1U;
    }

    vision_last_rx_tick = vision_target.rx_tick;
    vision_last_valid_tick = vision_target.rx_tick;

    /*
     * Flight-test calibration (2026-08-01):
     *   image X is opposite to aircraft fore/aft command;
     *   image Y has the same sign as aircraft left/right command.
     */
    raw_err_x_cm =
        -((float)vision_target.dx_px *
          target_distance_cm / VISION_CAMERA_FX_PX) +
        VISION_CAMERA_OFFSET_X_CM;

    raw_err_y_cm =
        ((float)vision_target.dy_px *
         target_distance_cm / VISION_CAMERA_FY_PX) +
        VISION_CAMERA_OFFSET_Y_CM;

    /* Low-pass filter only when a new K230 result arrives. */
    if (vision_have_error == 0U)
    {
        vision_err_x_filt_cm = raw_err_x_cm;
        vision_err_y_filt_cm = raw_err_y_cm;
        vision_have_error = 1U;
    }
    else
    {
        vision_err_x_filt_cm =
            0.3f * vision_err_x_filt_cm +
            0.7f * raw_err_x_cm;

        vision_err_y_filt_cm =
            0.3f * vision_err_y_filt_cm +
            0.7f * raw_err_y_cm;
    }

    /*
     * Estimate the car velocity from relative-error motion plus the previous
     * aircraft command. Adding this feed-forward prevents the position PID
     * from settling permanently behind a moving car and follows direction
     * changes through bends without assuming a fixed map heading.
     */
    if (vision_motion_valid != 0U)
    {
        motion_dt_ms =
            (uint32_t)(vision_target.rx_tick - vision_motion_last_tick);
        if ((motion_dt_ms >= VISION_VELOCITY_MIN_DT_MS) &&
            (motion_dt_ms <= VISION_VELOCITY_MAX_DT_MS))
        {
            measured_vx_cm_s =
                ((raw_err_x_cm - vision_motion_prev_err_x_cm) * 1000.0f /
                 (float)motion_dt_ms) + (float)move_cmd.dx;
            measured_vy_cm_s =
                ((raw_err_y_cm - vision_motion_prev_err_y_cm) * 1000.0f /
                 (float)motion_dt_ms) + (float)move_cmd.dy;

            measured_vx_cm_s =
                Move_ClampVisionVelocity(measured_vx_cm_s);
            measured_vy_cm_s =
                Move_ClampVisionVelocity(measured_vy_cm_s);

            vision_target_vx_cm_s =
                (1.0f - VISION_VELOCITY_ALPHA) * vision_target_vx_cm_s +
                VISION_VELOCITY_ALPHA * measured_vx_cm_s;
            vision_target_vy_cm_s =
                (1.0f - VISION_VELOCITY_ALPHA) * vision_target_vy_cm_s +
                VISION_VELOCITY_ALPHA * measured_vy_cm_s;
        }
        else
        {
            vision_target_vx_cm_s = 0.0f;
            vision_target_vy_cm_s = 0.0f;
        }
    }
    else
    {
        vision_motion_valid = 1U;
    }

    vision_motion_prev_err_x_cm = raw_err_x_cm;
    vision_motion_prev_err_y_cm = raw_err_y_cm;
    vision_motion_last_tick = vision_target.rx_tick;
    
    err_x_cm =
        Move_VisionErrorToInt16(vision_err_x_filt_cm);
    err_y_cm =
        Move_VisionErrorToInt16(vision_err_y_filt_cm);

    /*
     * Keep the car at its initial creep speed while the aircraft closes a
     * large visual error.  Five consecutive centred frames constitute a
     * genuine catch and make the acceleration request resistant to a single
     * noisy/reacquired blob.
     */
    if (vision_car_accel_notified == 0U)
    {
        if ((abs16(err_x_cm) <= VISION_ACCEL_CENTER_ERR_CM) &&
            (abs16(err_y_cm) <= VISION_ACCEL_CENTER_ERR_CM))
        {
            if (vision_accel_center_confirm_count <
                VISION_ACCEL_CONFIRM_FRAMES)
            {
                vision_accel_center_confirm_count++;
            }
            if (vision_accel_center_confirm_count >=
                VISION_ACCEL_CONFIRM_FRAMES)
            {
                vision_car_accel_notified = 1U;
                Telecom_RequestCarAccelerate();
            }
        }
        else
        {
            vision_accel_center_confirm_count = 0U;
        }
    }
            /*
        * 视觉控制范围切换：
        *
        * X、Y同时进入±10cm：
        *     进入精细PID。
        *
        * 任意一轴超过±15cm：
        *     退出精细PID，重新使用巡航追赶。
        *
        * 10cm和15cm之间形成滞回，防止反复切换。
        */
        if (vision_small_range == 0U)
        {
            if ((abs16(err_x_cm) <= VISION_FINE_ENTER_CM) &&
                (abs16(err_y_cm) <= VISION_FINE_ENTER_CM))
            {
                vision_small_range = 1U;
            }
        }
        else
        {
            if ((abs16(err_x_cm) > VISION_FINE_EXIT_CM) ||
                (abs16(err_y_cm) > VISION_FINE_EXIT_CM))
            {
                vision_small_range = 0U;
            }
        }

        /*
        * 远距离启用原有巡航速度规划；
        * 小范围关闭巡航速度强制，只使用成品PID。
        */
        if (vision_small_range != 0U)
        {
            use_cruise_speed = 0U;
        }
        else
        {
            use_cruise_speed = 1U;
        }
    /*
     * Publish the exact values used by the controller. This update
     * happens only after a complete valid pixel/height conversion.
     */
    vision_debug.armed = vision_follow_armed;
    vision_debug.following = vision_follow_latched;
    vision_debug.conversion_valid = 1U;
    vision_debug.laser_height_cm =
        (int32_t)VISION_FIXED_TARGET_DISTANCE_CM;
    vision_debug.target_distance_cm = target_distance_cm;
    vision_debug.raw_body_x_cm = raw_err_x_cm;
    vision_debug.raw_body_y_cm = raw_err_y_cm;
    vision_debug.filtered_body_x_cm = vision_err_x_filt_cm;
    vision_debug.filtered_body_y_cm = vision_err_y_filt_cm;
    vision_debug.source_rx_tick = vision_target.rx_tick;

    /*
     * Far from target, retain the normal cruise-speed branch. Inside
     * the fine range, use the visual PID output directly.
     */
    pid_cmd_x_cm_s =
        MovePid_Step(&pid_vision_x,
                     err_x_cm,
                     use_cruise_speed,
                     VISION_CHASE_SPEED);
    pid_cmd_y_cm_s =
        MovePid_Step(&pid_vision_y,
                     err_y_cm,
                     use_cruise_speed,
                     VISION_CHASE_SPEED);

    /* Do not amplify noisy edge-of-frame reacquisition before a real catch. */
    if (vision_car_accel_notified != 0U)
    {
        feedforward_x_cm_s =
            Move_VisionErrorToInt16(vision_target_vx_cm_s);
        feedforward_y_cm_s =
            Move_VisionErrorToInt16(vision_target_vy_cm_s);
    }
    else
    {
        feedforward_x_cm_s = 0;
        feedforward_y_cm_s = 0;
    }

    move_cmd.dx = Move_ClampVisionCommand(
        (int32_t)pid_cmd_x_cm_s + feedforward_x_cm_s);
    move_cmd.dy = Move_ClampVisionCommand(
        (int32_t)pid_cmd_y_cm_s + feedforward_y_cm_s);

    /* Visual following does not own altitude or yaw. */
    move_cmd.dyaw = 0;

    return 1U;
}

/* 收到一个新的转向目标 */
static void Move_BeginHeadingChange(MoveHeading_t new_heading)
{
    move_pending_heading = new_heading;
    move_pending_heading_valid = 1U;

    move_turning = 1U;
    move_turn_confirm_ticks = 0U;

    Move_StopHorizontal();

    /* 旧方向的位置PID不能带到新方向 */
    MovePid_Reset(&pid_x);
    MovePid_Reset(&pid_y);

    /* 新的yaw目标也重新开始控制 */
    MovePid_Reset(&pid_yaw);
}

/*
 * 每个20ms控制周期调用一次。
 *
 * 返回1：方向已经确认，允许输出dx/dy
 * 返回0：仍在转向或等待稳定，禁止输出dx/dy
 */
static uint8_t Move_UpdateHeadingState(int16_t target_yaw,
                                       int16_t yaw_error)
{
    MoveHeading_t decoded_heading;
    uint8_t decode_ok;

    decode_ok =
        Move_YawToHeading(target_yaw,
                          &decoded_heading);

    /*
     * 目标不是0/90/180/270时，禁止平移。
     * 防止飞机转到45°以后仍使用旧坐标系。
     */
    if (decode_ok == 0U)
    {
        if ((move_turning == 0U) ||
            (move_pending_heading_valid != 0U))
        {
            MovePid_Reset(&pid_x);
            MovePid_Reset(&pid_y);
        }

        move_pending_heading_valid = 0U;
        move_turning = 1U;
        move_turn_confirm_ticks = 0U;

        Move_StopHorizontal();
        return 0U;
    }

    /*
     * 检测新的目标方向。
     *
     * 例如：
     * pending原来是90，新航点要求180，
     * 立即重新开始一次转向。
     */
    if ((move_pending_heading_valid == 0U) ||
        (decoded_heading != move_pending_heading))
    {
        Move_BeginHeadingChange(decoded_heading);
    }

    /*
     * yaw还没有进入允许范围。
     * 转向过程中必须停止水平移动。
     */
    if (abs16(yaw_error) > ARRIVAL_YAW_TOL)
    {
        /*
         * 也处理“方向没变，但是yaw被吹偏”的情况。
         */
        if (move_turning == 0U)
        {
            MovePid_Reset(&pid_x);
            MovePid_Reset(&pid_y);
        }

        move_turning = 1U;
        move_turn_confirm_ticks = 0U;

        Move_StopHorizontal();
        return 0U;
    }

    /*
     * yaw已经进入±8°，但还需要连续稳定200ms。
     */
    if ((move_current_heading_valid == 0U) ||
        (move_current_heading != move_pending_heading) ||
        (move_turning != 0U))
    {
        move_turning = 1U;
        Move_StopHorizontal();

        if (move_turn_confirm_ticks <
            MOVE_TURN_CONFIRM_TICKS)
        {
            move_turn_confirm_ticks++;
        }

        if (move_turn_confirm_ticks >=
            MOVE_TURN_CONFIRM_TICKS)
        {
            /*
             * 只有真正稳定完成，才提交新的方向标志。
             */
            move_current_heading =
                move_pending_heading;

            move_current_heading_valid = 1U;
            move_turning = 0U;
            move_turn_confirm_ticks = 0U;

            /*
             * 方向切换完成，再清一次XY PID。
             * 下一控制周期才恢复平移。
             */
            MovePid_Reset(&pid_x);
            MovePid_Reset(&pid_y);
        }

        return 0U;
    }

    /* 当前方向与目标方向一致，允许平移 */
    return 1U;
}
void Move_Init(void) // 该结构体是移动指令值，并记录上次更新的时间戳
{
    // 三项移动指令初始化
    move_cmd.dx = 0;
    move_cmd.dy = 0;
    move_cmd.dz = 0;
    move_cmd.dyaw = 0;

    // 获取当前时间
    move_last_tick = HAL_GetTick();
    move_launch_run_id = 0U;
    move_launch_start_tick = 0U;
    move_launch_active = 0U;

    // 到达事件处理标志
    move_arrival_handled = 0;

    // 到点确认需要连续满足条件的次数
    move_arrival_confirm_ticks = 0;
#if (MOVE_ENABLE_HOVER == 1U)
    // 悬停计时初始状态激活标志
    move_hover_active = 0;

    // 悬停计时起始时间戳
    move_hover_start_tick = 0;
#endif

    // 初始化无效索引，确保第一次更新时能正确检测到航点切换
    move_prev_waypoint_index = 0xFFFF;
    move_current_heading = MOVE_HEADING_0;
    move_pending_heading = MOVE_HEADING_0;

    move_current_heading_valid = 0U;
    move_pending_heading_valid = 0U;

    move_turning = 1U;
    move_turn_confirm_ticks = 0U;
    // 初始化PID控制器参数和状态
    MovePid_Init();

    /*
     * With AUTO_ARM=1, the K230 valid+TRACK flag is the takeover
     * trigger requested by the mission. Set it to 0 later if takeover
     * should be restricted to one explicit mission state.
     */
    vision_follow_armed = VISION_FOLLOW_AUTO_ARM;
    vision_follow_latched = 0U;
    vision_have_error = 0U;
    vision_small_range = 0U;
    vision_last_rx_tick = 0U;
    vision_last_valid_tick = 0U;
    vision_track_confirm_count = 0U;
    vision_car_accel_notified = 0U;
    vision_accel_center_confirm_count = 0U;
    vision_confirm_last_rx_tick = 0U;
    vision_err_x_filt_cm = 0.0f;
    vision_err_y_filt_cm = 0.0f;
    Move_ResetVisionMotionEstimate();

    vision_debug.armed = vision_follow_armed;
    vision_debug.following = 0U;
    vision_debug.conversion_valid = 0U;
    vision_debug.small_range = 0U;
    vision_debug.laser_height_cm = 0;
    vision_debug.target_distance_cm = 0.0f;
    vision_debug.raw_body_x_cm = 0.0f;
    vision_debug.raw_body_y_cm = 0.0f;
    vision_debug.filtered_body_x_cm = 0.0f;
    vision_debug.filtered_body_y_cm = 0.0f;
    vision_debug.camera_fx_px = VISION_CAMERA_FX_PX;
    vision_debug.camera_fy_px = VISION_CAMERA_FY_PX;
    vision_debug.camera_offset_x_cm =
        VISION_CAMERA_OFFSET_X_CM;
    vision_debug.camera_offset_y_cm =
        VISION_CAMERA_OFFSET_Y_CM;
    vision_debug.source_rx_tick = 0U;
}

void Move_Update(void)
{
    uint32_t now_tick = HAL_GetTick();                                    // 获取当前系统时间戳
    uint16_t waypoint_count = Location_GetWaypointCount();                // 获取航点总数
    uint16_t current_waypoint_index = Location_GetCurrentWaypointIndex(); // 获取当前航点索引
    int16_t err_x, err_y, err_yaw;                                        // 误差值
    int16_t cur_x = pose_data.x;
    int16_t cur_y = pose_data.y;
    uint8_t has_next_waypoint;     // 是否有下一个航点
    uint8_t reached_this_waypoint; // 是否已到达当前航点，是否严格进入5cm
#if (MOVE_ENABLE_HOVER == 1U)
    uint8_t stayed_near_target;    // 是否仍在8cm保持范围
#endif
    int16_t body_err_x;
    int16_t body_err_y;
    uint8_t heading_ready;
    cur_waypoint_index = current_waypoint_index; // 更新全局当前航点索引，供外部访问
    // 检测到航点进行切换，重置相关状态
    if (current_waypoint_index != move_prev_waypoint_index)
    {
        move_prev_waypoint_index = current_waypoint_index;
        Move_StopHorizontal();
        move_arrival_handled = 0;
        move_arrival_confirm_ticks = 0;
    #if (MOVE_ENABLE_HOVER == 1U)
        move_hover_active = 0;
        move_hover_start_tick = 0;
#endif
        MovePid_Reset(&pid_x);
        MovePid_Reset(&pid_y);
        MovePid_Reset(&pid_yaw);
    }

    // 控制周期判断，确保Move_Update（发送速度指令）以固定频率运行
    if ((now_tick - move_last_tick) < MOVE_CTRL_PERIOD_MS)
    {
        return; // 控制周期未到，保持上次移动指令不变
    }
    move_last_tick = now_tick;

    /*
     * SLAM becoming valid during takeoff must not start the waypoint route.
     * Wait for the car's TASK_START so horizontal control cannot fight the
     * takeoff stabilisation loop.
     */
    if (Decision_IsTaskActive() == 0U)
    {
        move_launch_active = 0U;
        move_launch_run_id = 0U;
        move_launch_start_tick = 0U;
        Move_StopHorizontal();
        move_cmd.dyaw = 0;
        MovePid_Reset(&pid_x);
        MovePid_Reset(&pid_y);
        MovePid_Reset(&pid_yaw);
        return;
    }

    /* 新任务与凌霄起飞共用时钟；约3 s起飞并保持3 s后释放水平航线。 */
    if ((move_launch_active == 0U) ||
        (move_launch_run_id != Decision_GetRunId()))
    {
        move_launch_active = 1U;
        move_launch_run_id = Decision_GetRunId();
        move_launch_start_tick = now_tick;
        Move_VisionFollowStop();

        /*
         * 航点0只是起飞原点。任务开始后预置到航点1，6秒起飞门控
         * 解除时立即执行“先向右”，避免再次等待原点到达判定。
         */
        if ((Decision_IsReturning() == 0U) &&
            (waypoint_count > TASK_ROUTE_FIRST_WAYPOINT_INDEX))
        {
            (void)Location_SetCurrentWaypointIndex(
                TASK_ROUTE_FIRST_WAYPOINT_INDEX);
        }
        Move_StopHorizontal();
        move_cmd.dyaw = 0;
        MovePid_Reset(&pid_x);
        MovePid_Reset(&pid_y);
        MovePid_Reset(&pid_yaw);
        return;
    }

    if ((uint32_t)(now_tick - move_launch_start_tick) <
        TASK_TAKEOFF_SETTLE_MS)
    {
        Move_StopHorizontal();
        move_cmd.dyaw = 0;
        MovePid_Reset(&pid_x);
        MovePid_Reset(&pid_y);
        MovePid_Reset(&pid_yaw);
        return;
    }

    /*
     * Two independent gates prevent the takeoff marker from taking over XY:
     *   1. the initial rightward leg must already be complete;
     *   2. this aircraft must have received the car's current TASK_START.
     *
     * This check runs every control cycle.  Therefore it still arms correctly
     * when the aircraft reaches index 2 before the later-starting car moves.
     */
    if ((vision_follow_armed == 0U) &&
        (Decision_IsReturning() == 0U) &&
        (current_waypoint_index >= VISION_ARM_MIN_WAYPOINT_INDEX) &&
        (Decision_IsTaskActive() != 0U))
    {
        Move_VisionFollowArm();
    }

    /* 无新鲜SLAM位姿时禁止继续输出水平运动。 */
    if (Telecom_IsPoseValid() == 0U)
    {
        Move_StopHorizontal();
        move_cmd.dyaw = 0;
        return;
    }

    /*
     * A valid K230 TRACK frame takes over XY before any waypoint error,
     * turn, arrival, hover, or waypoint-advance logic is evaluated.
     */
    if (Move_UpdateVisionFollow() != 0U)
    {
        return;
    }

    // 当目标yaw为180，且当前yaw在容忍范围内时，将当前x/y取负


    // 各方向误差计算
    err_x = (int16_t)(target_location.x - cur_x);
    err_y = (int16_t)(target_location.y - cur_y);
    err_yaw = yaw_shortest_diff(target_location.yaw, pose_data.yaw);

heading_ready =
    Move_UpdateHeadingState(target_location.yaw,
                            err_yaw);

if (heading_ready != 0U)
{
    Move_MapErrorToBody(err_x,
                        err_y,
                        move_current_heading,
                        &body_err_x,
                        &body_err_y);

    move_cmd.dx =
        MovePid_Step(&pid_x,
                     body_err_x,
                     1U,
                     (Decision_IsReturning() != 0U) ?
                         MOVE_RETURN_SPEED : MOVE_CRUISE_SPEED);

    move_cmd.dy =
        MovePid_Step(&pid_y,
                     body_err_y,
                     1U,
                     (Decision_IsReturning() != 0U) ?
                         MOVE_RETURN_SPEED : MOVE_CRUISE_SPEED);
}
else
{
    Move_StopHorizontal();
}

move_cmd.dyaw =
    MovePid_Step(&pid_yaw,
                 err_yaw,
                 0U,
                 MOVE_CRUISE_SPEED);

    // 存在下一航点判定标志位的逻辑
    has_next_waypoint = (uint8_t)(
        (Decision_IsReturning() == 0U) &&
        (waypoint_count > 0U) &&
        ((uint16_t)(current_waypoint_index + 1U) < waypoint_count));

    // 到达当前航点判定：当x、y、yaw误差都在容忍范围内时认为到达
// 严格到达判断：X、Y必须进入±5cm，yaw误差必须在允许范围内
reached_this_waypoint =
    (uint8_t)((heading_ready != 0U) &&
              (abs16(err_x) <= ARRIVAL_ENTER_POS_TOL) &&
              (abs16(err_y) <= ARRIVAL_ENTER_POS_TOL) &&
              (abs16(err_yaw) <= ARRIVAL_YAW_TOL));

#if (MOVE_ENABLE_HOVER == 1U)

stayed_near_target =
    (uint8_t)((heading_ready != 0U) &&
              (abs16(err_x) <= ARRIVAL_EXIT_POS_TOL) &&
              (abs16(err_y) <= ARRIVAL_EXIT_POS_TOL) &&
              (abs16(err_yaw) <= ARRIVAL_YAW_TOL));

// 当前航点还没有处理完成
if (move_arrival_handled == 0U)
{
    // 还没有开始2秒悬停
    if (move_hover_active == 0U)
    {
        // 必须先进入严格的±5cm范围
        if (reached_this_waypoint)
        {
            if (move_arrival_confirm_ticks < ARRIVAL_CONFIRM_TICKS)
            {
                move_arrival_confirm_ticks++;
            }
        }
        else
        {
            // 尚未开始悬停时，离开±5cm就重新确认
            move_arrival_confirm_ticks = 0;
        }

        // 在±5cm范围内连续保持0.5秒，开始2秒悬停
        if (move_arrival_confirm_ticks >= ARRIVAL_CONFIRM_TICKS)
        {
            move_hover_active = 1U;
            move_hover_start_tick = now_tick;
        }
    }
    else
    {
        // 已经开始悬停，但误差超过±8cm
        if (stayed_near_target == 0U)
        {
            // 说明飞机真的飘远了，取消悬停并重新到点
            move_hover_active = 0U;
            move_hover_start_tick = 0U;
            move_arrival_confirm_ticks = 0U;
        }
        else if ((uint32_t)(now_tick - move_hover_start_tick) >=
                 ARRIVAL_HOVER_MS)
        {
            /*
             * 已经悬停满2秒。
             *
             * 如果此刻回到严格的±5cm，立即切换航点。
             * 如果此刻在5～8cm之间，继续PID修正，
             * 但不重新开始2秒计时。
             */
            if (reached_this_waypoint)
            {
                move_hover_active = 0U;
                move_arrival_handled = 1U;

                if (has_next_waypoint)
                {
                 if (Location_SetCurrentWaypointIndex(
								(uint16_t)(current_waypoint_index + 1U)) != 0U)
								{
										Move_StopHorizontal();
								}
                }
                else
                {
                    Location_SetMissionDonePending(111U);
                }
            }
        }
    }
}

#else

// 关闭悬停功能时，只执行到达确认
if (reached_this_waypoint)
{
    if (move_arrival_confirm_ticks < ARRIVAL_CONFIRM_TICKS)
    {
        move_arrival_confirm_ticks++;
    }
}
else
{
    move_arrival_confirm_ticks = 0U;
}

if ((move_arrival_confirm_ticks >= ARRIVAL_CONFIRM_TICKS) &&
    (move_arrival_handled == 0U))
{
    move_arrival_handled = 1U;

    if (has_next_waypoint)
    {
       if (Location_SetCurrentWaypointIndex(
               (uint16_t)(current_waypoint_index + 1U)) != 0U)
			{
					Move_StopHorizontal();
			}
    }
    else
    {
        Location_SetMissionDonePending(111U);
    }
}

#endif

}
const MoveCmd_t *Move_GetLastCmd(void)
{
    return &move_cmd;
}

uint32_t Move_GetLaunchElapsedMs(void)
{
    if ((Decision_IsTaskActive() == 0U) ||
        (move_launch_active == 0U) ||
        (move_launch_run_id != Decision_GetRunId()))
    {
        return 0U;
    }

    return (uint32_t)(HAL_GetTick() - move_launch_start_tick);
}

const MoveVisionDebug_t *Move_GetVisionDebug(void)
{
    return &vision_debug;
}

MoveHeading_t Move_GetHeading(void)
{
    return move_current_heading;
}

uint8_t Move_IsTurning(void)
{
    return move_turning;
}

