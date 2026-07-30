#include "car_control.h"

#include "tim.h"
#include "usart.h"

#include <stdio.h>

/*
 * 速度闭环的数据流：
 *
 *   左右轮目标速度（pps）
 *        |
 *        v
 *   左右独立软启动斜坡
 *        |
 *        v
 *   PI控制器：目标速度 - 编码器实测速度
 *        |
 *        v
 *   PWM千分比（0～CAR_PWM_MAX_PERMILLE）
 *        |
 *        v
 *   TB6612 -> 左右电机
 *
 * CarControl_Task() 每 CAR_CONTROL_PERIOD_MS 调用一次 control_step()，
 * control_step() 是整个速度闭环的核心调度函数。
 */

/*
 * TB6612方向控制引脚。
 *
 * 2026-07-30交换左右电机后的实物连接：
 *   PB0/PB1   这一组驱动当前实体左轮（原右电机）；
 *   PB12/PB13 这一组驱动当前实体右轮（原左电机）。
 *
 * 这里必须按照“实体车轮”命名，保证左轮PI读取TIM4后仍然驱动实体左轮，
 * 右轮PI读取TIM2后仍然驱动实体右轮。若写反，会出现一轮狂转，手转另一轮
 * 后两轮交替启停的典型交叉闭环现象。
 */
#define MOTOR_GPIO_PORT                   GPIOB
#define LEFT_MOTOR_IN1_PIN                GPIO_PIN_0
#define LEFT_MOTOR_IN2_PIN                GPIO_PIN_1
#define RIGHT_MOTOR_IN1_PIN               GPIO_PIN_12
#define RIGHT_MOTOR_IN2_PIN               GPIO_PIN_13
#define MOTOR_STBY_PIN                    GPIO_PIN_14

/*
 * PWM硬件映射（同样按实体车轮命名）：
 *   当前实体左轮 PWM：TIM1_CH1，PA8
 *   当前实体右轮 PWM：TIM12_CH2，PB15
 */
#define LEFT_PWM_TIMER                    htim1
#define LEFT_PWM_CHANNEL                  TIM_CHANNEL_1
#define RIGHT_PWM_TIMER                   htim12
#define RIGHT_PWM_CHANNEL                 TIM_CHANNEL_2

/*
 * 编码器硬件映射（已经根据手动转轮测试确认）：
 *   当前实体左轮（原右电机）：TIM4，PB6/PB7，16位计数器
 *   当前实体右轮（原左电机）：TIM2，PA5/PA1，32位计数器
 */
#define LEFT_ENCODER_TIMER                htim4
#define RIGHT_ENCODER_TIMER               htim2

/*
 * [PI参数调整入口]
 *
 * SPEED_FILTER_ALPHA：速度一阶低通滤波系数。
 *   变小 -> 波形更平滑，但响应变慢；变大 -> 响应更快，但更容易抖动。
 *
 * SPEED_KP：比例系数。
 *   增大 -> 追赶目标更快；过大会导致速度振荡和电机声音尖锐。
 *
 * SPEED_KI：积分系数。
 *   增大 -> 更快消除长期速度误差；过大会导致超调和来回波动。
 *
 * LEFT/RIGHT_SPEED_FF_OFFSET、LEFT/RIGHT_SPEED_KFF：
 *   基础PWM = 启动补偿 + 目标速度 * 斜率。
 *
 * 不能把1000 pps附近的PWM比例直接线性外推到20000 pps，否则前馈会超过
 * PWM上限并引发周期性加速/制动。当前参数根据20000 pps实测数据重新标定：
 * 原左电机基础PWM约150，原右电机约135；换边后参数也随电机交换。
 *
 * PWM_SLEW_PER_CONTROL_STEP：
 *   每个20 ms周期允许PWM最多变化多少千分比，用来消除肉眼可见的顿挫。
 */
#define SPEED_FILTER_ALPHA                0.60f
#define SPEED_KP                          0.010f
#define SPEED_KI                          0.050f
#define LEFT_SPEED_FF_OFFSET              70.0f
#define LEFT_SPEED_KFF                    0.00325f
#define RIGHT_SPEED_FF_OFFSET             50.0f
#define RIGHT_SPEED_KFF                   0.0050f
#define PWM_SLEW_PER_CONTROL_STEP         15

/*
 * 允许积分项为负，以便在空载轮速高于前馈估计值时主动减小PWM；
 * 最终PWM仍会被限制在0～CAR_PWM_MAX_PERMILLE。
 */
#define INTEGRAL_MIN                      (-(float)CAR_PWM_MAX_PERMILLE)
#define INTEGRAL_MAX                      ((float)CAR_PWM_MAX_PERMILLE)

/* 连续检测到5次反向速度后才报错，避免单个干扰脉冲触发停车。 */
#define REVERSE_FAULT_SAMPLE_COUNT        5U

/* 每个车轮都有独立的积分项，左右轮分别闭环。 */
typedef struct
{
  float integral;
} SpeedController;

/* 小车控制模块的全部运行状态。 */
typedef struct
{
  int32_t requested_left_pps;       /* 用户要求的最终左轮速度 */
  int32_t requested_right_pps;      /* 用户要求的最终右轮速度 */
  int32_t ramp_left_pps;            /* 经过斜坡限制的当前左轮目标 */
  int32_t ramp_right_pps;           /* 经过斜坡限制的当前右轮目标 */
  int32_t left_speed_pps;           /* 滤波后的左轮实测速度 */
  int32_t right_speed_pps;          /* 滤波后的右轮实测速度 */
  int32_t tim4_delta_counts;        /* TIM4最近一个控制周期的原始有符号计数 */
  int32_t tim2_delta_counts;        /* TIM2最近一个控制周期的原始有符号计数 */
  int32_t tim4_total_counts;        /* TIM4从上电开始累计的原始有符号计数 */
  int32_t tim2_total_counts;        /* TIM2从上电开始累计的原始有符号计数 */
  int16_t left_pwm_permille;        /* 当前左轮PWM，单位千分比 */
  int16_t right_pwm_permille;       /* 当前右轮PWM，单位千分比 */
  uint32_t fault;                   /* CarFault故障位 */
  uint32_t enable_at_ms;            /* 到达这个系统时刻后才允许启动 */
  uint32_t last_control_ms;         /* 上一次执行速度闭环的时刻 */
  uint32_t last_telemetry_ms;       /* 上一次发送VOFA+数据的时刻 */
  uint8_t left_reverse_samples;     /* 左轮连续反向采样次数 */
  uint8_t right_reverse_samples;    /* 右轮连续反向采样次数 */
  uint8_t line_state;               /* 巡线状态：0停车、1跟踪、2短时丢线 */
  int16_t lateral_error_mm;         /* K230横向误差 */
  int16_t heading_error_deg;        /* K230航向误差 */
  int32_t steering_correction_pps;  /* 巡线差速修正量 */
  uint16_t line_confidence;         /* K230置信度0～1000 */
  uint8_t differential_targets;     /* 1表示巡线/差速目标，使用快速目标斜坡 */
  uint8_t initialized;              /* 模块初始化成功标志 */
} CarControlState;

static CarControlState car;
static SpeedController left_controller;
static SpeedController right_controller;
/* 十七个FireWater通道，预留足够空间容纳32位负数。 */
#if CAR_ENABLE_VOFA_TELEMETRY
static char vofa_tx_buffer[256];
#endif

/* 将32位整数限制在指定范围内。 */
static int32_t clamp_i32(int32_t value, int32_t minimum, int32_t maximum)
{
  if (value < minimum)
  {
    return minimum;
  }
  if (value > maximum)
  {
    return maximum;
  }
  return value;
}

/* 将浮点数限制在指定范围内。 */
static float clamp_f32(float value, float minimum, float maximum)
{
  if (value < minimum)
  {
    return minimum;
  }
  if (value > maximum)
  {
    return maximum;
  }
  return value;
}

/*
 * 限制相邻控制周期之间的PWM变化量。
 *
 * 正常闭环输出只能每20 ms增加或减少PWM_SLEW_PER_CONTROL_STEP，避免PI因为
 * 瞬时速度误差把PWM从接近0直接跳到几百。目标为0、故障和紧急停止不经过
 * 本函数，仍会立即关闭PWM和STBY。
 */
static int16_t limit_pwm_slew(int16_t requested, int16_t previous)
{
  int32_t lower =
      (int32_t)previous - (int32_t)PWM_SLEW_PER_CONTROL_STEP;
  int32_t upper =
      (int32_t)previous + (int32_t)PWM_SLEW_PER_CONTROL_STEP;

  return (int16_t)clamp_i32((int32_t)requested, lower, upper);
}

/*
 * 控制TB6612的STBY：
 *   enabled=1 -> 驱动器工作
 *   enabled=0 -> 驱动器待机，输出关闭
 */
static void motor_set_standby(uint8_t enabled)
{
  HAL_GPIO_WritePin(MOTOR_GPIO_PORT, MOTOR_STBY_PIN,
                    enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* 按 car_control.h 中的方向宏设置两侧电机为“前进”方向。 */
static void motor_set_forward_direction(void)
{
  HAL_GPIO_WritePin(MOTOR_GPIO_PORT, LEFT_MOTOR_IN1_PIN,
                    CAR_LEFT_FORWARD_IN1_HIGH ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_GPIO_PORT, LEFT_MOTOR_IN2_PIN,
                    CAR_LEFT_FORWARD_IN1_HIGH ? GPIO_PIN_RESET : GPIO_PIN_SET);

  HAL_GPIO_WritePin(MOTOR_GPIO_PORT, RIGHT_MOTOR_IN1_PIN,
                    CAR_RIGHT_FORWARD_IN1_HIGH ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_GPIO_PORT, RIGHT_MOTOR_IN2_PIN,
                    CAR_RIGHT_FORWARD_IN1_HIGH ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/* 四个方向输入全部置低，让两侧电机处于滑行停止状态。 */
static void motor_coast(void)
{
  HAL_GPIO_WritePin(MOTOR_GPIO_PORT,
                    LEFT_MOTOR_IN1_PIN | LEFT_MOTOR_IN2_PIN |
                    RIGHT_MOTOR_IN1_PIN | RIGHT_MOTOR_IN2_PIN,
                    GPIO_PIN_RESET);
}

/*
 * [最终PWM输出位置]
 *
 * 输入单位是千分比，不是定时器CCR原始值。例如：
 *   100 -> 10%占空比
 *   600 -> 60%占空比
 *
 * TIM1和TIM12的ARR不同，因此分别读取各自周期，再换算成CCR。
 * 这样相同的千分比会得到相同的实际占空比。
 */
static void motor_set_pwm(int16_t left_permille, int16_t right_permille)
{
  uint32_t left_period = __HAL_TIM_GET_AUTORELOAD(&LEFT_PWM_TIMER) + 1U;
  uint32_t right_period = __HAL_TIM_GET_AUTORELOAD(&RIGHT_PWM_TIMER) + 1U;
  uint32_t left_compare;
  uint32_t right_compare;

  left_permille = (int16_t)clamp_i32(left_permille, 0, CAR_PWM_MAX_PERMILLE);
  right_permille = (int16_t)clamp_i32(right_permille, 0, CAR_PWM_MAX_PERMILLE);

  left_compare = (left_period * (uint32_t)left_permille) / 1000U;
  right_compare = (right_period * (uint32_t)right_permille) / 1000U;

  __HAL_TIM_SET_COMPARE(&LEFT_PWM_TIMER, LEFT_PWM_CHANNEL, left_compare);
  __HAL_TIM_SET_COMPARE(&RIGHT_PWM_TIMER, RIGHT_PWM_CHANNEL, right_compare);

  car.left_pwm_permille = left_permille;
  car.right_pwm_permille = right_permille;
}

/*
 * [单轮速度PI控制器]
 *
 * 计算过程：
 *   误差 error = 目标速度 - 实测速度
 *   前馈 feedforward = Kff * 目标速度
 *   输出 output = feedforward + Kp * error + 积分项
 *
 * 返回值为PWM千分比。左右轮分别调用本函数，因此每个轮子可以根据自己的
 * 编码器速度自动增加或减小PWM。
 */
static int16_t speed_pi_update(SpeedController *controller,
                               int32_t target_pps,
                               int32_t measured_pps,
                               float dt_seconds,
                               float feedforward_offset,
                               float feedforward_gain)
{
  float error = (float)(target_pps - measured_pps);
  float feedforward =
      feedforward_offset + feedforward_gain * (float)target_pps;
  float candidate_integral;
  float candidate_output;
  float output;

  candidate_integral =
      clamp_f32(controller->integral + SPEED_KI * error * dt_seconds,
                INTEGRAL_MIN, INTEGRAL_MAX);
  candidate_output =
      feedforward + SPEED_KP * error + candidate_integral;

  /*
   * 条件积分抗饱和：
   * 1. 输出尚未饱和时，正常更新积分；
   * 2. 输出超过上限且误差为负时，允许积分减小；
   * 3. 输出低于下限且误差为正时，允许积分增大。
   *
   * 这样既不会在堵转时无限积累，也能在超调后及时释放已有积分。
   */
  if (((candidate_output >= 0.0f) &&
       (candidate_output <= (float)CAR_PWM_MAX_PERMILLE)) ||
      ((candidate_output > (float)CAR_PWM_MAX_PERMILLE) &&
       (error < 0.0f)) ||
      ((candidate_output < 0.0f) && (error > 0.0f)))
  {
    controller->integral = candidate_integral;
  }

  output = clamp_f32(feedforward + SPEED_KP * error +
                         controller->integral,
                     0.0f, (float)CAR_PWM_MAX_PERMILLE);
  return (int16_t)(output + 0.5f);
}

/*
 * 目标速度斜坡。
 * 防止目标速度从0瞬间跳到500 pps，降低启动电流和机械冲击。
 */
static int32_t update_ramp_target(int32_t current,
                                  int32_t requested,
                                  uint32_t elapsed_ms,
                                  int32_t ramp_pps_per_second)
{
  int32_t step =
      (int32_t)(((int64_t)ramp_pps_per_second * elapsed_ms) / 1000U);

  if (step < 1)
  {
    step = 1;
  }

  if (current < requested)
  {
    current += step;
    if (current > requested)
    {
      current = requested;
    }
  }
  else if (current > requested)
  {
    current -= step;
    if (current < requested)
    {
      current = requested;
    }
  }

  return current;
}

/*
 * [编码器测速]
 *
 * 每个控制周期读取“这段时间内增加了多少个正交计数”，然后立即把CNT清零。
 * pps = 周期内计数差值 * 1000 / 周期毫秒数。
 *
 * 注意：
 *   左轮TIM4是16位计数器，必须先转成int16_t解释反向溢出值；
 *   右轮TIM2是32位计数器，直接转成int32_t即可解释正反方向；
 *   两个定时器的Counter Period不同只影响范围，不影响每个脉冲计数一次。
 */
static void update_encoder_speeds(uint32_t elapsed_ms)
{
  int32_t left_delta;
  int32_t right_delta;
  int32_t left_raw_pps;
  int32_t right_raw_pps;

  /* 当前接线：左轮TIM4（16位），右轮TIM2（32位）。 */
  left_delta =
      (int32_t)(int16_t)__HAL_TIM_GET_COUNTER(&LEFT_ENCODER_TIMER);
  right_delta =
      (int32_t)__HAL_TIM_GET_COUNTER(&RIGHT_ENCODER_TIMER);
  __HAL_TIM_SET_COUNTER(&LEFT_ENCODER_TIMER, 0U);
  __HAL_TIM_SET_COUNTER(&RIGHT_ENCODER_TIMER, 0U);

  /*
   * 保留滤波、方向修正之前的定时器原始数据，专门用于排查TIM2/TIM4
   * 计数倍率差异。当前LEFT_ENCODER_TIMER是TIM4，RIGHT_ENCODER_TIMER是TIM2。
   *
   * delta：最近一个控制周期内的计数，静止时应为0。
   * total：从上电开始累加的净计数，手转完整一圈后读取其绝对值即可得到
   *        该定时器记录的每圈计数，不受手转快慢影响。
   */
  car.tim4_delta_counts = left_delta;
  car.tim2_delta_counts = right_delta;
  car.tim4_total_counts += left_delta;
  car.tim2_total_counts += right_delta;

  left_raw_pps =
      (int32_t)(((int64_t)left_delta * 1000) / (int32_t)elapsed_ms);
  right_raw_pps =
      (int32_t)(((int64_t)right_delta * 1000) / (int32_t)elapsed_ms);

  left_raw_pps *= CAR_LEFT_ENCODER_SIGN;
  right_raw_pps *= CAR_RIGHT_ENCODER_SIGN;

  /*
   * 一阶低通滤波：
   * 新速度 = 旧速度 + alpha * (原始速度 - 旧速度)。
 * 用于减小20 ms短采样窗口造成的速度跳动。
   */
  car.left_speed_pps +=
      (int32_t)(SPEED_FILTER_ALPHA *
                (float)(left_raw_pps - car.left_speed_pps));
  car.right_speed_pps +=
      (int32_t)(SPEED_FILTER_ALPHA *
                (float)(right_raw_pps - car.right_speed_pps));
}

/*
 * 编码器方向安全检查。
 * 已经要求前进且PWM较大时，如果编码器持续报告明显负速度，则关闭驱动器，
 * 防止编码器方向接反后PI控制器不断加大PWM。
 */
static void check_encoder_direction(void)
{
  if ((car.ramp_left_pps > 200) &&
      (car.left_pwm_permille > 150) &&
      (car.left_speed_pps < -CAR_ENCODER_REVERSE_LIMIT_PPS))
  {
    if (++car.left_reverse_samples >= REVERSE_FAULT_SAMPLE_COUNT)
    {
      car.fault |= CAR_FAULT_LEFT_ENCODER_REVERSED;
    }
  }
  else
  {
    car.left_reverse_samples = 0U;
  }

  if ((car.ramp_right_pps > 200) &&
      (car.right_pwm_permille > 150) &&
      (car.right_speed_pps < -CAR_ENCODER_REVERSE_LIMIT_PPS))
  {
    if (++car.right_reverse_samples >= REVERSE_FAULT_SAMPLE_COUNT)
    {
      car.fault |= CAR_FAULT_RIGHT_ENCODER_REVERSED;
    }
  }
  else
  {
    car.right_reverse_samples = 0U;
  }
}

/*
 * [速度闭环核心]
 *
 * 每个20 ms控制周期依次执行：
 *   1. 读取左右编码器并计算实测pps；
 *   2. 根据软启动速度限制更新当前目标；
 *   3. 左右轮分别执行PI计算；
 *   4. 把PI结果写入TIM1/TIM12的PWM；
 *   5. 检查编码器方向是否异常。
 */
static void control_step(uint32_t elapsed_ms, uint32_t now_ms)
{
  float dt_seconds;
  int16_t left_pwm;
  int16_t right_pwm;
  int32_t requested_left;
  int32_t requested_right;
  int32_t target_ramp_rate;

  update_encoder_speeds(elapsed_ms);

  /* 上电延时结束前，内部目标速度保持为0。 */
  if ((int32_t)(now_ms - car.enable_at_ms) >= 0)
  {
    requested_left = car.requested_left_pps;
    requested_right = car.requested_right_pps;
  }
  else
  {
    requested_left = 0;
    requested_right = 0;
  }
  target_ramp_rate =
      (car.differential_targets != 0U) ?
      CAR_DIFFERENTIAL_RAMP_PPS_PER_SECOND :
      CAR_TARGET_RAMP_PPS_PER_SECOND;
  car.ramp_left_pps =
      update_ramp_target(car.ramp_left_pps, requested_left,
                         elapsed_ms, target_ramp_rate);
  car.ramp_right_pps =
      update_ramp_target(car.ramp_right_pps, requested_right,
                         elapsed_ms, target_ramp_rate);

  /*
   * 目标为0或出现故障时，彻底关闭TB6612。
   *
   * 不能只把PWM写成0而继续保持STBY为高：在MCU复位、PWM复用切换或重新烧录
   * 的瞬间，PWM脚可能出现短暂跳变。STBY保持低电平可以切断驱动输出，避免
   * 车轮在零目标状态下突然转动。
   */
  if ((car.fault != CAR_FAULT_NONE) ||
      ((car.ramp_left_pps <= 0) && (car.ramp_right_pps <= 0)))
  {
    motor_set_pwm(0, 0);
    motor_set_standby(0U);
    motor_coast();
    left_controller.integral = 0.0f;
    right_controller.integral = 0.0f;
    return;
  }

  /*
   * 先设置方向和PWM，最后才拉高STBY。这样TB6612真正使能时，输入信号已经
   * 处于确定状态，不会使用上一周期或上电过程中的瞬态电平。
   */
  motor_set_forward_direction();

  /*
   * 左右轮分别使用自己的目标值和实测速度计算PWM：
   *   直行时两个目标相同；
   *   巡线时由上层给出不同目标，底层PI只负责让各轮跟上各自目标。
   */
  dt_seconds = (float)elapsed_ms / 1000.0f;
  if (car.ramp_left_pps > 0)
  {
    left_pwm = speed_pi_update(&left_controller, car.ramp_left_pps,
                               car.left_speed_pps, dt_seconds,
                               LEFT_SPEED_FF_OFFSET,
                               LEFT_SPEED_KFF);
  }
  else
  {
    left_pwm = 0;
    left_controller.integral = 0.0f;
  }

  if (car.ramp_right_pps > 0)
  {
    right_pwm = speed_pi_update(&right_controller, car.ramp_right_pps,
                                car.right_speed_pps, dt_seconds,
                                RIGHT_SPEED_FF_OFFSET,
                                RIGHT_SPEED_KFF);
  }
  else
  {
    right_pwm = 0;
    right_controller.integral = 0.0f;
  }

  /*
   * PI计算的是期望PWM，再以当前实际PWM为起点做斜率限制。
   * 左右轮分别限制，互不影响。
   */
  left_pwm =
      limit_pwm_slew(left_pwm, car.left_pwm_permille);
  right_pwm =
      limit_pwm_slew(right_pwm, car.right_pwm_permille);
  motor_set_pwm(left_pwm, right_pwm);
  motor_set_standby(1U);
  check_encoder_direction();

  if (car.fault != CAR_FAULT_NONE)
  {
    motor_set_pwm(0, 0);
    motor_set_standby(0U);
    motor_coast();
  }
}

#if CAR_ENABLE_VOFA_TELEMETRY
/* 通过USART2向VOFA+发送当前目标、速度、PWM和故障状态。 */
static void vofa_send(void)
{
  int length;
  int32_t average_target_pps;

  if (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY)
  {
    return;
  }

  average_target_pps =
      (int32_t)(((int64_t)car.ramp_left_pps +
                 (int64_t)car.ramp_right_pps) / 2);

  /*
   * VOFA+ FireWater通道顺序：
   *   通道1：当前斜坡目标速度，pps
   *   通道2：左轮实测速度，pps
   *   通道3：右轮实测速度，pps
   *   通道4：左轮PWM，千分比
   *   通道5：右轮PWM，千分比
   *   通道6：故障位
   *   通道7：TIM4最近一个控制周期原始计数（未滤波、未修正方向）
   *   通道8：TIM2最近一个控制周期原始计数（未滤波、未修正方向）
   *   通道9：TIM4上电累计原始计数
   *   通道10：TIM2上电累计原始计数
   *   通道11：左轮斜坡目标速度
   *   通道12：右轮斜坡目标速度
   *   通道13：巡线状态（0停车、1有效跟踪、2短时丢线）
   *   通道14：K230横向误差，mm
   *   通道15：K230航向误差，deg
   *   通道16：巡线差速修正，pps；正数表示右转
   *   通道17：K230置信度，0～1000
   * 换边后通道7/9对应左轮TIM4，通道8/10对应右轮TIM2。
   * 比较两只编码器每圈计数时，应取通道9/10的绝对值。
  */
  length = snprintf(vofa_tx_buffer, sizeof(vofa_tx_buffer),
                    "%ld,%ld,%ld,%d,%d,%lu,%ld,%ld,%ld,%ld,"
                    "%ld,%ld,%u,%d,%d,%ld,%u\r\n",
                    (long)average_target_pps,
                    (long)car.left_speed_pps,
                    (long)car.right_speed_pps,
                    (int)car.left_pwm_permille,
                    (int)car.right_pwm_permille,
                    (unsigned long)car.fault,
                    (long)car.tim4_delta_counts,
                    (long)car.tim2_delta_counts,
                    (long)car.tim4_total_counts,
                    (long)car.tim2_total_counts,
                    (long)car.ramp_left_pps,
                    (long)car.ramp_right_pps,
                    (unsigned int)car.line_state,
                    (int)car.lateral_error_mm,
                    (int)car.heading_error_deg,
                    (long)car.steering_correction_pps,
                    (unsigned int)car.line_confidence);
  if ((length > 0) && ((size_t)length < sizeof(vofa_tx_buffer)))
  {
    (void)HAL_UART_Transmit_IT(&huart2, (uint8_t *)vofa_tx_buffer,
                              (uint16_t)length);
  }
}
#endif

/*
 * 初始化顺序：
 *   1. 先关闭TB6612并清零PWM，避免初始化过程中误转；
 *   2. 启动TIM2/TIM4编码器接口；
 *   3. 启动TIM1/TIM12 PWM；
 *   4. 设置默认目标速度和2秒启动延时；
 *   5. 保持STBY低电平，直到控制任务收到非零目标。
 */
HAL_StatusTypeDef CarControl_Init(void)
{
  uint32_t now_ms = HAL_GetTick();

  motor_set_standby(0U);
  motor_coast();
  motor_set_pwm(0, 0);

  __HAL_TIM_SET_COUNTER(&LEFT_ENCODER_TIMER, 0U);
  __HAL_TIM_SET_COUNTER(&RIGHT_ENCODER_TIMER, 0U);

  if (HAL_TIM_Encoder_Start(&LEFT_ENCODER_TIMER, TIM_CHANNEL_ALL) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (HAL_TIM_Encoder_Start(&RIGHT_ENCODER_TIMER, TIM_CHANNEL_ALL) != HAL_OK)
  {
    (void)HAL_TIM_Encoder_Stop(&LEFT_ENCODER_TIMER, TIM_CHANNEL_ALL);
    return HAL_ERROR;
  }
  if (HAL_TIM_PWM_Start(&LEFT_PWM_TIMER, LEFT_PWM_CHANNEL) != HAL_OK)
  {
    (void)HAL_TIM_Encoder_Stop(&LEFT_ENCODER_TIMER, TIM_CHANNEL_ALL);
    (void)HAL_TIM_Encoder_Stop(&RIGHT_ENCODER_TIMER, TIM_CHANNEL_ALL);
    return HAL_ERROR;
  }
  if (HAL_TIM_PWM_Start(&RIGHT_PWM_TIMER, RIGHT_PWM_CHANNEL) != HAL_OK)
  {
    (void)HAL_TIM_PWM_Stop(&LEFT_PWM_TIMER, LEFT_PWM_CHANNEL);
    (void)HAL_TIM_Encoder_Stop(&LEFT_ENCODER_TIMER, TIM_CHANNEL_ALL);
    (void)HAL_TIM_Encoder_Stop(&RIGHT_ENCODER_TIMER, TIM_CHANNEL_ALL);
    return HAL_ERROR;
  }

  car.requested_left_pps = CAR_DEFAULT_TARGET_PPS;
  car.requested_right_pps = CAR_DEFAULT_TARGET_PPS;
  car.ramp_left_pps = 0;
  car.ramp_right_pps = 0;
  car.left_speed_pps = 0;
  car.right_speed_pps = 0;
  car.tim4_delta_counts = 0;
  car.tim2_delta_counts = 0;
  car.tim4_total_counts = 0;
  car.tim2_total_counts = 0;
  car.left_pwm_permille = 0;
  car.right_pwm_permille = 0;
  car.fault = CAR_FAULT_NONE;
  car.enable_at_ms = now_ms + CAR_START_DELAY_MS;
  car.last_control_ms = now_ms;
  car.last_telemetry_ms = now_ms;
  car.left_reverse_samples = 0U;
  car.right_reverse_samples = 0U;
  car.line_state = 0U;
  car.lateral_error_mm = 0;
  car.heading_error_deg = 0;
  car.steering_correction_pps = 0;
  car.line_confidence = 0U;
  car.differential_targets = 0U;
  car.initialized = 1U;
  left_controller.integral = 0.0f;
  right_controller.integral = 0.0f;

  /*
   * 初始化完成后仍保持驱动器待机。只有control_step()确认目标速度大于0，
   * 并且已经写好方向和PWM之后，才允许拉高STBY。
   */
  motor_set_pwm(0, 0);
  motor_coast();
  motor_set_standby(0U);
  return HAL_OK;
}

/*
 * 主循环任务。
 * main.c 的 while(1) 会持续调用本函数，但真正的速度闭环和串口发送只会在
 * 对应周期到达后执行，不使用HAL_Delay阻塞主循环。
 */
void CarControl_Task(void)
{
  uint32_t now_ms;
  uint32_t elapsed_ms;

  if (car.initialized == 0U)
  {
    return;
  }

  now_ms = HAL_GetTick();
  elapsed_ms = now_ms - car.last_control_ms;
  if (elapsed_ms >= CAR_CONTROL_PERIOD_MS)
  {
    car.last_control_ms = now_ms;
    control_step(elapsed_ms, now_ms);
  }

#if CAR_ENABLE_VOFA_TELEMETRY
  if ((now_ms - car.last_telemetry_ms) >= CAR_TELEMETRY_PERIOD_MS)
  {
    car.last_telemetry_ms = now_ms;
    vofa_send();
  }
#endif
}

/*
 * [运行时速度调整接口]
 *
 * 例如 CarControl_SetTargetPps(800) 会把目标速度设为800 pps。
 * 设为0会按照斜坡减速；紧急情况应直接调用CarControl_EmergencyStop()。
 */
void CarControl_SetTargetPps(int32_t target_pps)
{
  target_pps = clamp_i32(target_pps, 0, CAR_MAX_TARGET_PPS);
  car.requested_left_pps = target_pps;
  car.requested_right_pps = target_pps;
  car.differential_targets = 0U;
}

void CarControl_SetWheelTargetsPps(int32_t left_target_pps,
                                   int32_t right_target_pps)
{
  car.requested_left_pps =
      clamp_i32(left_target_pps, 0, CAR_MAX_TARGET_PPS);
  car.requested_right_pps =
      clamp_i32(right_target_pps, 0, CAR_MAX_TARGET_PPS);
  car.differential_targets = 1U;
}

void CarControl_SetLineTelemetry(uint8_t line_state,
                                 int16_t lateral_error_mm,
                                 int16_t heading_error_deg,
                                 int32_t steering_correction_pps,
                                 uint16_t confidence)
{
  car.line_state = line_state;
  car.lateral_error_mm = lateral_error_mm;
  car.heading_error_deg = heading_error_deg;
  car.steering_correction_pps = steering_correction_pps;
  car.line_confidence = confidence;
}

/* 紧急停止：立即关闭PWM和TB6612，不等待速度斜坡。 */
void CarControl_EmergencyStop(void)
{
  car.requested_left_pps = 0;
  car.requested_right_pps = 0;
  car.ramp_left_pps = 0;
  car.ramp_right_pps = 0;
  left_controller.integral = 0.0f;
  right_controller.integral = 0.0f;
  motor_set_pwm(0, 0);
  motor_set_standby(0U);
  motor_coast();
}

/*
 * 清除故障并重置PI积分。
 * 清故障不等于立即启动电机，仍保持STBY低，等待下一次非零目标控制周期。
 */
void CarControl_ClearFault(void)
{
  car.fault = CAR_FAULT_NONE;
  car.left_reverse_samples = 0U;
  car.right_reverse_samples = 0U;
  car.ramp_left_pps = 0;
  car.ramp_right_pps = 0;
  left_controller.integral = 0.0f;
  right_controller.integral = 0.0f;
  motor_set_pwm(0, 0);
  motor_coast();
  motor_set_standby(0U);
}

/* 返回当前故障位，0表示没有故障。 */
uint32_t CarControl_GetFault(void)
{
  return car.fault;
}

void CarControl_GetSnapshot(CarControlSnapshot *snapshot)
{
  if (snapshot == NULL)
  {
    return;
  }

  snapshot->left_total_counts =
      car.tim4_total_counts * CAR_LEFT_ENCODER_SIGN;
  snapshot->right_total_counts =
      car.tim2_total_counts * CAR_RIGHT_ENCODER_SIGN;
  snapshot->left_speed_pps = car.left_speed_pps;
  snapshot->right_speed_pps = car.right_speed_pps;
  snapshot->left_requested_pps = car.requested_left_pps;
  snapshot->right_requested_pps = car.requested_right_pps;
  snapshot->left_target_pps = car.ramp_left_pps;
  snapshot->right_target_pps = car.ramp_right_pps;
  snapshot->left_pwm_permille = car.left_pwm_permille;
  snapshot->right_pwm_permille = car.right_pwm_permille;
  snapshot->fault = car.fault;
  snapshot->motor_standby =
      (HAL_GPIO_ReadPin(MOTOR_GPIO_PORT, MOTOR_STBY_PIN) ==
       GPIO_PIN_SET) ? 1U : 0U;
  snapshot->direction_forward =
      ((HAL_GPIO_ReadPin(MOTOR_GPIO_PORT, LEFT_MOTOR_IN1_PIN) ==
        (CAR_LEFT_FORWARD_IN1_HIGH ? GPIO_PIN_SET : GPIO_PIN_RESET)) &&
       (HAL_GPIO_ReadPin(MOTOR_GPIO_PORT, LEFT_MOTOR_IN2_PIN) ==
        (CAR_LEFT_FORWARD_IN1_HIGH ? GPIO_PIN_RESET : GPIO_PIN_SET)) &&
       (HAL_GPIO_ReadPin(MOTOR_GPIO_PORT, RIGHT_MOTOR_IN1_PIN) ==
        (CAR_RIGHT_FORWARD_IN1_HIGH ? GPIO_PIN_SET : GPIO_PIN_RESET)) &&
       (HAL_GPIO_ReadPin(MOTOR_GPIO_PORT, RIGHT_MOTOR_IN2_PIN) ==
        (CAR_RIGHT_FORWARD_IN1_HIGH ? GPIO_PIN_RESET : GPIO_PIN_SET))) ?
      1U : 0U;
  snapshot->start_delay_active =
      ((int32_t)(car.enable_at_ms - HAL_GetTick()) > 0) ? 1U : 0U;
}

void CarControl_ResetOdometryCounts(void)
{
  __HAL_TIM_SET_COUNTER(&LEFT_ENCODER_TIMER, 0U);
  __HAL_TIM_SET_COUNTER(&RIGHT_ENCODER_TIMER, 0U);
  car.tim2_delta_counts = 0;
  car.tim4_delta_counts = 0;
  car.tim2_total_counts = 0;
  car.tim4_total_counts = 0;
}
