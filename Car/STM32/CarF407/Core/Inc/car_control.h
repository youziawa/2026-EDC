#ifndef CAR_CONTROL_H
#define CAR_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/*
 * 小车速度控制参数
 *
 * 当前速度单位为 pps（encoder quadrature counts per second），即编码器经过
 * STM32 正交解码之后，每秒累计的计数值。
 *
 * MG310MGR编码器和47 mm车轮已经完成实测标定：
 * 2026-07-30更换高强度底板并交换左右电机后：
 *   当前实体左轮（原右电机）：41589 count/rev
 *   当前实体右轮（原左电机）：41348 count/rev
 *
 * 下列常量均使用计数绝对值；编码器正负方向仍由CAR_*_ENCODER_SIGN处理。
 */
#define CAR_WHEEL_DIAMETER_MM             47.0f
#define CAR_WHEEL_TRACK_MM                432.0f
#define CAR_WHEEL_TRACK_X10_MM            4320
#define CAR_WHEEL_CIRCUMFERENCE_MM        147.654855f
#define CAR_LEFT_COUNTS_PER_REV           41589.0f
#define CAR_RIGHT_COUNTS_PER_REV          41348.0f
/*
 * 里程标定入口。地图一圈为7712 mm，实车在旧系数1.0下越过A点约400 mm
 * 才达到7712 mm，因此按(7712 + 400) / 7712 = 1.05187修正。
 */
#define CAR_ODOMETRY_CALIBRATION          1.05187f
#define CAR_LEFT_MM_PER_COUNT             \
  ((CAR_WHEEL_CIRCUMFERENCE_MM / CAR_LEFT_COUNTS_PER_REV) * \
   CAR_ODOMETRY_CALIBRATION)
#define CAR_RIGHT_MM_PER_COUNT            \
  ((CAR_WHEEL_CIRCUMFERENCE_MM / CAR_RIGHT_COUNTS_PER_REV) * \
   CAR_ODOMETRY_CALIBRATION)

#define CAR_CONTROL_PERIOD_MS             20U    /* 低速闭环周期：20 ms，增加单次编码器计数 */
#define CAR_TELEMETRY_PERIOD_MS           20U    /* VOFA+ 数据发送周期：每 20 ms 一帧 */
#define CAR_START_DELAY_MS                2000U  /* 上电后等待 2 s 再启动电机 */
#define CAR_ENABLE_VOFA_TELEMETRY         0U     /* USART2用于飞机ECB02二进制链路 */

/*
 * [速度调整入口]
 *
 * CAR_DEFAULT_TARGET_PPS：上电后的默认目标速度，调快/调慢小车优先修改这里。
 * CAR_MAX_TARGET_PPS：CarControl_SetTargetPps() 允许设置的最大目标速度。
 * CAR_TARGET_RAMP_PPS_PER_SECOND：目标速度每秒最多增加多少 pps，用于软启动。
 */
#define CAR_DEFAULT_TARGET_PPS            4000
#define CAR_MAX_TARGET_PPS                100000
#define CAR_TARGET_RAMP_PPS_PER_SECOND    5000
#define CAR_DIFFERENTIAL_RAMP_PPS_PER_SECOND 100000

/*
 * PWM 使用千分比表示：600 表示最大允许 60% 占空比。
 * 首次调车时不要直接改成 1000，以免闭环接反时电机全速运行。
 */
#define CAR_PWM_MAX_PERMILLE              600

/* 编码器反向检测阈值：速度小于 -100 pps 才视为明显反转。 */
#define CAR_ENCODER_REVERSE_LIMIT_PPS     100

/*
 * [编码器方向调整]
 *
 * 手动向小车前进方向转动轮子时，对应的 VOFA+ 速度必须为正数。
 * 如果某一轮显示负数，只修改该轮符号：1 改为 -1，或者 -1 改为 1。
 *
 * 当前实测接线：
 *   驱动板E1A/E1B，左轮（原右电机）-> TIM4（PB6、PB7）
 *   驱动板E2A/E2B，右轮（原左电机）-> TIM2（PA5、PA1）
 */
#define CAR_LEFT_ENCODER_SIGN             (-1)
#define CAR_RIGHT_ENCODER_SIGN            1

/*
 * [电机方向调整]
 *
 * 该值为 1 时：IN1=高、IN2=低；该值为 0 时：IN1=低、IN2=高。
 * 如果架空车轮测试时某一轮向后转，只修改对应轮的宏。
 *
 * 注意：修改该宏只改变旋转方向，不会交换左右电机通道。改变电机方向后，
 * 必须重新确认对应编码器的速度仍然为正数。
 */
#define CAR_LEFT_FORWARD_IN1_HIGH         0U
#define CAR_RIGHT_FORWARD_IN1_HIGH        0U

/* 故障位可以组合，例如 fault=3 表示左右编码器方向都异常。 */
typedef enum
{
  CAR_FAULT_NONE = 0U,
  CAR_FAULT_LEFT_ENCODER_REVERSED = (1U << 0),
  CAR_FAULT_RIGHT_ENCODER_REVERSED = (1U << 1)
} CarFault;

typedef struct
{
  int32_t left_total_counts;
  int32_t right_total_counts;
  int32_t left_speed_pps;
  int32_t right_speed_pps;
  int32_t left_requested_pps;
  int32_t right_requested_pps;
  int32_t left_target_pps;
  int32_t right_target_pps;
  int16_t left_pwm_permille;
  int16_t right_pwm_permille;
  uint32_t fault;
  uint8_t motor_standby;
  uint8_t direction_forward;
  uint8_t start_delay_active;
} CarControlSnapshot;

/* 初始化编码器、PWM和控制状态；所有 CubeMX 外设初始化完成后调用一次。 */
HAL_StatusTypeDef CarControl_Init(void);

/* 放在 while(1) 中持续调用，函数内部按照设定周期执行闭环和数据发送。 */
void CarControl_Task(void);

/* 运行过程中修改目标速度，单位 pps，输入值会被限制在 0～最大速度。 */
void CarControl_SetTargetPps(int32_t target_pps);

/*
 * 分别设置实体左轮和右轮目标速度，供差速转向/巡线使用。
 * 当前电机控制只支持前进，因此负值会被限制为0。
 */
void CarControl_SetWheelTargetsPps(int32_t left_target_pps,
                                   int32_t right_target_pps);

/* 把巡线诊断量附加到VOFA+数据，不参与底层速度PI计算。 */
void CarControl_SetLineTelemetry(uint8_t line_state,
                                 int16_t lateral_error_mm,
                                 int16_t heading_error_deg,
                                 int32_t steering_correction_pps,
                                 uint16_t confidence);

/* 紧急停车：目标清零、PWM清零、TB6612进入待机并让电机滑行停止。 */
void CarControl_EmergencyStop(void);

/* 清除编码器方向故障，并让控制器从 0 速度重新开始。 */
void CarControl_ClearFault(void);

/* 读取当前故障位。 */
uint32_t CarControl_GetFault(void);

/* 获取里程计和任务状态机使用的原子快照，计数方向已转换为前进为正。 */
void CarControl_GetSnapshot(CarControlSnapshot *snapshot);

/* 任务开始时清零编码器硬件计数和软件累计值。 */
void CarControl_ResetOdometryCounts(void);

#ifdef __cplusplus
}
#endif

#endif /* CAR_CONTROL_H */
