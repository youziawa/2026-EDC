#include "line_follow.h"

#include "car_control.h"
#include "k230_vision.h"

#define LINE_CONTROL_PERIOD_MS            20U
#define LINE_COMM_TIMEOUT_MS              180U
#define LINE_LOST_HOLD_MS                 120U
/* Low-speed fallback for briefly invalid frames while crossing track dots. */
#define LINE_REACQUIRE_MAX_MS             3000U
#define LINE_CONFIDENCE_MIN               380
#define LINE_CONFIDENCE_MAX               1000
#define LINE_MIN_BASE_MM_S                35U
#define LINE_REACQUIRE_MM_S               40U
#define LINE_MIN_INNER_WHEEL_PPS          4000
#define LINE_STEERING_OUTPUT_SIGN         (-1)
#define LINE_LATERAL_KP_PPS_PER_MM        130
#define LINE_HEADING_KP_PPS_PER_DEG       220
#define LINE_LATERAL_DEADBAND_MM          3
#define LINE_HEADING_DEADBAND_DEG         1
#define LINE_CURVATURE_DEADBAND           120
#define LINE_CURVATURE_WEIGHT_PERCENT     35
#define LINE_MAX_CORRECTION_PPS           15000
#define LINE_CORRECTION_FILTER_DIVISOR    6
#define LINE_CORRECTION_MAX_STEP_PPS      2200

typedef struct
{
  int32_t filtered_correction_pps;
  int32_t last_valid_correction_pps;
  uint32_t last_valid_ms;
  uint32_t last_task_ms;
  uint16_t cruise_speed_mm_s;
  uint8_t enabled;
  LineFollowState state;
} LineFollowControl;

static LineFollowControl follow;

static int32_t clamp_i32(int32_t value, int32_t minimum, int32_t maximum)
{
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}

static int32_t abs_i32(int32_t value)
{
  return (value < 0) ? -value : value;
}

static int32_t remove_deadband(int32_t value, int32_t deadband)
{
  if (value > deadband) return value - deadband;
  if (value < -deadband) return value + deadband;
  return 0;
}

static int32_t mmps_to_pps(uint16_t mm_s)
{
  float mean_mm_per_count =
      (CAR_LEFT_MM_PER_COUNT + CAR_RIGHT_MM_PER_COUNT) * 0.5f;
  return (int32_t)((float)mm_s / mean_mm_per_count);
}

static int32_t calculate_base_speed(const K230VisionData *vision)
{
  int32_t confidence = clamp_i32(vision->confidence,
                                 LINE_CONFIDENCE_MIN,
                                 LINE_CONFIDENCE_MAX);
  int32_t minimum = mmps_to_pps(LINE_MIN_BASE_MM_S);
  int32_t cruise = mmps_to_pps(follow.cruise_speed_mm_s);
  int32_t base = minimum +
      (int32_t)(((int64_t)(confidence - LINE_CONFIDENCE_MIN) *
                 (cruise - minimum)) /
                (LINE_CONFIDENCE_MAX - LINE_CONFIDENCE_MIN));
  int32_t penalty =
      abs_i32(vision->lateral_error_mm) * 80 +
      abs_i32(vision->heading_error_deg) * 120;
  return clamp_i32(base - clamp_i32(penalty, 0, 9000),
                   minimum, cruise);
}

static int32_t calculate_steering(const K230VisionData *vision,
                                  int32_t base_speed)
{
  int32_t lateral =
      remove_deadband(vision->lateral_error_mm,
                      LINE_LATERAL_DEADBAND_MM);
  int32_t heading =
      remove_deadband(vision->heading_error_deg,
                      LINE_HEADING_DEADBAND_DEG);
  int32_t curvature =
      remove_deadband(vision->curvature,
                      LINE_CURVATURE_DEADBAND);
  int64_t curvature_numerator =
      (int64_t)base_speed * curvature * CAR_WHEEL_TRACK_X10_MM *
      LINE_CURVATURE_WEIGHT_PERCENT;
  int32_t raw =
      lateral * LINE_LATERAL_KP_PPS_PER_MM +
      heading * LINE_HEADING_KP_PPS_PER_DEG +
      (int32_t)(curvature_numerator / 2000000000LL);
  int32_t correction_limit;
  int32_t filtered_step;

  raw = clamp_i32(raw, -LINE_MAX_CORRECTION_PPS,
                  LINE_MAX_CORRECTION_PPS);
  filtered_step =
      (raw - follow.filtered_correction_pps) /
      LINE_CORRECTION_FILTER_DIVISOR;
  filtered_step = clamp_i32(filtered_step,
                            -LINE_CORRECTION_MAX_STEP_PPS,
                            LINE_CORRECTION_MAX_STEP_PPS);
  follow.filtered_correction_pps += filtered_step;
  correction_limit = base_speed - LINE_MIN_INNER_WHEEL_PPS;
  correction_limit = clamp_i32(correction_limit, 0,
                               LINE_MAX_CORRECTION_PPS);
  return clamp_i32(follow.filtered_correction_pps,
                   -correction_limit, correction_limit);
}

static void set_wheels(int32_t base, int32_t correction)
{
  correction *= LINE_STEERING_OUTPUT_SIGN;
  CarControl_SetWheelTargetsPps(
      clamp_i32(base + correction, 0, CAR_MAX_TARGET_PPS),
      clamp_i32(base - correction, 0, CAR_MAX_TARGET_PPS));
}

static void stop(LineFollowState state)
{
  follow.state = state;
  follow.filtered_correction_pps = 0;
  CarControl_EmergencyStop();
  CarControl_SetLineTelemetry((uint8_t)state, 0, 0, 0, 0U);
}

void LineFollow_Init(void)
{
  follow.filtered_correction_pps = 0;
  follow.last_valid_correction_pps = 0;
  follow.last_valid_ms = 0U;
  follow.last_task_ms = HAL_GetTick();
  follow.cruise_speed_mm_s = 150U;
  follow.enabled = 0U;
  stop(LINE_FOLLOW_DISABLED);
}

void LineFollow_SetEnabled(uint8_t enabled)
{
  follow.enabled = (enabled != 0U) ? 1U : 0U;
  if (follow.enabled == 0U)
  {
    stop(LINE_FOLLOW_DISABLED);
  }
}

void LineFollow_SetCruiseSpeedMmps(uint16_t speed_mm_s)
{
  follow.cruise_speed_mm_s = speed_mm_s;
}

LineFollowState LineFollow_GetState(void)
{
  return follow.state;
}

void LineFollow_Task(void)
{
  K230VisionData vision;
  uint32_t now_ms = HAL_GetTick();
  int32_t base;
  int32_t correction;

  if ((now_ms - follow.last_task_ms) < LINE_CONTROL_PERIOD_MS) return;
  follow.last_task_ms = now_ms;

  if (follow.enabled == 0U)
  {
    /*
     * LineFollow_SetEnabled(0) already stopped the motors once. Do not keep
     * issuing EmergencyStop here: STARTING owns the wheels temporarily while
     * it creeps out of the camera blind area.
     */
    return;
  }

  if ((K230Vision_GetLatest(&vision) == 0U) ||
      ((now_ms - vision.received_at_ms) > LINE_COMM_TIMEOUT_MS))
  {
    stop(LINE_FOLLOW_STOPPED);
    return;
  }

  if ((vision.valid != 0U) &&
      (vision.confidence >= LINE_CONFIDENCE_MIN))
  {
    base = calculate_base_speed(&vision);
    correction = calculate_steering(&vision, base);
    follow.last_valid_correction_pps = correction;
    follow.last_valid_ms = now_ms;
    follow.state = LINE_FOLLOW_TRACKING;
    set_wheels(base, correction);
    CarControl_SetLineTelemetry((uint8_t)follow.state,
                                vision.lateral_error_mm,
                                vision.heading_error_deg,
                                correction, vision.confidence);
    return;
  }

  correction = follow.last_valid_correction_pps / 2;
  base = mmps_to_pps(LINE_REACQUIRE_MM_S);
  correction = clamp_i32(correction,
                         -(base - LINE_MIN_INNER_WHEEL_PPS),
                         base - LINE_MIN_INNER_WHEEL_PPS);
  if ((follow.last_valid_ms != 0U) &&
      ((now_ms - follow.last_valid_ms) <= LINE_LOST_HOLD_MS))
  {
    follow.state = LINE_FOLLOW_LOST_HOLD;
    set_wheels(base, correction);
  }
  else if ((follow.last_valid_ms != 0U) &&
           ((now_ms - follow.last_valid_ms) <=
            (LINE_LOST_HOLD_MS + LINE_REACQUIRE_MAX_MS)))
  {
    follow.state = LINE_FOLLOW_REACQUIRE;
    set_wheels(base, correction);
  }
  else
  {
    stop(LINE_FOLLOW_STOPPED);
    return;
  }
  CarControl_SetLineTelemetry((uint8_t)follow.state,
                              0, 0, correction, 0U);
}
