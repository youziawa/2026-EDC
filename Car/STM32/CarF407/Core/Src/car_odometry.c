#include "car_odometry.h"

#include "car_control.h"

#include <math.h>

#define ODOMETRY_PERIOD_MS                20U
#define START_X_MM                        1500.0f
#define START_Y_MM                        3000.0f
#define START_HEADING_RAD                 (-1.57079632679f)

static struct
{
  int32_t previous_left;
  int32_t previous_right;
  float x_mm;
  float y_mm;
  float heading_rad;
  float path_mm;
  uint32_t last_task_ms;
  CarOdometryData published;
} odometry;

static int16_t clamp_i16(float value)
{
  if (value > 32767.0f) return 32767;
  if (value < -32768.0f) return -32768;
  return (int16_t)value;
}

void CarOdometry_Reset(void)
{
  CarControl_ResetOdometryCounts();
  odometry.previous_left = 0;
  odometry.previous_right = 0;
  odometry.x_mm = START_X_MM;
  odometry.y_mm = START_Y_MM;
  odometry.heading_rad = START_HEADING_RAD;
  odometry.path_mm = 0.0f;
  odometry.last_task_ms = HAL_GetTick();
  odometry.published.x_mm = (int16_t)START_X_MM;
  odometry.published.y_mm = (int16_t)START_Y_MM;
  odometry.published.heading_deg = -90;
  odometry.published.path_mm = 0U;
  odometry.published.speed_mm_s = 0;
}

void CarOdometry_Init(void)
{
  CarOdometry_Reset();
}

void CarOdometry_Task(void)
{
  CarControlSnapshot snapshot;
  uint32_t now_ms = HAL_GetTick();
  int32_t delta_left_counts;
  int32_t delta_right_counts;
  float delta_left;
  float delta_right;
  float distance;
  float delta_heading;
  float midpoint_heading;
  float speed;

  if ((now_ms - odometry.last_task_ms) < ODOMETRY_PERIOD_MS) return;
  odometry.last_task_ms = now_ms;
  CarControl_GetSnapshot(&snapshot);

  delta_left_counts = snapshot.left_total_counts - odometry.previous_left;
  delta_right_counts = snapshot.right_total_counts - odometry.previous_right;
  odometry.previous_left = snapshot.left_total_counts;
  odometry.previous_right = snapshot.right_total_counts;

  delta_left = delta_left_counts * CAR_LEFT_MM_PER_COUNT;
  delta_right = delta_right_counts * CAR_RIGHT_MM_PER_COUNT;
  distance = (delta_left + delta_right) * 0.5f;
  delta_heading = (delta_right - delta_left) / CAR_WHEEL_TRACK_MM;
  midpoint_heading = odometry.heading_rad + delta_heading * 0.5f;

  odometry.x_mm += distance * cosf(midpoint_heading);
  odometry.y_mm += distance * sinf(midpoint_heading);
  odometry.heading_rad += delta_heading;
  odometry.path_mm += fabsf(distance);
  speed = ((snapshot.left_speed_pps * CAR_LEFT_MM_PER_COUNT) +
           (snapshot.right_speed_pps * CAR_RIGHT_MM_PER_COUNT)) * 0.5f;

  odometry.published.x_mm = clamp_i16(odometry.x_mm);
  odometry.published.y_mm = clamp_i16(odometry.y_mm);
  odometry.published.heading_deg =
      clamp_i16(odometry.heading_rad * 57.2957795f);
  odometry.published.path_mm = (uint32_t)odometry.path_mm;
  odometry.published.speed_mm_s = clamp_i16(speed);
}

void CarOdometry_Get(CarOdometryData *data)
{
  if (data != NULL) *data = odometry.published;
}
