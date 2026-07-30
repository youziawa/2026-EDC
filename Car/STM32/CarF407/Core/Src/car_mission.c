#include "car_mission.h"

#include "car_control.h"
#include "car_link.h"
#include "car_odometry.h"
#include "k230_vision.h"
#include "line_follow.h"

#include <string.h>

#define TASK_MODE_DROP                    1U
#define TASK_MODE_DYNAMIC_LANDING         2U
#ifndef CAR_MISSION_TASK_MODE
#define CAR_MISSION_TASK_MODE             TASK_MODE_DROP
#endif

#define NORMAL_SPEED_MM_S                 150U
#define ACTION_SPEED_MM_S                 80U
#define REACQUIRE_SPEED_MM_S              40U
#define MAP_POINT_FINISH_COUNT            4U
#define POINT_B_PATH_MM                   1500U
#define POINT_C_PATH_MM                   3856U
#define POINT_D_PATH_MM                   5356U
#define LAP_PATH_MM                       7712U
#define START_SAFETY_DELAY_MS             10000U
#define STARTING_SEARCH_SPEED_MM_S        40U
#define STARTING_TIMEOUT_MS               3000U
#define ACTION_SLOW_MAX_MS                25000U
#define STATUS_PERIOD_MS                  100U
#define C_ENTER_PERIOD_MS                 100U
#define HEARTBEAT_PERIOD_MS               1000U
#define FINISH_CONFIRM_MS                 300U
#define GLOBAL_STATE_RETURN               9U

typedef enum
{
  TRACK_EVENT_B_CROSS = 1U,
  TRACK_EVENT_C_ENTER = 2U,
  TRACK_EVENT_D_EXIT = 3U,
  TRACK_EVENT_A_FINISH = 4U
} TrackEvent;

static struct
{
  CarMissionState state;
  CarMissionState pre_loss_state;
  uint8_t run_id;
  uint8_t start_requested;
  uint8_t start_countdown_active;
  uint8_t finish_marker;
  uint8_t map_point_count;
  uint8_t event_sent_mask;
  uint8_t aircraft_global_state;
  uint32_t state_enter_ms;
  uint32_t action_enter_ms;
  uint32_t last_status_ms;
  uint32_t last_c_event_ms;
  uint32_t last_heartbeat_ms;
} mission;

__weak uint8_t CarMission_ReadStartButton(void)
{
  return 0U;
}

__weak uint8_t CarMission_ReadStartButtonRaw(void)
{
  return CarMission_ReadStartButton();
}

static void send_to(CarLinkMask link, uint8_t destination,
                    uint8_t message_id, const uint8_t *data,
                    uint8_t length)
{
  Lxs1Frame frame;
  frame.src = LXS1_NODE_CAR_MCU;
  frame.dst = destination;
  frame.msg_id = message_id;
  frame.data_len = length;
  if ((data != NULL) && (length > 0U))
  {
    memcpy(frame.data, data, length);
  }
  (void)CarLink_Send(link, &frame);
}

static void publish_both(uint8_t message_id, const uint8_t *data,
                         uint8_t length)
{
  send_to(CAR_LINK_AIRCRAFT, LXS1_NODE_AIR_MCU,
          message_id, data, length);
  send_to(CAR_LINK_GROUND, LXS1_NODE_GROUND,
          message_id, data, length);
}

static void enter_state(CarMissionState state)
{
  mission.state = state;
  mission.state_enter_ms = HAL_GetTick();
}

static void stop_and_enter(CarMissionState state)
{
  LineFollow_SetEnabled(0U);
  CarControl_EmergencyStop();
  enter_state(state);
}

static void drive_straight_mm_s(uint16_t speed_mm_s)
{
  int32_t left_pps =
      (int32_t)((float)speed_mm_s / CAR_LEFT_MM_PER_COUNT);
  int32_t right_pps =
      (int32_t)((float)speed_mm_s / CAR_RIGHT_MM_PER_COUNT);
  CarControl_SetWheelTargetsPps(left_pps, right_pps);
}

static void publish_task_start(void)
{
  uint8_t data[6];
  data[0] = mission.run_id;
  data[1] = CAR_MISSION_TASK_MODE;
  Lxs1_PutU16(&data[2], NORMAL_SPEED_MM_S);
  Lxs1_PutU16(&data[4], ACTION_SPEED_MM_S);
  publish_both(LXS1_MSG_TASK_START, data, sizeof(data));
}

static void publish_event(TrackEvent event, uint32_t path_mm)
{
  uint8_t data[6];
  data[0] = mission.run_id;
  data[1] = (uint8_t)event;
  Lxs1_PutU32(&data[2], path_mm);
  publish_both(LXS1_MSG_TRACK_EVENT, data, sizeof(data));
}

static uint8_t line_is_valid(void)
{
  K230VisionData vision;
  return ((K230Vision_GetLatest(&vision) != 0U) &&
          (vision.valid != 0U) &&
          ((HAL_GetTick() - vision.received_at_ms) <= 180U)) ? 1U : 0U;
}

static void publish_diagnostic_ground(const CarOdometryData *odometry)
{
  uint8_t data[52];
  uint8_t flags = 0U;
  uint32_t now_ms = HAL_GetTick();
  uint16_t vision_age_ms = 0xFFFFU;
  K230VisionData vision;
  CarControlSnapshot control;

  memset(&vision, 0, sizeof(vision));
  CarControl_GetSnapshot(&control);

  if (CarMission_ReadStartButtonRaw() != 0U) flags |= (1U << 0);
  if (CarMission_ReadStartButton() != 0U) flags |= (1U << 1);
  if (K230Vision_GetLatest(&vision) != 0U)
  {
    uint32_t age_ms = now_ms - vision.received_at_ms;
    flags |= (1U << 2);
    vision_age_ms =
        (age_ms > 0xFFFEU) ? 0xFFFEU : (uint16_t)age_ms;
    if ((vision.valid != 0U) && (age_ms <= 180U))
      flags |= (1U << 3);
    if ((vision.marker_detected != 0U) && (age_ms <= 180U))
      flags |= (1U << 4);
  }
  if (control.motor_standby != 0U) flags |= (1U << 5);
  if (control.direction_forward != 0U) flags |= (1U << 6);
  if ((control.start_delay_active != 0U) ||
      (mission.start_countdown_active != 0U))
    flags |= (1U << 7);

  data[0] = mission.run_id;
  data[1] = (uint8_t)mission.state;
  data[2] = (uint8_t)LineFollow_GetState();
  data[3] = flags;
  Lxs1_PutU16(&data[4], (uint16_t)control.fault);
  Lxs1_PutU32(&data[6], (uint32_t)control.left_requested_pps);
  Lxs1_PutU32(&data[10], (uint32_t)control.right_requested_pps);
  Lxs1_PutU32(&data[14], (uint32_t)control.left_target_pps);
  Lxs1_PutU32(&data[18], (uint32_t)control.right_target_pps);
  Lxs1_PutU32(&data[22], (uint32_t)control.left_speed_pps);
  Lxs1_PutU32(&data[26], (uint32_t)control.right_speed_pps);
  Lxs1_PutI16(&data[30], control.left_pwm_permille);
  Lxs1_PutI16(&data[32], control.right_pwm_permille);
  Lxs1_PutU32(&data[34], (uint32_t)control.left_total_counts);
  Lxs1_PutU32(&data[38], (uint32_t)control.right_total_counts);
  Lxs1_PutU16(&data[42], vision_age_ms);
  Lxs1_PutU16(&data[44], (uint16_t)vision.frame_count);
  Lxs1_PutU16(&data[46], (uint16_t)vision.malformed_count);
  Lxs1_PutU32(&data[48], odometry->path_mm);

  send_to(CAR_LINK_GROUND, LXS1_NODE_GROUND,
          LXS1_MSG_CAR_DIAGNOSTIC, data, sizeof(data));
}

static void publish_status(const CarOdometryData *odometry)
{
  uint8_t state_data[13];
  uint8_t pose_data[11];
  uint16_t fault = (uint16_t)CarControl_GetFault();
  uint8_t speed_mode = 1U;

  if ((mission.state == CAR_STATE_READY) ||
      (mission.state >= CAR_STATE_DONE))
  {
    speed_mode = 0U;
  }
  else if (mission.state == CAR_STATE_ACTION_SLOW)
  {
    speed_mode = 2U;
  }
  else if ((mission.state == CAR_STATE_STARTING) ||
           (mission.state == CAR_STATE_REACQUIRE) ||
           (mission.state == CAR_STATE_LOST_HOLD))
  {
    speed_mode = 3U;
  }

  state_data[0] = mission.run_id;
  state_data[1] = (uint8_t)mission.state;
  state_data[2] = speed_mode;
  state_data[3] = line_is_valid();
  Lxs1_PutU16(&state_data[4],
              (uint16_t)((odometry->speed_mm_s < 0) ?
                         0 : odometry->speed_mm_s));
  Lxs1_PutU16(&state_data[6], fault);
  Lxs1_PutU32(&state_data[8], odometry->path_mm);
  state_data[12] =
      (mission.map_point_count >= MAP_POINT_FINISH_COUNT) ? 1U : 0U;
  publish_both(LXS1_MSG_CAR_STATE, state_data, sizeof(state_data));

  pose_data[0] = mission.run_id;
  Lxs1_PutI16(&pose_data[1], odometry->x_mm);
  Lxs1_PutI16(&pose_data[3], odometry->y_mm);
  Lxs1_PutI16(&pose_data[5], odometry->heading_deg);
  Lxs1_PutU32(&pose_data[7], odometry->path_mm);
  publish_both(LXS1_MSG_CAR_POSE, pose_data, sizeof(pose_data));
  publish_diagnostic_ground(odometry);
}

static void process_received(void)
{
  CarLinkReceivedFrame received;
  while (CarLink_Receive(&received) != 0U)
  {
    Lxs1Frame *frame = &received.frame;
    if ((frame->dst != LXS1_NODE_CAR_MCU) &&
        (frame->dst != LXS1_NODE_BROADCAST)) continue;

    if ((frame->msg_id == LXS1_MSG_TASK_ABORT) &&
        (frame->data_len >= 2U) &&
        (frame->data[0] == mission.run_id))
    {
      stop_and_enter(CAR_STATE_ABORT);
    }
    else if ((frame->msg_id == LXS1_MSG_TASK_STATE) &&
             (received.source_link == CAR_LINK_AIRCRAFT) &&
             (frame->src == LXS1_NODE_AIR_MCU) &&
             (frame->data_len >= 3U) &&
             (frame->data[0] == mission.run_id))
    {
      mission.aircraft_global_state = frame->data[2];
      if ((mission.aircraft_global_state == GLOBAL_STATE_RETURN) &&
          (mission.state == CAR_STATE_ACTION_SLOW))
      {
        LineFollow_SetCruiseSpeedMmps(NORMAL_SPEED_MM_S);
        enter_state(CAR_STATE_NORMAL_TRACK);
      }
    }
  }
}

static void begin_run(void)
{
  mission.start_countdown_active = 0U;
  mission.run_id++;
  if (mission.run_id == 0U) mission.run_id = 1U;
  mission.finish_marker = 0U;
  mission.map_point_count = 0U;
  mission.event_sent_mask = 0U;
  mission.aircraft_global_state = 0U;
  CarControl_ClearFault();
  CarOdometry_Reset();
  LineFollow_SetCruiseSpeedMmps(NORMAL_SPEED_MM_S);
  LineFollow_SetEnabled(0U);
  enter_state(CAR_STATE_STARTING);
  publish_task_start();
}

static void update_map_section(uint32_t path_mm)
{
  uint8_t reached_point_count = 0U;

  if (path_mm >= LAP_PATH_MM)
    reached_point_count = 4U;
  else if (path_mm >= POINT_D_PATH_MM)
    reached_point_count = 3U;
  else if (path_mm >= POINT_C_PATH_MM)
    reached_point_count = 2U;
  else if (path_mm >= POINT_B_PATH_MM)
    reached_point_count = 1U;

  /*
   * The mapped path is monotonic within one run. Camera marker data remains
   * available for telemetry, but it no longer advances mission point events.
   */
  if (reached_point_count > mission.map_point_count)
    mission.map_point_count = reached_point_count;
  if (mission.map_point_count >= MAP_POINT_FINISH_COUNT)
    mission.finish_marker = 1U;
}

void CarMission_Init(void)
{
  memset(&mission, 0, sizeof(mission));
  mission.state = CAR_STATE_READY;
  mission.pre_loss_state = CAR_STATE_NORMAL_TRACK;
  mission.state_enter_ms = HAL_GetTick();
}

void CarMission_RequestStart(void)
{
  mission.start_requested = 1U;
}

CarMissionState CarMission_GetState(void)
{
  return mission.state;
}

void CarMission_Task(void)
{
  uint32_t now_ms = HAL_GetTick();
  CarOdometryData odometry;
  LineFollowState line_state;

  process_received();
  CarOdometry_Get(&odometry);

  if ((mission.state >= CAR_STATE_NORMAL_TRACK) &&
      (mission.state <= CAR_STATE_REACQUIRE))
    update_map_section(odometry.path_mm);

  if (mission.state == CAR_STATE_READY)
  {
    if ((mission.start_countdown_active == 0U) &&
        ((mission.start_requested != 0U) ||
         (CarMission_ReadStartButton() != 0U)))
    {
      mission.start_requested = 0U;
      mission.start_countdown_active = 1U;
      mission.state_enter_ms = now_ms;
      CarControl_EmergencyStop();
    }
    if ((mission.start_countdown_active != 0U) &&
        ((now_ms - mission.state_enter_ms) >= START_SAFETY_DELAY_MS))
    {
      begin_run();
    }
  }
  else if (mission.state == CAR_STATE_STARTING)
  {
    /*
     * The camera cannot see the A marker from the launch pose. Creep forward
     * through that blind area, then hand over to line following as soon as a
     * fresh drivable line is available. The timeout bounds blind travel to
     * about 120 mm at the configured speed.
     */
    if (line_is_valid() != 0U)
    {
      LineFollow_SetEnabled(1U);
      enter_state(CAR_STATE_NORMAL_TRACK);
    }
    else if ((now_ms - mission.state_enter_ms) >= STARTING_TIMEOUT_MS)
    {
      stop_and_enter(CAR_STATE_FAULT);
    }
    else
    {
      drive_straight_mm_s(STARTING_SEARCH_SPEED_MM_S);
    }
  }
  else if ((mission.state >= CAR_STATE_NORMAL_TRACK) &&
           (mission.state <= CAR_STATE_REACQUIRE))
  {
    if ((CarControl_GetFault() != 0U))
    {
      stop_and_enter(CAR_STATE_FAULT);
    }
    else
    {
      if ((mission.map_point_count >= 1U) &&
          ((mission.event_sent_mask & (1U << TRACK_EVENT_B_CROSS)) == 0U))
      {
        mission.event_sent_mask |= (1U << TRACK_EVENT_B_CROSS);
        publish_event(TRACK_EVENT_B_CROSS, odometry.path_mm);
      }

      if (mission.map_point_count >= 2U)
      {
        if ((mission.event_sent_mask &
             (1U << TRACK_EVENT_C_ENTER)) == 0U)
        {
          mission.event_sent_mask |= (1U << TRACK_EVENT_C_ENTER);
          mission.action_enter_ms = now_ms;
          LineFollow_SetCruiseSpeedMmps(ACTION_SPEED_MM_S);
          mission.pre_loss_state = CAR_STATE_ACTION_SLOW;
          enter_state(CAR_STATE_ACTION_SLOW);
        }
        if ((mission.map_point_count == 2U) &&
            ((now_ms - mission.last_c_event_ms) >= C_ENTER_PERIOD_MS))
        {
          mission.last_c_event_ms = now_ms;
          publish_event(TRACK_EVENT_C_ENTER, odometry.path_mm);
        }
      }

      if ((mission.map_point_count >= 3U) &&
          ((mission.event_sent_mask & (1U << TRACK_EVENT_D_EXIT)) == 0U))
      {
        mission.event_sent_mask |= (1U << TRACK_EVENT_D_EXIT);
        publish_event(TRACK_EVENT_D_EXIT, odometry.path_mm);
        LineFollow_SetCruiseSpeedMmps(NORMAL_SPEED_MM_S);
        mission.pre_loss_state = CAR_STATE_NORMAL_TRACK;
        enter_state(CAR_STATE_NORMAL_TRACK);
      }
      else if ((mission.state == CAR_STATE_ACTION_SLOW) &&
               ((now_ms - mission.action_enter_ms) >=
                ACTION_SLOW_MAX_MS))
      {
        LineFollow_SetCruiseSpeedMmps(NORMAL_SPEED_MM_S);
        mission.pre_loss_state = CAR_STATE_NORMAL_TRACK;
        enter_state(CAR_STATE_NORMAL_TRACK);
      }

      line_state = LineFollow_GetState();
      if (line_state == LINE_FOLLOW_LOST_HOLD)
      {
        if ((mission.state != CAR_STATE_LOST_HOLD) &&
            (mission.state != CAR_STATE_REACQUIRE))
          mission.pre_loss_state = mission.state;
        enter_state(CAR_STATE_LOST_HOLD);
      }
      else if (line_state == LINE_FOLLOW_REACQUIRE)
      {
        enter_state(CAR_STATE_REACQUIRE);
        LineFollow_SetCruiseSpeedMmps(REACQUIRE_SPEED_MM_S);
      }
      else if ((line_state == LINE_FOLLOW_TRACKING) &&
               ((mission.state == CAR_STATE_LOST_HOLD) ||
                (mission.state == CAR_STATE_REACQUIRE)))
      {
        uint16_t restored_speed =
            (mission.pre_loss_state == CAR_STATE_ACTION_SLOW) ?
            ACTION_SPEED_MM_S : NORMAL_SPEED_MM_S;
        LineFollow_SetCruiseSpeedMmps(restored_speed);
        enter_state(mission.pre_loss_state);
      }
      else if (line_state == LINE_FOLLOW_STOPPED)
      {
        stop_and_enter(CAR_STATE_FAULT);
      }

      if (mission.finish_marker != 0U)
      {
        publish_event(TRACK_EVENT_A_FINISH, odometry.path_mm);
        /* LAP_PATH_MM is the mapped A finish boundary for this run. */
        stop_and_enter(CAR_STATE_FINISH_CONFIRM);
      }
    }
  }
  else if ((mission.state == CAR_STATE_FINISH_CONFIRM) &&
           ((now_ms - mission.state_enter_ms) >= FINISH_CONFIRM_MS))
  {
    stop_and_enter(CAR_STATE_DONE);
  }

  if ((now_ms - mission.last_status_ms) >= STATUS_PERIOD_MS)
  {
    mission.last_status_ms = now_ms;
    publish_status(&odometry);
  }
  if ((now_ms - mission.last_heartbeat_ms) >= HEARTBEAT_PERIOD_MS)
  {
    uint8_t data[2] = {mission.run_id, (uint8_t)mission.state};
    mission.last_heartbeat_ms = now_ms;
    publish_both(LXS1_MSG_HEARTBEAT, data, sizeof(data));
  }
}
