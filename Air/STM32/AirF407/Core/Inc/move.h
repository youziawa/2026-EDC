#ifndef __MOVE_H__
#define __MOVE_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

typedef struct
{
    int16_t dx;
    int16_t dy;
    int16_t dz;
    int16_t dyaw;
} MoveCmd_t;

/*
 * Snapshot of the exact K230 pixel-to-centimeter conversion used by
 * the visual controller. USART2 debug output reads this structure so
 * it never performs a second, potentially different calculation.
 */
typedef struct
{
    uint8_t armed;
    uint8_t following;
    uint8_t conversion_valid;
    uint8_t small_range;

    int32_t laser_height_cm;
    float target_distance_cm;

    float raw_body_x_cm;
    float raw_body_y_cm;
    float filtered_body_x_cm;
    float filtered_body_y_cm;

    float camera_fx_px;
    float camera_fy_px;
    float camera_offset_x_cm;
    float camera_offset_y_cm;

    uint32_t source_rx_tick;
} MoveVisionDebug_t;

typedef enum
{
    MOVE_HEADING_0 = 0,
    MOVE_HEADING_90,
    MOVE_HEADING_180,
    MOVE_HEADING_270
} MoveHeading_t;

typedef enum
{
    MOVE_RELATIVE_LEFT_90 = 0,
    MOVE_RELATIVE_RIGHT_90,
    MOVE_RELATIVE_LEFT_180,
    MOVE_RELATIVE_RIGHT_180
} MoveRelativeTurn_t;

typedef enum
{
    MOVE_TURN_INVALID = 0,
    MOVE_TURN_STARTED,
    MOVE_TURN_ALREADY_THERE,
    MOVE_TURN_BUSY
} MoveTurnResult_t;

void Move_Init(void);
void Move_Update(void);

const MoveCmd_t *Move_GetLastCmd(void);

/* Milliseconds since the current TASK_START launch sequence began. */
uint32_t Move_GetLaunchElapsedMs(void);

/* Read-only visual conversion values for ground-station debugging. */
const MoveVisionDebug_t *Move_GetVisionDebug(void);

/*
 * Arm K230 visual takeover. Call once when the mission enters
 * the car-following stage. A valid TRACK frame then takes over XY.
 */
void Move_VisionFollowArm(void);

/*
 * Leave visual following and return control to waypoint navigation.
 */
void Move_VisionFollowStop(void);

/* 1 when K230 currently owns horizontal control; otherwise 0. */
uint8_t Move_IsVisionFollowing(void);

/* 1 after five consecutive centred frames have confirmed a genuine catch. */
uint8_t Move_IsVisionCatchConfirmed(void);

MoveTurnResult_t Move_RequestRelativeTurn(
    MoveRelativeTurn_t turn);

MoveTurnResult_t Move_TurnLeft90(void);
MoveTurnResult_t Move_TurnRight90(void);
MoveTurnResult_t Move_TurnLeft180(void);
MoveTurnResult_t Move_TurnRight180(void);

MoveTurnResult_t Move_TurnToHeading(
    MoveHeading_t heading);

MoveTurnResult_t Move_TurnToYaw01(
    int16_t logical_yaw_01deg);

uint8_t Move_IsTurning(void);
uint8_t Move_TakeTurnFinished(void);

MoveHeading_t Move_GetHeading(void);

uint8_t Move_CalibrateHeading0(
    int16_t current_yaw_01deg);

extern uint16_t cur_waypoint_index;

#ifdef __cplusplus
}
#endif

#endif /* __MOVE_H__ */


