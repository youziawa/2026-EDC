#include "decision.h"
#include "main.h"
#include "move.h"
#include "telecom.h"
#include "location.h"

/* PB4 high: magnet holds the payload; PB4 low: payload released/off. */
#define MAGNET_GPIO_PORT GPIOB
#define MAGNET_GPIO_PIN  GPIO_PIN_4

/* Release only after the aircraft is stably centered over the target. */
#define DROP_CENTER_TOL_CM 8.0f
#define DROP_CONFIRM_MS    1000U

/* Car odometry boundaries for the mapped C -> D payload section. */
#define DROP_SECTION_C_PATH_MM 3856U
#define DROP_SECTION_D_PATH_MM 5356U
#define DROP_MIN_FOLLOW_MS     12000U
#define CAR_STATE_ACTION_SLOW  3U
#define DROP_POINT_B_X_CM      200
#define DROP_POINT_B_Y_CM      (-40)
#define DROP_B_CLEARANCE_CM    80

/* Task 2: keep visual XY control active while lowering the height in steps. */
#define DYNAMIC_CRUISE_HEIGHT_CM          125
#define DYNAMIC_START_HEIGHT_MAX_CM       150
#define DYNAMIC_APPROACH_HEIGHT_CM         20
#define DYNAMIC_LOCK_CENTER_TOL_CM         15.0f
#define DYNAMIC_LOW_CENTER_TOL_CM          12.0f
#define DYNAMIC_LOCK_CONFIRM_MS           800U
#define DYNAMIC_FINAL_CENTER_CONFIRM_MS   300U
#define DYNAMIC_FINAL_LAND_START_CM        25
#define DYNAMIC_TOUCHDOWN_HEIGHT_CM        12
#define DYNAMIC_TOUCHDOWN_CONFIRM_MS      800U
#define DYNAMIC_PLATFORM_WAIT_MS         5000U
#define DYNAMIC_RETAKEOFF_CONFIRM_CM       110
#define DYNAMIC_RETAKEOFF_CONFIRM_MS      3000U
#define INITIAL_TAKEOFF_TOTAL_MS          6000U

volatile int decision = 0;

static uint8_t task_mode = AIR_TASK_NONE;
static uint8_t task_run_id = 0U;
static uint8_t task_active = 0U;
static uint8_t magnet_on = 0U;
static uint8_t payload_released = 0U;
static uint8_t drop_confirming = 0U;
static uint32_t drop_confirm_start_tick = 0U;
static uint8_t return_started = 0U;
static uint8_t drop_follow_timing = 0U;
static uint32_t drop_follow_start_tick = 0U;

static uint8_t dynamic_state = AIR_DYNAMIC_IDLE;
static int16_t dynamic_target_height_cm = DYNAMIC_CRUISE_HEIGHT_CM;
static uint32_t dynamic_state_start_tick = 0U;
static uint32_t dynamic_center_start_tick = 0U;
static uint32_t dynamic_touchdown_start_tick = 0U;
static uint32_t dynamic_retakeoff_start_tick = 0U;
static uint8_t dynamic_center_timing = 0U;
static uint8_t dynamic_touchdown_timing = 0U;
static uint8_t dynamic_retakeoff_timing = 0U;

static void Decision_SetMagnet(uint8_t enable)
{
    magnet_on = (enable != 0U) ? 1U : 0U;

    HAL_GPIO_WritePin(MAGNET_GPIO_PORT,
                      MAGNET_GPIO_PIN,
                      (magnet_on != 0U) ?
                          GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void Decision_ResetDynamicLanding(void)
{
    dynamic_state = AIR_DYNAMIC_IDLE;
    dynamic_target_height_cm = DYNAMIC_CRUISE_HEIGHT_CM;
    dynamic_state_start_tick = 0U;
    dynamic_center_start_tick = 0U;
    dynamic_touchdown_start_tick = 0U;
    dynamic_retakeoff_start_tick = 0U;
    dynamic_center_timing = 0U;
    dynamic_touchdown_timing = 0U;
    dynamic_retakeoff_timing = 0U;
}

static uint8_t Decision_IsVisionCentered(float tolerance_cm)
{
    const MoveVisionDebug_t *vision;

    if ((Move_IsVisionFollowing() == 0U) ||
        (Telecom_IsVisionValid() == 0U))
    {
        return 0U;
    }

    vision = Move_GetVisionDebug();
    if ((vision == 0) ||
        (vision->conversion_valid == 0U) ||
        (vision->filtered_body_x_cm < -tolerance_cm) ||
        (vision->filtered_body_x_cm > tolerance_cm) ||
        (vision->filtered_body_y_cm < -tolerance_cm) ||
        (vision->filtered_body_y_cm > tolerance_cm))
    {
        return 0U;
    }

    return 1U;
}

static void Decision_StartReturnHome(void)
{
    Location_StartReturnHome();
    Move_VisionFollowStop();
    return_started = 1U;
    Telecom_NotifyCarReturn();
}

static void Decision_UpdateDynamicLanding(void)
{
    uint32_t now_tick;
    int32_t actual_height_cm;

    now_tick = HAL_GetTick();

    switch (dynamic_state)
    {
    case AIR_DYNAMIC_WAIT_LOCK:
        /* Follow from takeoff, but crossing B is the sole landing permission. */
        if (Telecom_IsCarLandingPermitted() == 0U)
        {
            dynamic_center_timing = 0U;
            return;
        }

        /*
         * Reuse Move's five-frame catch latch. The old independent 10 cm
         * gate was stricter than the follow controller and could remain
         * blocked forever behind a slowly moving car.
         */
        if ((Move_IsVisionCatchConfirmed() == 0U) ||
            (Decision_IsVisionCentered(DYNAMIC_LOCK_CENTER_TOL_CM) == 0U))
        {
            dynamic_center_timing = 0U;
            return;
        }

        if (dynamic_center_timing == 0U)
        {
            dynamic_center_timing = 1U;
            dynamic_center_start_tick = now_tick;
            return;
        }

        if ((uint32_t)(now_tick - dynamic_center_start_tick) >=
            DYNAMIC_LOCK_CONFIRM_MS)
        {
            /* Require a fresh, sane height before commanding direct descent. */
            if ((Telecom_GetFlightHeightCm(&actual_height_cm) == 0U) ||
                (actual_height_cm < DYNAMIC_APPROACH_HEIGHT_CM) ||
                (actual_height_cm > DYNAMIC_START_HEIGHT_MAX_CM))
            {
                return;
            }

            /* Descend directly to 20 cm while visual XY keeps following. */
            dynamic_target_height_cm = DYNAMIC_APPROACH_HEIGHT_CM;

            dynamic_state = AIR_DYNAMIC_DESCENDING;
            dynamic_state_start_tick = now_tick;
            dynamic_center_timing = 0U;
        }
        return;

    case AIR_DYNAMIC_DESCENDING:
        if (Telecom_GetFlightHeightCm(&actual_height_cm) == 0U)
        {
            /* No height feedback: hold the last safe target. */
            dynamic_center_timing = 0U;
            return;
        }

        /* Keep descending directly to 20 cm; centre before final landing. */
        if (actual_height_cm > DYNAMIC_FINAL_LAND_START_CM)
        {
            dynamic_center_timing = 0U;
            return;
        }

        if (Decision_IsVisionCentered(DYNAMIC_LOW_CENTER_TOL_CM) == 0U)
        {
            dynamic_center_timing = 0U;
            return;
        }

        if (dynamic_center_timing == 0U)
        {
            dynamic_center_timing = 1U;
            dynamic_center_start_tick = now_tick;
            return;
        }

        if ((uint32_t)(now_tick - dynamic_center_start_tick) >=
            DYNAMIC_FINAL_CENTER_CONFIRM_MS)
        {
            Location_SetMissionDonePending(FLIGHT_ACTION_LAND_PLATFORM);
            dynamic_state = AIR_DYNAMIC_FINAL_LAND;
            dynamic_state_start_tick = now_tick;
            dynamic_center_timing = 0U;
            dynamic_touchdown_timing = 0U;
        }
        return;

    case AIR_DYNAMIC_FINAL_LAND:
        if ((Telecom_GetFlightHeightCm(&actual_height_cm) == 0U) ||
            (actual_height_cm > DYNAMIC_TOUCHDOWN_HEIGHT_CM))
        {
            dynamic_touchdown_timing = 0U;
            return;
        }

        if (dynamic_touchdown_timing == 0U)
        {
            dynamic_touchdown_timing = 1U;
            dynamic_touchdown_start_tick = now_tick;
            return;
        }

        if ((uint32_t)(now_tick - dynamic_touchdown_start_tick) >=
            DYNAMIC_TOUCHDOWN_CONFIRM_MS)
        {
            dynamic_state = AIR_DYNAMIC_WAIT_5S;
            dynamic_state_start_tick = now_tick;
            dynamic_touchdown_timing = 0U;
        }
        return;

    case AIR_DYNAMIC_WAIT_5S:
        if ((uint32_t)(now_tick - dynamic_state_start_tick) >=
            DYNAMIC_PLATFORM_WAIT_MS)
        {
            dynamic_target_height_cm = DYNAMIC_CRUISE_HEIGHT_CM;
            Location_SetMissionDonePending(FLIGHT_ACTION_RETAKEOFF);
            dynamic_state = AIR_DYNAMIC_RETAKEOFF;
            dynamic_state_start_tick = now_tick;
            dynamic_retakeoff_timing = 0U;
        }
        return;

    case AIR_DYNAMIC_RETAKEOFF:
        /* Reuse Task 1's three-second stable hover at the takeoff height. */
        if ((Telecom_GetFlightHeightCm(&actual_height_cm) == 0U) ||
            (actual_height_cm < DYNAMIC_RETAKEOFF_CONFIRM_CM))
        {
            dynamic_retakeoff_timing = 0U;
            return;
        }

        if (dynamic_retakeoff_timing == 0U)
        {
            dynamic_retakeoff_timing = 1U;
            dynamic_retakeoff_start_tick = now_tick;
            return;
        }

        if ((uint32_t)(now_tick - dynamic_retakeoff_start_tick) >=
            DYNAMIC_RETAKEOFF_CONFIRM_MS)
        {
            dynamic_state = AIR_DYNAMIC_COMPLETE;
            Decision_StartReturnHome();
        }
        return;

    case AIR_DYNAMIC_COMPLETE:
    case AIR_DYNAMIC_FAILED:
    case AIR_DYNAMIC_IDLE:
    default:
        return;
    }
}

void Decision_Init(void)
{
    task_mode = AIR_TASK_NONE;
    task_run_id = 0U;
    task_active = 0U;
    payload_released = 0U;
    drop_confirming = 0U;
    drop_confirm_start_tick = 0U;
    return_started = 0U;
    drop_follow_timing = 0U;
    drop_follow_start_tick = 0U;
    Decision_ResetDynamicLanding();
    decision = 0;

    /* Hold the preloaded payload before the car selects a task. */
    Decision_SetMagnet(1U);
}

void Decision_OnTaskStart(uint8_t run_id, uint8_t new_task_mode)
{
    /* Ignore malformed task modes. */
    if ((new_task_mode != AIR_TASK_DROP) &&
        (new_task_mode != AIR_TASK_DYNAMIC_LANDING))
    {
        return;
    }

    /* Duplicate TASK_START is idempotent, especially after a completed drop. */
    if ((task_active != 0U) &&
        (task_run_id == run_id) &&
        (task_mode == new_task_mode))
    {
        return;
    }

    task_run_id = run_id;
    task_mode = new_task_mode;
    task_active = 1U;
    payload_released = 0U;
    drop_confirming = 0U;
    drop_confirm_start_tick = 0U;
    return_started = 0U;
    drop_follow_timing = 0U;
    drop_follow_start_tick = 0U;
    Decision_ResetDynamicLanding();
    decision = (int)new_task_mode;

    if (task_mode == AIR_TASK_DROP)
    {
        /* Task 1 carries a sandbag, so energize the magnet. */
        Decision_SetMagnet(1U);
    }
    else
    {
        /* Task 2 never drops a payload; keep the magnet off to save power. */
        Decision_SetMagnet(0U);
        dynamic_state = AIR_DYNAMIC_WAIT_LOCK;
        dynamic_state_start_tick = HAL_GetTick();
    }
}

void Decision_OnTaskAbort(uint8_t run_id)
{
    if ((task_active != 0U) &&
        (run_id != task_run_id))
    {
        return;
    }

    task_active = 0U;
    drop_confirming = 0U;
    return_started = 0U;
    drop_follow_timing = 0U;
    Decision_ResetDynamicLanding();
    Decision_SetMagnet(0U);
}

void Decision_Update(void)
{
    const MoveVisionDebug_t *vision;
    const CarStatusData_t *car;
    uint32_t now_tick;
    int32_t from_b_x_cm;
    int32_t from_b_y_cm;

    if (return_started != 0U)
    {
        drop_confirming = 0U;
        Decision_SetMagnet(0U);
        Telecom_NotifyCarReturn();
        return;
    }

    /* Before the first task command, keep holding the preloaded payload. */
    if ((task_active == 0U) &&
        (task_mode == AIR_TASK_NONE))
    {
        drop_confirming = 0U;
        Decision_SetMagnet(1U);
        return;
    }

    if ((task_active != 0U) &&
        (task_mode == AIR_TASK_DYNAMIC_LANDING))
    {
        Decision_SetMagnet(0U);
        Decision_UpdateDynamicLanding();
        return;
    }

    /* Every other non-drop state is low, including task 2 and abort. */
    if ((task_active == 0U) ||
        (task_mode != AIR_TASK_DROP) ||
        (payload_released != 0U))
    {
        drop_confirming = 0U;
        Decision_SetMagnet(0U);
        return;
    }

    /* Task 1 must retain the payload until every drop condition is valid. */
    Decision_SetMagnet(1U);

    /* Start the independent timer as soon as visual following really begins. */
    if (Move_IsVisionFollowing() == 0U)
    {
        drop_follow_timing = 0U;
        drop_confirming = 0U;
        return;
    }

    now_tick = HAL_GetTick();
    if (drop_follow_timing == 0U)
    {
        drop_follow_timing = 1U;
        drop_follow_start_tick = now_tick;
        drop_confirming = 0U;
        return;
    }

    /* Never release before C or after D, and reject stale/wrong-run data. */
    if (Telecom_IsCarStatusValid() == 0U)
    {
        drop_confirming = 0U;
        return;
    }

    car = Telecom_GetCarStatus();
    if ((car == 0) ||
        (car->run_id != task_run_id) ||
        (car->state != CAR_STATE_ACTION_SLOW) ||
        (car->path_mm < DROP_SECTION_C_PATH_MM) ||
        (car->path_mm >= DROP_SECTION_D_PATH_MM))
    {
        drop_confirming = 0U;
        return;
    }

    /* A second, SLAM-based guard: never release in the B-point area. */
    if (Telecom_IsPoseValid() == 0U)
    {
        drop_confirming = 0U;
        return;
    }

    from_b_x_cm = (int32_t)pose_data.x - DROP_POINT_B_X_CM;
    from_b_y_cm = (int32_t)pose_data.y - DROP_POINT_B_Y_CM;
    if (((from_b_x_cm * from_b_x_cm) +
         (from_b_y_cm * from_b_y_cm)) <
        ((int32_t)DROP_B_CLEARANCE_CM * DROP_B_CLEARANCE_CM))
    {
        drop_confirming = 0U;
        return;
    }

    /* Independent guard against odometry making B look like C. */
    if ((uint32_t)(now_tick - drop_follow_start_tick) <
        DROP_MIN_FOLLOW_MS)
    {
        drop_confirming = 0U;
        return;
    }

    if (Telecom_IsVisionValid() == 0U)
    {
        drop_confirming = 0U;
        return;
    }

    vision = Move_GetVisionDebug();
    if ((vision == 0) ||
        (vision->conversion_valid == 0U) ||
        (vision->filtered_body_x_cm < -DROP_CENTER_TOL_CM) ||
        (vision->filtered_body_x_cm > DROP_CENTER_TOL_CM) ||
        (vision->filtered_body_y_cm < -DROP_CENTER_TOL_CM) ||
        (vision->filtered_body_y_cm > DROP_CENTER_TOL_CM))
    {
        drop_confirming = 0U;
        return;
    }

    if (drop_confirming == 0U)
    {
        drop_confirming = 1U;
        drop_confirm_start_tick = now_tick;
        return;
    }

    if ((uint32_t)(now_tick - drop_confirm_start_tick) >=
        DROP_CONFIRM_MS)
    {
        payload_released = 1U;
        drop_confirming = 0U;
        Decision_SetMagnet(0U);

        /* Leave car following and fly home at 125 cm before landing. */
        Decision_StartReturnHome();
    }
}

uint8_t Decision_GetTaskMode(void)
{
    return task_mode;
}

uint8_t Decision_GetRunId(void)
{
    return task_run_id;
}

uint8_t Decision_IsTaskActive(void)
{
    return task_active;
}

uint8_t Decision_IsMagnetOn(void)
{
    return magnet_on;
}

uint8_t Decision_IsPayloadReleased(void)
{
    return payload_released;
}

uint8_t Decision_IsReturning(void)
{
    return return_started;
}

int16_t Decision_GetCommandedHeightCm(void)
{
    if (task_active == 0U)
    {
        return 0;
    }

    /* Initial launch is exactly the same 125 cm / 6 s sequence as task 1. */
    if (Move_GetLaunchElapsedMs() < INITIAL_TAKEOFF_TOTAL_MS)
    {
        return DYNAMIC_CRUISE_HEIGHT_CM;
    }

    if ((task_mode == AIR_TASK_DYNAMIC_LANDING) &&
        (return_started == 0U))
    {
        return dynamic_target_height_cm;
    }

    return Location_GetCurrentWaypointZ();
}

uint8_t Decision_GetDynamicLandingState(void)
{
    return dynamic_state;
}

uint16_t Decision_GetDynamicStableTimeS(void)
{
    uint32_t elapsed_ms;

    if ((dynamic_state == AIR_DYNAMIC_RETAKEOFF) ||
        (dynamic_state == AIR_DYNAMIC_COMPLETE))
    {
        return 5U;
    }

    if (dynamic_state != AIR_DYNAMIC_WAIT_5S)
    {
        return 0U;
    }

    elapsed_ms = (uint32_t)(HAL_GetTick() - dynamic_state_start_tick);
    if (elapsed_ms >= DYNAMIC_PLATFORM_WAIT_MS)
    {
        return 5U;
    }

    return (uint16_t)(elapsed_ms / 1000U);
}
