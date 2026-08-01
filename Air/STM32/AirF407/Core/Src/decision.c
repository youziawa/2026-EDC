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

static void Decision_SetMagnet(uint8_t enable)
{
    magnet_on = (enable != 0U) ? 1U : 0U;

    HAL_GPIO_WritePin(MAGNET_GPIO_PORT,
                      MAGNET_GPIO_PIN,
                      (magnet_on != 0U) ?
                          GPIO_PIN_SET : GPIO_PIN_RESET);
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
        Location_StartReturnHome();
        Move_VisionFollowStop();
        return_started = 1U;
        Telecom_NotifyCarReturn();
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
