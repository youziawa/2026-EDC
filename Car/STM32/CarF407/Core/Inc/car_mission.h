#ifndef CAR_MISSION_H
#define CAR_MISSION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef enum
{
  CAR_STATE_READY = 0U,
  CAR_STATE_STARTING = 1U,
  CAR_STATE_NORMAL_TRACK = 2U,
  CAR_STATE_ACTION_SLOW = 3U,
  CAR_STATE_LOST_HOLD = 4U,
  CAR_STATE_REACQUIRE = 5U,
  CAR_STATE_FINISH_CONFIRM = 6U,
  CAR_STATE_DONE = 7U,
  CAR_STATE_ABORT = 8U,
  CAR_STATE_FAULT = 9U
} CarMissionState;

typedef enum
{
  CAR_TASK_DROP = 1U,
  CAR_TASK_DYNAMIC_LANDING = 2U
} CarMissionTaskMode;

void CarMission_Init(void);
void CarMission_Task(void);
void CarMission_RequestStart(void);
void CarMission_RequestReset(void);
CarMissionState CarMission_GetState(void);
CarMissionTaskMode CarMission_GetTaskMode(void);

/* Override in a board file after assigning and debouncing a real GPIO. */
uint8_t CarMission_ReadStartButton(void);
uint8_t CarMission_ReadStartButtonRaw(void);
uint8_t CarMission_ReadDropButton(void);
uint8_t CarMission_ReadLandButton(void);
uint8_t CarMission_ReadResetButton(void);

#ifdef __cplusplus
}
#endif

#endif
