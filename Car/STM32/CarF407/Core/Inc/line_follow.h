#ifndef LINE_FOLLOW_H
#define LINE_FOLLOW_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef enum
{
  LINE_FOLLOW_STOPPED = 0U,
  LINE_FOLLOW_TRACKING = 1U,
  LINE_FOLLOW_LOST_HOLD = 2U,
  LINE_FOLLOW_REACQUIRE = 3U,
  LINE_FOLLOW_DISABLED = 4U
} LineFollowState;

void LineFollow_Init(void);
void LineFollow_Task(void);
void LineFollow_SetEnabled(uint8_t enabled);
void LineFollow_SetCruiseSpeedMmps(uint16_t speed_mm_s);
LineFollowState LineFollow_GetState(void);

#ifdef __cplusplus
}
#endif

#endif
