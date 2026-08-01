#ifndef __DECISION_H__
#define __DECISION_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

typedef enum
{
    AIR_TASK_NONE = 0U,
    AIR_TASK_DROP = 1U,
    AIR_TASK_DYNAMIC_LANDING = 2U
} AirTaskMode_t;

typedef enum
{
    AIR_DYNAMIC_IDLE = 0U,
    AIR_DYNAMIC_WAIT_LOCK = 1U,
    AIR_DYNAMIC_DESCENDING = 2U,
    AIR_DYNAMIC_FINAL_LAND = 3U,
    AIR_DYNAMIC_WAIT_5S = 4U,
    AIR_DYNAMIC_RETAKEOFF = 5U,
    AIR_DYNAMIC_COMPLETE = 6U,
    AIR_DYNAMIC_FAILED = 7U
} AirDynamicLandingState_t;

/* Kept for compatibility with the existing debug code. */
extern volatile int decision;

void Decision_Init(void);
void Decision_Update(void);

/* Called only after a valid LXS1 command from the car has been parsed. */
void Decision_OnTaskStart(uint8_t run_id, uint8_t task_mode);
void Decision_OnTaskAbort(uint8_t run_id);

/* Read-only diagnostics for Keil Watch/debug output. */
uint8_t Decision_GetTaskMode(void);
uint8_t Decision_GetRunId(void);
uint8_t Decision_IsTaskActive(void);
uint8_t Decision_IsMagnetOn(void);
uint8_t Decision_IsPayloadReleased(void);
uint8_t Decision_IsReturning(void);
int16_t Decision_GetCommandedHeightCm(void);
uint8_t Decision_GetDynamicLandingState(void);
uint16_t Decision_GetDynamicStableTimeS(void);

#ifdef __cplusplus
}
#endif

#endif /* __DECISION_H__ */
