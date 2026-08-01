#ifndef __LOCATION_H__
#define __LOCATION_H__

#include <stdint.h>

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
    int16_t yaw;
    // uint8_t board1;        // 代表当前航点所在棋盘面，A/B/C/D
    // uint8_t board2;        // 代表当前航点所在棋盘格，1~6
    // int16_t camera_number; // 代表当前航点对应的摄像头编号
} target_location_t;

extern target_location_t target_location;

int16_t Location_GetCurrentWaypointZ(void);
void Location_InitWaypoints(void);
uint16_t Location_GetWaypointCount(void);
uint16_t Location_GetCurrentWaypointIndex(void);
uint8_t Location_SetCurrentWaypointIndex(uint16_t index);
uint8_t Location_AdvanceWaypoint(void);
uint8_t Location_IsCurrentWaypointLast(void);
void Location_StartReturnHome(void);
uint8_t Location_IsReturnHomeActive(void);
void Location_SetMissionDonePending(uint8_t pending);
uint8_t Location_GetMissionDonePending(void);
uint8_t Location_AssignCameraNumberOnSwitch(uint16_t camera_number);

#endif /* __LOCATION_H__ */
