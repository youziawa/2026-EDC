#include "location.h"
#include "telecom.h"

target_location_t target_location = {0};

extern uint16_t cur_waypoint_index;// 来自move.c的当前航点索引

#define WAYPOINT_COUNT ((uint16_t)(sizeof(g_waypoints) / sizeof(g_waypoints[0]))) // 航点数量由g_waypoints数组的长度决定

// 手动航点列表: 按任务顺序填写每组位姿。
static target_location_t g_waypoints[] =
    {
        {0,   0, 125, 0},
        {0, -40, 125, 0},
        {85,-40, 125, 0},
        {200,-40, 125, 0},
};

// g_waypoints为所有航点清单

static uint16_t g_current_waypoint_index = 0; // 当前航点索引
static uint8_t g_mission_done_pending = 0;

void Location_InitWaypoints(void) // 初始化航点列表，并将目标位置设置为第一个航点。若无航点，则目标位置为0。
{
    if (WAYPOINT_COUNT == 0U)
    {
        target_location.x = 0;
        target_location.y = 0;
        target_location.z = 0;
        target_location.yaw = 0;
        g_current_waypoint_index = 0;
        g_mission_done_pending = 0;
        return;
    }

    g_current_waypoint_index = 0;
    g_mission_done_pending = 0;
    target_location = g_waypoints[g_current_waypoint_index];
}

int16_t Location_GetCurrentWaypointZ(void) // 获取当前航点的z值（高度）。
{
    if (WAYPOINT_COUNT == 0U) // 如果没有航点，默认高度为0
    {
        return 0;
    }
    return g_waypoints[g_current_waypoint_index].z; // 有航点时返回当前航点的z值
}

uint16_t Location_GetWaypointCount(void) // 返回航点总数
{
    return WAYPOINT_COUNT;
}

uint16_t Location_GetCurrentWaypointIndex(void) // 返回当前航点索引
{
    return g_current_waypoint_index;
}

/*将当前航点索引设置为指定值，并更新目标位置为对应航点。若索引无效，返回0；否则返回1。*/
uint8_t Location_SetCurrentWaypointIndex(uint16_t index)
{
    uint16_t count = WAYPOINT_COUNT; // 获取航点总数

    if (index >= count) // 索引必须小于航点总数，否则无效
    {
        return 0;
    }

    g_current_waypoint_index = index;                        // 更新当前航点索引为指定值
    g_mission_done_pending = 0;                              // 降落标识不待发送
    target_location = g_waypoints[g_current_waypoint_index]; // 更新目标位置为当前航点的位姿
    return 1;
}

uint8_t Location_AdvanceWaypoint(void) // 将当前航点索引前进到下一个航点，并更新目标位置。（暂未用到）
{
    uint16_t count = WAYPOINT_COUNT; // 获取航点总数
    uint16_t next_index;

    if (count == 0U) // 若总航点数为零，返回0
    {
        return 0U;
    }

    next_index = (uint16_t)(g_current_waypoint_index + 1U); // 计算下一个航点索引
    if (next_index >= count)                                // 若下一个航点索引超出索引范围，返回0
    {
        return 0U;
    }

    g_current_waypoint_index = next_index;                   // 更新当前航点索引为下一个航点
    g_mission_done_pending = 0U;                             // 降落标识不待发送
    target_location = g_waypoints[g_current_waypoint_index]; // 更新目标位置为下一个航点的位姿
    return 1U;
}

uint8_t Location_IsCurrentWaypointLast(void) // 判断当前航点是否为最后一个航点
{
    uint16_t count = WAYPOINT_COUNT;

    if (count == 0)
    {
        return 1;
    }

    return (uint8_t)((g_current_waypoint_index + 1U) >= count);
}

void Location_SetMissionDonePending(uint8_t pending) // 设置降落标识待发送状态，1表示待发送，0表示不待发送
{
    g_mission_done_pending = pending; // 该函数由Move模块在到达最后一个航点时调用，触发降落标识发送流程
}

uint8_t Location_GetMissionDonePending(void) // 获取降落标识待发送状态，1表示待发送，0表示不待发送
{
    return g_mission_done_pending; // 该函数由Transmit模块在Transmit_ProcessMissionDone中调用，检查是否需要发送降落标识
}

// 将摄像头编号写入当前航点
// uint8_t Location_AssignCameraNumberOnSwitch(uint16_t camera_number)
// {
//     if (cur_waypoint_index >= WAYPOINT_COUNT)
//     {
//         return 0U;
//     }

//     g_waypoints[cur_waypoint_index].camera_number = (int16_t)camera_number;

//     return 1U;
// }
