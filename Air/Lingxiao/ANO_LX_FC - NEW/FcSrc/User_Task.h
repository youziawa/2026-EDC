#ifndef __USER_TASK_H
#define __USER_TASK_H
#include "control.h"

#include "SysConfig.h"

void UserTask_OneKeyCmd(void);
void UserTask_UpdateExtPose(void);
void TMD_Set(s16 vel_x, s16 vel_y, s16 vel_zpcm, s16 yaw_dps);

#endif
