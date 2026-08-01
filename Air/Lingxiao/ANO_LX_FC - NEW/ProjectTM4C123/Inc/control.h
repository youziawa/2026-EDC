#ifndef CONTROL_H
#define CONTROL_H
#include "Ano_DT_LX.h"

#define Safe_div(numerator, denominator, zero) ((denominator == 0) ? (zero) : ((numerator) / (denominator))) // 分母不为0
#define Limit(x, min, max) (((x) <= (min)) ? (min) : (((x) > (max)) ? (max) : (x)))                          // 限幅

typedef struct
{
    float Kp;          // 比例系数
    float Ki;          // 积分系数
    float Kd_expect;   // 微分系数
    float Kd_feedback; // 微分先行
    float K_ff;        // 前馈

    float err;
    float expect_old;
    float feedback_old;

    float expect_d;   // 目标微分
    float feedback_d; // 实际微分

    float err_i; // 积分
    float out;   // PID输出
} PID_DATA;

void PID_Calculate(float expect, float feedback, float feedforward, float dT, PID_DATA *PID, float ierr_Limit, float i_Limit);
void PID_Init(void);
u8 keep_high(u16 HIGH_input);

extern PID_DATA centre_xy[3];

void TMD_Val_Zero(void);
extern s16 TMD_Val[4];
void TMD_Set(s16 vel_x, s16 vel_y, s16 vel_zpcm, s16 yaw_dps);

#endif // CONTROL_H
