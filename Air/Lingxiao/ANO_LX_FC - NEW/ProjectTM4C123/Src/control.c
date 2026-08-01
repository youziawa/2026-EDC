#include "control.h"
#include "Drv_Uart.h"
#include "ANO_DT_LX.h"
#include "Drv_UbloxGPS.h"
#include "Drv_AnoOf.h"
#include "ANO_Math.h"

// 实时控制帧函数组
s16 TMD_Val[4];
extern s32 HIGH;

PID_DATA centre_xy[3]; // 结构体数组

// 定点PID初始化
void PID_Init(void)
{
    centre_xy[0].Kp = 0.37f;
    centre_xy[0].Ki = 0.07f;
    centre_xy[0].Kd_expect = 0.15f;
    centre_xy[0].Kd_feedback = 0.05f;
    centre_xy[0].K_ff = 0;
}

// PID计算函数
// P:比例
// I:积分
// D:微分
void PID_Calculate(float expect, float feedback, float feedforward, float dT, PID_DATA *PID, float ierr_Limit, float i_Limit)
{
    static float hz, derros;
    hz = Safe_div(1.0f, dT, 0);
    PID->feedback_old = feedback;
    PID->expect_old = expect;

    PID->err = expect - feedback; // 误差

    PID->expect_d = PID->Kd_expect * (expect - PID->expect_old) * hz; // 期望值微分

    PID->feedback_d = PID->Kd_feedback * (feedback - PID->feedback_old) * hz; // 实际值微分

    derros = PID->expect_d - PID->feedback_d; // 微分KD*

    PID->err_i += PID->Ki * Limit(PID->err, -ierr_Limit, ierr_Limit) * dT;
    PID->err_i = Limit(PID->err_i, -i_Limit, i_Limit);
    /*P                    I               D          前馈 */
    PID->out = PID->Kp * PID->err + PID->err_i + derros + PID->K_ff * feedforward;
}

u8 keep_high(u16 HIGH_input)
{
    static s16 H_Point = 0;
    static float H_cm = 0;
    static u8 centre_ok = 0;
    H_Point = HIGH_input - HIGH;      // 计算高度误差
    H_cm += 0.25f * (H_Point - H_cm); // 低通滤波

    if (ABS(H_Point) > 5)
    {
        centre_ok = 0;
        PID_Calculate(H_cm, 0, 0, 20 * 1e-3f, &centre_xy[0], 5, 5);
        centre_xy[0].out = LIMIT(-centre_xy[0].out, -8, 8); // 头向速度x
        //		  TMD_set(0,0,0,-(s16)centre_xy[0].out);
        TMD_Val[2] = -(s16)centre_xy[0].out;
        //		UARTprintf("上下速度:%d\r\n",-(s16)centre_xy[0].out);
        return 0;
    }
    else
    {
        if (centre_ok < 10U)
        {
            centre_ok++;
        }
        if (centre_ok == 10) // 当连续40个时序(40*0.02=0.8s)都对准时 飞机判定开始工作
        {
            // 实时帧清零
            //			TMD_set(0,0,0,0);
            TMD_Val[2] = 0;
            //			mission_step += 1;				//执行下一步操作
            centre_xy[0].err_i = 0; // 积分误差清零
            H_cm = 0;               // 坐标误差清零
                                    // UARTprintf("上下速度:0\r\n");
        }
        return (centre_ok >= 10) ? 1 : 0;
    }
}

void TMD_Set(s16 vel_x, s16 vel_y, s16 vel_zpcm, s16 yaw_dps)
{
    TMD_Val[0] = vel_x;    // 头向速度，厘米每秒
    TMD_Val[1] = vel_y;    // 左向速度，厘米每秒
    TMD_Val[2] = vel_zpcm; // 天向速度，厘米每秒
    TMD_Val[3] = yaw_dps;  // 航向转动角速度，度每秒，逆时针为正
}
///////////////////////////////////////////////////////////////////////
// 实时控制帧函数清零
void TMD_Val_Zero(void)
{
    TMD_Val[0] = 0;
    TMD_Val[1] = 0;
    TMD_Val[2] = 0;
    TMD_Val[3] = 0;
}
