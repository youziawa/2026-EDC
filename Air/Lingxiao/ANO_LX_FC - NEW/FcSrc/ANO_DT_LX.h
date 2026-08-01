#ifndef __ANO_DT_LX_H
#define __ANO_DT_LX_H
//==����
#include "SysConfig.h"

//==����/����
#define FUN_NUM_LEN 256

extern s32 HIGH; // 高度数据

typedef struct
{
	u8 D_Addr;		 //Ŀ���ַ
	u8 WTS;			 //wait to send�ȴ����ͱ��
	u16 fre_ms;		 //��������
	u16 time_cnt_ms; //��ʱ����
} _dt_frame_st;

//cmd
typedef struct
{
	u8 CID;
	u8 CMD[10];
} _cmd_st;

//check
typedef struct
{
	u8 ID;
	u8 SC;
	u8 AC;
} _ck_st;

//param
typedef struct
{
	u16 par_id;
	s32 par_val;
} _par_st;

typedef struct
{
	_dt_frame_st fun[FUN_NUM_LEN];
	//
	u8 wait_ck;
	//
	_cmd_st cmd_send;
	_ck_st ck_send;
	_ck_st ck_back;
	_par_st par_data;
} _dt_st;

//==��������
extern _dt_st dt;
//==��������
//static
static void ANO_DT_LX_Send_Data(u8 *dataToSend, u8 length);
static void ANO_DT_LX_Data_Receive_Anl(u8 *data, u8 len);

//public
//
void ANO_DT_Init(void);
void ANO_LX_Data_Exchange_Task(float dT_s);
void ANO_DT_LX_Data_Receive_Prepare(u8 data);
//
void CMD_Send(u8 dest_addr, _cmd_st *cmd);
void CK_Back(u8 dest_addr, _ck_st *ck);
void PAR_Back(u8 dest_addr, _par_st *par);
#endif
