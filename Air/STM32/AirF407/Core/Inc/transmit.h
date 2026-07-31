#ifndef __TRANSMIT_H__
#define __TRANSMIT_H__

#include <stdint.h>

void Transmit_Init(void);
void Transmit_Update(void);
void Transmit_UpdateUsart2(void);
void Transmit_SendPose(void);
uint8_t Transmit_SendMissionDone(void);
void Transmit_ProcessMissionDone(void);
uint8_t Transmit_SendCustomUsart2(void);

extern uint8_t board1; // 占位值，代表棋盘面ABCD
extern uint8_t board2; // 占位值，代表棋盘格

#endif // __TRANSMIT_H__
