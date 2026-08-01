#include "Drv_Uart.h"
#include "uart.h"
#include "hw_ints.h"
#include "hw_gpio.h"
#include "hw_types.h"
#include "Ano_DT_LX.h"
#include "Drv_UbloxGPS.h"
#include "Drv_AnoOf.h"
#include "uartstdio.h"

// TM4C的串口0对应底板串口1
// TM4C的串口2对应底板串口5
// TM4C的串口4对应底板串口2
// TM4C的串口5对应底板串口3
// TM4C的串口7对应底板串口4
// 这里都以底板的串口标号为准，比如Drv_Uart1Init，为底板串口1的初始化，即TM4C的串口0
/////////////////////////////////////////////////////////////////////////////////////////////////
// 串口接收发送快速定义，直接修改此处的函数名称宏，修改成自己的串口解析和发送函数名称即可，注意函数参数格式需统一
void NoUse(u8 data) {}
static void U2PoseDataReceive(u8 data);
#define U1GetOneByte UBLOX_M8_GPS_Data_Receive // 接收串口1数据的函数，参数为接收到的一个字节数据
#define U2GetOneByte U2PoseDataReceive		   // 接收串口2数据的函数，参数为接收到的一个字节数据
#define U3GetOneByte NoUse
#define U4GetOneByte AnoOF_GetOneByte
#define U5GetOneByte ANO_DT_LX_Data_Receive_Prepare
/////////////////////////////////////////////////////////////////////////////////////////////////
u8 U1TxDataTemp[256];		 // 串口1发送数据的临时缓冲区
u8 U1TxInCnt = 0;			 // 串口1发送数据的输入计数
u8 U1TxOutCnt = 0;			 // 串口1的中断服务函数和发送检查函数声明
void UART1_IRQHandler(void); // 串口1的中断服务函数声明
void DrvUart1TxCheck(void);	 // 串口1发送检查函数声明
void DrvUart1Init(uint32_t baudrate)
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0); // 外设使能
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	/*GPIO的UART模式配置*/
	ROM_GPIOPinConfigure(UART0_RX); // 复用使能;
	ROM_GPIOPinConfigure(UART0_TX);
	ROM_GPIOPinTypeUART(UART0_PORT, UART0_PIN_TX | UART0_PIN_RX); // 引脚配置为串口
	/*配置串口号波特率和时钟源*/
	ROM_UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(), baudrate, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
	UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
	//	SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |SYSCTL_XTAL_16MHZ);
	UARTStdioConfig(0, baudrate, 16000000);

	/*FIFO设置*/
	ROM_UARTFIFOLevelSet(UART0_BASE, UART_FIFO_TX7_8, UART_FIFO_RX7_8); // FIFO字节的设置
	ROM_UARTFIFOEnable(UART0_BASE);										// FIFO使能
	/*使能串口*/
	ROM_UARTEnable(UART0_BASE); // 串口使能
	/*使能UART0接收中断*/
	UARTIntRegister(UART0_BASE, UART1_IRQHandler);							// 注册中断函数
	ROM_IntPrioritySet(INT_UART0, USER_INT2);								// 中断优先级设置
	ROM_UARTTxIntModeSet(UART0_BASE, UART_TXINT_MODE_EOT);					// 串口中断模式设置
	ROM_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT | UART_INT_TX); // 串口中断使能;
}
void DrvUart1SendBuf(u8 *data, u8 len) // 串口1发送数据函数，参数为数据缓冲区指针和数据长度
{
	for (u8 i = 0; i < len; i++)
	{
		U1TxDataTemp[U1TxInCnt++] = *(data + i);
	}
	DrvUart1TxCheck();
}
void DrvUart1TxCheck(void)
{
	while ((U1TxOutCnt != U1TxInCnt) && (ROM_UARTCharPutNonBlocking(UART0_BASE, U1TxDataTemp[U1TxOutCnt])))
		U1TxOutCnt++;
}
void UART1_IRQHandler(void)
{
	uint8_t com_data;
	/*获取中断标志 原始中断状态 不屏蔽中断标志*/
	uint32_t flag = ROM_UARTIntStatus(UART0_BASE, 1);
	/*清除中断标志*/
	ROM_UARTIntClear(UART0_BASE, flag);
	/*判断FIFO是否还有数据*/
	while (ROM_UARTCharsAvail(UART0_BASE))
	{
		com_data = ROM_UARTCharGet(UART0_BASE);
		U1GetOneByte(com_data);
	}
	if (flag & UART_INT_TX)
	{
		DrvUart1TxCheck();
	}
}
/////////////////////////////////////////////////////////////////////////////////////////////////
u8 U2TxDataTemp[256];
u8 U2TxInCnt = 0;
u8 U2TxOutCnt = 0;
static volatile s16 U2PoseX = 0;
static volatile s16 U2PoseY = 0;
static volatile s16 U2PoseZ = 0;
static volatile s16 U2PoseYaw = 0;
// 位姿数据最新值（U2路径：TM4C硬件UART4/底板串口2）
static volatile u8 U2PoseUpdated = 0; // 收到一帧新位姿后置1（被ReadPose读取后清0）
// 降落标识事件与位姿共用U2路径（TM4C硬件UART4/底板串口2）
static volatile u8 U5MissionDoneUpdated = 0; // 收到FD 01 0D后置1（被ReadMissionDone读取后清0）

#define U2_POSE_FRAME_HEAD 0xFB
#define U2_POSE_FRAME_TAIL 0x0D
#define U2_POSE_FRAME_LEN 10
#define MISSION_DONE_HEAD 0xFD
#define MISSION_DONE_FLAG 0x01
#define MISSION_DONE_TAIL 0x0D
#define MISSION_DONE_FRAME_LEN 3

// U2复用接收状态机：位姿FB帧与降落FD帧共用同一条凌霄链路。
static void U2PoseDataReceive(u8 data)
{
	static u8 rx_buf[U2_POSE_FRAME_LEN];
	static u8 rx_state = 0; // 当前已收了多少字节
	static u8 frame_len = 0;

	if (rx_state == 0)
	{
		// 空闲态按帧头区分10字节位姿帧和3字节降落帧。
		if (data == U2_POSE_FRAME_HEAD)
		{
			frame_len = U2_POSE_FRAME_LEN;
			rx_buf[rx_state++] = data;
		}
		else if (data == MISSION_DONE_HEAD)
		{
			frame_len = MISSION_DONE_FRAME_LEN;
			rx_buf[rx_state++] = data;
		}
	}
	else
	{
		rx_buf[rx_state++] = data;
		// 收满一帧后开始判断并解析
		if (rx_state >= frame_len)
		{
			if ((frame_len == U2_POSE_FRAME_LEN) && (rx_buf[U2_POSE_FRAME_LEN - 1] == U2_POSE_FRAME_TAIL))
			{
				// 位姿帧格式：FB + xH xL + yH yL + zH zL + yawH yawL + 0D
				// 注意：这里按高字节在前进行拼接
				U2PoseX = (s16)(((u16)rx_buf[1] << 8) | rx_buf[2]);
				U2PoseY = (s16)(((u16)rx_buf[3] << 8) | rx_buf[4]);
				U2PoseZ = (s16)(((u16)rx_buf[5] << 8) | rx_buf[6]);
				U2PoseYaw = (s16)(((u16)rx_buf[7] << 8) | rx_buf[8]);
				U2PoseUpdated = 1;
			}
			else if ((frame_len == MISSION_DONE_FRAME_LEN) &&
					 (rx_buf[0] == MISSION_DONE_HEAD) &&
					 (rx_buf[1] == MISSION_DONE_FLAG) &&
					 (rx_buf[2] == MISSION_DONE_TAIL))
			{
				U5MissionDoneUpdated = 1;
			}

			// 重新同步：
			// 如果当前字节又是合法帧头，直接作为下一帧起点。
			// 否则回到空闲态等待新帧头
			if (data == U2_POSE_FRAME_HEAD)
			{
				rx_state = 1;
				frame_len = U2_POSE_FRAME_LEN;
				rx_buf[0] = U2_POSE_FRAME_HEAD;
			}
			else if (data == MISSION_DONE_HEAD)
			{
				rx_state = 1;
				frame_len = MISSION_DONE_FRAME_LEN;
				rx_buf[0] = MISSION_DONE_HEAD;
			}
			else
			{
				rx_state = 0;
				frame_len = 0;
			}
		}
	}
}

void UART2_IRQHandler(void);
void DrvUart2TxCheck(void);
static void DrvUart2RxPollCheck(void);
void DrvUart2Init(uint32_t baudrate)
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART4);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

	/*GPIO的UART模式配置*/
	ROM_GPIOPinConfigure(UART4_RX);
	ROM_GPIOPinConfigure(UART4_TX);
	ROM_GPIOPinTypeUART(UART4_PORT, UART4_PIN_TX | UART4_PIN_RX);
	/*配置串口号波特率和时钟源*/
	ROM_UARTConfigSetExpClk(UART4_BASE, ROM_SysCtlClockGet(), baudrate, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
	/*FIFO设置*/
	ROM_UARTFIFOLevelSet(UART4_BASE, UART_FIFO_TX7_8, UART_FIFO_RX7_8);
	ROM_UARTFIFOEnable(UART4_BASE);
	/*使能串口*/
	ROM_UARTEnable(UART4_BASE);
	/*使能UART0接收中断*/
	UARTIntRegister(UART4_BASE, UART2_IRQHandler);
	ROM_IntPrioritySet(INT_UART4, USER_INT2);
	ROM_UARTTxIntModeSet(UART4_BASE, UART_TXINT_MODE_EOT);
	ROM_UARTIntEnable(UART4_BASE, UART_INT_RX | UART_INT_RT | UART_INT_TX);
}
void DrvUart2SendBuf(u8 *data, u8 len)
{
	for (u8 i = 0; i < len; i++)
	{
		U2TxDataTemp[U2TxInCnt++] = *(data + i);
	}
	DrvUart2TxCheck();
}
void DrvUart2TxCheck(void)
{
	while ((U2TxOutCnt != U2TxInCnt) && (ROM_UARTCharPutNonBlocking(UART4_BASE, U2TxDataTemp[U2TxOutCnt])))
		U2TxOutCnt++;
}

static void DrvUart2RxPollCheck(void)
{
	u8 com_data;
	// 兜底轮询：即使UART4中断没有进来，也能把FIFO里的字节取走并做协议解析。
	while (ROM_UARTCharsAvail(UART4_BASE))
	{
		com_data = ROM_UARTCharGet(UART4_BASE);
		U2GetOneByte(com_data);
	}
}

u8 DrvUart2ReadPose(s16 *x, s16 *y, s16 *z, s16 *yaw) // 读取串口2接收到的位姿数据，返回是否有新数据
{
	// 事件型读取：读到新帧后会清标志，避免重复消费同一帧
	u8 has_new = U2PoseUpdated; // 读取是否有新数据的标志

	if (has_new) // 如果有
	{
		if (x)			  // 如果不为空指针
			*x = U2PoseX; // 将接收到的位姿数据赋值给传入的指针参数
		if (y)
			*y = U2PoseY;
		if (z)
			*z = U2PoseZ;
		if (yaw)
			*yaw = U2PoseYaw;

		U2PoseUpdated = 0; // 读取完毕后清除新数据标志，等待下一次接收新的位姿数据
	}

	return has_new; // 返回是否有新数据的标志
}
/////////////////////
///////////////////////////////////////////

void UART2_IRQHandler(void)
{
	uint8_t com_data;
	/*获取中断标志 原始中断状态 不屏蔽中断标志*/
	uint32_t flag = ROM_UARTIntStatus(UART4_BASE, 1);
	/*清除中断标志*/
	ROM_UARTIntClear(UART4_BASE, flag);
	/*判断FIFO是否还有数据*/
	while (ROM_UARTCharsAvail(UART4_BASE)) // 循环读取串口接收缓冲区中的数据，直到没有数据可读
	{
		com_data = ROM_UARTCharGet(UART4_BASE); // 从串口接收缓冲区中读取一个字节的数据，并存储在com_data变量中
		U2GetOneByte(com_data);					// 调用接收数据处理函数，传入读取到的字节数据
	}
	if (flag & UART_INT_TX)
	{
		DrvUart2TxCheck();
	}
}
/////////////////////////////////////////////////////////////////////////////////////////////////
u8 U3TxDataTemp[256];
u8 U3TxInCnt = 0;
u8 U3TxOutCnt = 0;
void UART3_IRQHandler(void);
void DrvUart3TxCheck(void);
void DrvUart3Init(uint32_t baudrate)
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART5);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

	/*GPIO的UART模式配置*/
	ROM_GPIOPinConfigure(UART5_RX);
	ROM_GPIOPinConfigure(UART5_TX);
	ROM_GPIOPinTypeUART(UART5_PORT, UART5_PIN_TX | UART5_PIN_RX);
	/*配置串口号波特率和时钟源*/
	ROM_UARTConfigSetExpClk(UART5_BASE, ROM_SysCtlClockGet(), baudrate, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
	/*FIFO设置*/
	ROM_UARTFIFOLevelSet(UART5_BASE, UART_FIFO_TX7_8, UART_FIFO_RX7_8);
	ROM_UARTFIFOEnable(UART5_BASE);
	/*使能串口*/
	ROM_UARTEnable(UART5_BASE);
	/*使能UART0接收中断*/
	UARTIntRegister(UART5_BASE, UART3_IRQHandler);
	ROM_IntPrioritySet(INT_UART5, USER_INT2);
	ROM_UARTTxIntModeSet(UART5_BASE, UART_TXINT_MODE_EOT);
	ROM_UARTIntEnable(UART5_BASE, UART_INT_RX | UART_INT_RT | UART_INT_TX);
}
void DrvUart3SendBuf(u8 *data, u8 len)
{
	for (u8 i = 0; i < len; i++)
	{
		U3TxDataTemp[U3TxInCnt++] = *(data + i);
	}
	DrvUart3TxCheck();
}
void DrvUart3TxCheck(void)
{
	while ((U3TxOutCnt != U3TxInCnt) && (ROM_UARTCharPutNonBlocking(UART5_BASE, U3TxDataTemp[U3TxOutCnt])))
		U3TxOutCnt++;
}
void UART3_IRQHandler(void)
{
	uint8_t com_data;
	/*获取中断标志 原始中断状态 不屏蔽中断标志*/
	uint32_t flag = ROM_UARTIntStatus(UART5_BASE, 1);
	/*清除中断标志*/
	ROM_UARTIntClear(UART5_BASE, flag);
	/*判断FIFO是否还有数据*/
	while (ROM_UARTCharsAvail(UART5_BASE))
	{
		com_data = ROM_UARTCharGet(UART5_BASE);
		U3GetOneByte(com_data);
	}
	if (flag & UART_INT_TX)
	{
		DrvUart3TxCheck();
	}
}

u8 DrvUart5ReadMissionDone(void)
{
	// 保留旧接口名；事件现在由U2位姿/降落复用解析器产生。
	u8 has_new = U5MissionDoneUpdated;
	if (has_new)
	{
		U5MissionDoneUpdated = 0;
	}

	return has_new;
}

/////////////////////////////////////////////////////////////////////////////////////////////////
u8 U4TxDataTemp[256];
u8 U4TxInCnt = 0;
u8 U4TxOutCnt = 0;
void UART4_IRQHandler(void);
void DrvUart4TxCheck(void);
void DrvUart4Init(uint32_t baudrate)
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

	/*GPIO的UART模式配置*/
	ROM_GPIOPinConfigure(UART7_RX);
	ROM_GPIOPinConfigure(UART7_TX);
	ROM_GPIOPinTypeUART(UART7_PORT, UART7_PIN_TX | UART7_PIN_RX);
	/*配置串口号波特率和时钟源*/
	ROM_UARTConfigSetExpClk(UART7_BASE, ROM_SysCtlClockGet(), baudrate, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
	/*FIFO设置*/
	ROM_UARTFIFOLevelSet(UART7_BASE, UART_FIFO_TX7_8, UART_FIFO_RX7_8);
	ROM_UARTFIFOEnable(UART7_BASE);
	/*使能串口*/
	ROM_UARTEnable(UART7_BASE);
	/*使能UART0接收中断*/
	UARTIntRegister(UART7_BASE, UART4_IRQHandler);
	ROM_IntPrioritySet(INT_UART7, USER_INT2);
	ROM_UARTTxIntModeSet(UART7_BASE, UART_TXINT_MODE_EOT);
	ROM_UARTIntEnable(UART7_BASE, UART_INT_RX | UART_INT_RT | UART_INT_TX);
}
void DrvUart4SendBuf(u8 *data, u8 len)
{
	for (u8 i = 0; i < len; i++)
	{
		U4TxDataTemp[U4TxInCnt++] = *(data + i);
	}
	DrvUart4TxCheck();
}
void DrvUart4TxCheck(void)
{
	while ((U4TxOutCnt != U4TxInCnt) && (ROM_UARTCharPutNonBlocking(UART7_BASE, U4TxDataTemp[U4TxOutCnt])))
		U4TxOutCnt++;
}
void UART4_IRQHandler(void)
{
	uint8_t com_data;
	/*获取中断标志 原始中断状态 不屏蔽中断标志*/
	uint32_t flag = ROM_UARTIntStatus(UART7_BASE, 1);
	/*清除中断标志*/
	ROM_UARTIntClear(UART7_BASE, flag);
	/*判断FIFO是否还有数据*/
	while (ROM_UARTCharsAvail(UART7_BASE))
	{
		com_data = ROM_UARTCharGet(UART7_BASE);
		U4GetOneByte(com_data);
	}
	if (flag & UART_INT_TX)
	{
		DrvUart4TxCheck();
	}
}
/////////////////////////////////////////////////////////////////////////////////////////////////
u8 U5TxDataTemp[256];
u8 U5TxInCnt = 0;
u8 U5TxOutCnt = 0;
void UART5_IRQHandler(void);
void DrvUart5TxCheck(void);
void DrvUart5Init(uint32_t baudrate)
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

	/*PD7解锁操作*/
	HWREG(UART2_PORT + GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(UART2_PORT + GPIO_O_CR) = UART2_PIN_TX;
	HWREG(UART2_PORT + GPIO_O_LOCK) = 0x00;
	/*GPIO的UART模式配置*/
	ROM_GPIOPinConfigure(UART2_RX);
	ROM_GPIOPinConfigure(UART2_TX);
	ROM_GPIOPinTypeUART(UART2_PORT, UART2_PIN_TX | UART2_PIN_RX);
	/*配置串口号波特率和时钟源*/
	ROM_UARTConfigSetExpClk(UART2_BASE, ROM_SysCtlClockGet(), baudrate, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
	/*FIFO设置*/
	ROM_UARTFIFOLevelSet(UART2_BASE, UART_FIFO_TX7_8, UART_FIFO_RX7_8);
	ROM_UARTFIFOEnable(UART2_BASE);
	/*使能串口*/
	ROM_UARTEnable(UART2_BASE);
	/*使能UART0接收中断*/
	UARTIntRegister(UART2_BASE, UART5_IRQHandler);
	ROM_IntPrioritySet(INT_UART2, USER_INT2);
	ROM_UARTTxIntModeSet(UART2_BASE, UART_TXINT_MODE_EOT);
	ROM_UARTIntEnable(UART2_BASE, UART_INT_RX | UART_INT_RT | UART_INT_TX);
}
void DrvUart5SendBuf(u8 *data, u8 len)
{
	for (u8 i = 0; i < len; i++)
	{
		U5TxDataTemp[U5TxInCnt++] = *(data + i);
	}
	DrvUart5TxCheck();
}
void DrvUart5TxCheck(void)
{
	while ((U5TxOutCnt != U5TxInCnt) && (ROM_UARTCharPutNonBlocking(UART2_BASE, U5TxDataTemp[U5TxOutCnt])))
		U5TxOutCnt++;
}
void UART5_IRQHandler(void)
{
	uint8_t com_data;
	/*获取中断标志 原始中断状态 不屏蔽中断标志*/
	uint32_t flag = ROM_UARTIntStatus(UART2_BASE, 1);
	/*清除中断标志*/
	ROM_UARTIntClear(UART2_BASE, flag);
	/*判断FIFO是否还有数据*/
	while (ROM_UARTCharsAvail(UART2_BASE))
	{
		com_data = ROM_UARTCharGet(UART2_BASE);
		U5GetOneByte(com_data);
	}
	if (flag & UART_INT_TX)
	{
		DrvUart5TxCheck();
	}
}

void DrvUartDataCheck(void)
{
	// TM4C有串口FIFO，就不写人工缓冲区了，仅判断是否发送完毕
	DrvUart2RxPollCheck();
	DrvUart1TxCheck();
	DrvUart2TxCheck();
	DrvUart3TxCheck();
	DrvUart4TxCheck();
	DrvUart5TxCheck();
}
