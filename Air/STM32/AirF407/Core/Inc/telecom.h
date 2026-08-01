#ifndef __TELECOM_H__
#define __TELECOM_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"

/*
 * USART6接收到的SLAM位姿数据。
 */
typedef struct
{
    /* X坐标。 */
    int16_t x;

    /* Y坐标。 */
    int16_t y;

    /* Z坐标。 */
    int16_t z;

    /* 航向角。 */
    int16_t yaw;

    /* 最近一帧完整SLAM位姿的接收时刻和有效标志。 */
    uint32_t rx_tick;
    uint8_t valid;
} PoseData_t;

/* K230 VISION_TARGET constants shared with move.c. */
#define VISION_TARGET_KIND_PLATFORM 2U
#define VISION_MODE_ACQUIRE         1
#define VISION_MODE_TRACK           2
#define VISION_MODE_LOST            3

/*
 * USART3 receives this 12-byte K230 VISION_TARGET payload:
 * valid, target_kind, confidence, dx_px, dy_px, radius_px,
 * vision_mode.
 */
typedef struct
{
    /* 0=no target/lost, 1=target valid. */
    uint8_t valid;

    /* Current K230 sends 2 for the car-top platform. */
    uint8_t target_kind;

    /* Detection confidence, 0..1000. */
    uint16_t confidence;

    /* Pixel offsets from image center: right/down are positive. */
    int16_t dx_px;
    int16_t dy_px;

    /* Detected outer-circle radius in pixels. */
    int16_t radius_px;

    /* 1=acquire, 2=track, 3=lost. */
    int16_t vision_mode;

    /* Local HAL tick when the newest frame was parsed. */
    uint32_t rx_tick;
} VisionTargetData_t;

/*
 * USART1从凌霄接收到的实际激光高度。
 */
typedef struct
{
    /* 实际激光高度，单位cm。 */
    int32_t height_cm;

    /* 最近一帧合法高度到达F407的时间。 */
    uint32_t rx_tick;

    /* 1=高度有效；0=无数据、异常或已经超时。 */
    uint8_t valid;
} FlightHeightData_t;

/* Latest 13-byte CAR_STATE snapshot received on the aircraft link. */
typedef struct
{
    uint8_t run_id;
    uint8_t state;
    uint8_t speed_mode;
    uint8_t line_valid;
    uint16_t speed_mm_s;
    uint16_t fault;
    uint32_t path_mm;
    uint8_t finish_marker;
    uint32_t rx_tick;
    uint8_t valid;
} CarStatusData_t;

/*
 * USART6接收到的SLAM位姿。
 * 实体定义位于telecom.c。
 */
extern PoseData_t pose_data;

/*
 * USART3接收到的完整视觉结果。
 * 实体定义位于telecom.c。
 */
extern VisionTargetData_t vision_target;

/*
 * USART1接收到的凌霄实际激光高度。
 * 实体定义位于telecom.c。
 */
extern FlightHeightData_t flight_height;

/* Latest decoded car status for USART2/XCOM diagnostics. */
extern CarStatusData_t car_status;

/*
 * 为了兼容transmit.c中的现有代码，
 * 继续保留board和camera_number。
 */
extern uint8_t board;
extern uint8_t camera_number;

/*
 * 初始化全部通信接口。
 */
void Telecom_Init(void);

/*
 * 启动USART1凌霄高度接收。
 */
void Telecom_USART1_Init(void);

/*
 * 判断实际高度是否有效。
 *
 * 返回：
 * 1 = 最近300ms内收到合法高度；
 * 0 = 从未收到、数据异常或者已经超时。
 */
uint8_t Telecom_IsFlightHeightValid(void);

/*
 * 安全取得当前实际高度。
 *
 * height_cm：返回实际激光高度，单位cm。
 * 返回1表示成功；返回0时禁止继续使用旧高度。
 */
uint8_t Telecom_GetFlightHeightCm(int32_t *height_cm);

/*
 * 初始化USART6的SLAM接收。
 */
void Telecom_USART6_Init(void);

/* SLAM位姿是否已收到且未超时。 */
uint8_t Telecom_IsPoseValid(void);

/*
 * 初始化USART2的地面站接收。
 */
void Telecom_USART2_Init(void);

/*
 * 初始化UART5的小车LXS1接收。
 */
void Telecom_UART5_Init(void);

/*
 * 初始化USART3的LXS1摄像头接收。
 */
void Telecom_USART3_Init(void);

/* Request the aircraft K230 to start platform detection. */
void Telecom_RequestVisionStart(void);

/* Notify the car that visual takeover succeeded and it may accelerate. */
void Telecom_RequestCarAccelerate(void);

/* Repeatedly notify the car that payload release is complete and return began. */
void Telecom_NotifyCarReturn(void);

/*
 * 处理USART2地面站、UART5小车和USART3摄像头接收缓冲区。
 *
 * 必须在main函数的while(1)中反复调用。
 */
void Telecom_Update(void);

/*
 * 判断当前视觉结果是否仍然有效。
 *
 * 返回：
 * 1 = 有效；
 * 0 = 无效或已经超过300ms。
 */
uint8_t Telecom_IsVisionValid(void);

/*
 * 判断天空K230通信链路是否在线。
 *
 * 1 = 最近2500ms内收到过合法心跳或视觉帧；
 * 0 = 从未收到或通信已经超时。
 */
uint8_t Telecom_IsK230Online(void);

/* Read-only USART3 diagnostics for the serial status output. */
uint32_t Telecom_GetK230RxByteCount(void);
uint32_t Telecom_GetK230ParsedFrameCount(void);
uint32_t Telecom_GetK230TargetFrameCount(void);
uint32_t Telecom_GetK230HeartbeatCount(void);
uint32_t Telecom_GetK230UartErrorCount(void);
uint32_t Telecom_GetK230BadPayloadCount(void);

/* Read-only UART5 car-task diagnostics. */
uint32_t Telecom_GetCarRxByteCount(void);
uint32_t Telecom_GetCarTaskStartCount(void);
uint32_t Telecom_GetCarBadTaskCount(void);
uint32_t Telecom_GetCarUartErrorCount(void);
uint8_t Telecom_GetCarAccelerationSendCount(void);
uint8_t Telecom_IsCarAccelerationAcknowledged(void);
/* True only after this Task-2 run receives the car's B-cross event. */
uint8_t Telecom_IsCarLandingPermitted(void);
uint8_t Telecom_IsCarStatusValid(void);
const CarStatusData_t *Telecom_GetCarStatus(void);

#ifdef __cplusplus
}
#endif

#endif /* __TELECOM_H__ */
