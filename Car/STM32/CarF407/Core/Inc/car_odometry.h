#ifndef CAR_ODOMETRY_H
#define CAR_ODOMETRY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef struct
{
  int16_t x_mm;
  int16_t y_mm;
  int16_t heading_deg;
  uint32_t path_mm;
  int16_t speed_mm_s;
} CarOdometryData;

void CarOdometry_Init(void);
void CarOdometry_Reset(void);
void CarOdometry_Task(void);
void CarOdometry_Get(CarOdometryData *data);

#ifdef __cplusplus
}
#endif

#endif
