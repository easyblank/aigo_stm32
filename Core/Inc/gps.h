#ifndef __GPS_H__
#define __GPS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "TinyGPS.h"
#include "usart.h"
#include "MPU9250.h"
#include "math.h"
#include "motor.h"
void getGPS(void);
void setWaypoint(void);
void clearWaypoint(void);
void getCompass(void);
void setHeading(void);
void gpsInfo(void);
void goWaypoint(void);
void Startup(void);

#ifdef __cplusplus
}
#endif

#endif /* __GPS_H__ */
