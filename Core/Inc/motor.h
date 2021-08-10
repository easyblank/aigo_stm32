/*
 * motor.h
 *
 *  Created on: Aug 11, 2021
 *      Author: clapd
 */

#ifndef __MOTOR_H__
#define __MOTOR_H__
#include "tim.h"
#include "gpio.h"
void set_motor(uint8_t, uint8_t, uint32_t);
void mvForward();
void mvBackward();
void mvLeft();
void mvRight();
void Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H__ */
