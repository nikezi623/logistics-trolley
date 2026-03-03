#ifndef __SERVO_H
#define __SERVO_H

#include "stm32f10x.h"                  // Device header
#include "PWM.h"

void Servo_Init(void);
void Servo_dipan_360_Spd_control(uint16_t pwm);

#endif
