#include "Servo.h"

void Servo_Init(void)
{
    PWM_Init();
}

void Servo_dipan_360_Spd_control(uint16_t pwm)
{
    PWM_SetCompare3(pwm);
}
