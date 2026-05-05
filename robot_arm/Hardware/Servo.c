#include "Servo.h"

void Servo_Init(void)
{
	PWM_Init();
}

void Servo_Control(uint8_t servo_num, uint16_t PWM)
{
	if (servo_num == 1)
	{
		PWM_SetCompare1(PWM);
	}
	else if(servo_num == 2)
	{
		PWM_SetCompare2(PWM);
	}
	else if(servo_num == 3)
	{
		PWM_SetCompare3(PWM);
	}
	else if(servo_num == 4)
	{
		PWM_SetCompare4(PWM);
	}
}
