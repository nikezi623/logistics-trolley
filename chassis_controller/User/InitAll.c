#include "PHC_HeadFile.h"

void Init_All(void)
{
    OLED_Init();
	MPU6050_Init();
	Timer_Init();
	BlueSerial_Init();
	LED_Init();
	Encoder_Init();
	Serial_Init();
	Key_Init();
	Motor_Init();
}
