#include "PHC_HeadFile.h"

uint8_t flag;

int main(void)
{
    uint16_t pulse; // 用于记录当前脉宽数值 (500~2500)

    Init_All();

    Trigger_Top_Init();

    /* 2. 上电复位：让四个舵机先缓慢回到中位 (90度 -> 对应 1500us) */ //20000
    Servo_Control(1, 2600); //底盘初始位置
    Servo_Control(2, 2000); //短臂初始位置
    Servo_Control(3, 500); //长臂初始位置
    Servo_Control(4, 1500); //夹爪初始位置，1800的夹取是合适的
    Delay_ms(1000); // 等待舵机走到位

    // Trigger_Set_Low(GPIOA, GPIO_Pin_12);
    while (1)
    {
        if (Trigger_Read_Pin(GPIOA, GPIO_Pin_12) == 1 && flag == 0)
        {
            flag++;

            // 抓取结束，通知下层
            Trigger_Set_High(GPIOA, GPIO_Pin_15);
        }
    }
}
void TIM1_UP_IRQHandler(void) // 1ms进入一次
{
    // if (flag == 0)
    // {
    //     Trigger_Set_Low(GPIOA, GPIO_Pin_15);
    // }

    TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
}

/*

状态描述,			涉及的传感器位 (0为触发),	二进制 (Bin),	十六进制 (Hex),	发送指令
丢线,				无,							1111 1111,		0xFF,			2
居中,				"3, 4",						1110 0111,		0xE7,			3
居中 (容错),		4 (左中单灯),				1110 1111,		0xEF,			3
居中 (容错),		3 (右中单灯),				1111 0111,		0xF7,			3
轻微偏右,			"2, 3",						1111 0011,		0xF3,			4
轻微偏右 (容错),	2 (单灯),					1111 1011,		0xFB,			4
严重偏右,			1,							1111 1101,		0xFD,			5
严重偏右 (容错),	0 (最外侧单灯),				1111 1110,		0xFE,			5
轻微偏左,			"4, 5",						1100 1111,		0xCF,			6
轻微偏左 (容错),	5 (单灯),					1101 1111,		0xDF,			6
严重偏左,			6,							1011 1111,		0xBF,			7
严重偏左 (容错),	7 (最外侧单灯),				0111 1111,		0x7F,			7

*/
