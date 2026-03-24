#include "PHC_HeadFile.h"

int main(void)
{
    uint16_t pulse; // 用于记录当前脉宽数值 (500~2500)

    Init_All();

    /* 2. 上电复位：让四个舵机先缓慢回到中位 (90度 -> 对应 1500us) */
    Servo_Control(1, 1500);
    Servo_Control(2, 1500);
    Servo_Control(3, 1500);
    Servo_Control(4, 1500);
    Delay_ms(1000); // 等待舵机走到位

    while (1)
    {
        /* 测试动作 1：所有舵机平滑转到 0 度 (对应 500us) */
        for (pulse = 1500; pulse >= 500; pulse -= 10)
        {
            Servo_Control(1, pulse);
            Servo_Control(2, pulse);
            Servo_Control(3, pulse);
            Servo_Control(4, pulse);
            Delay_ms(10); // 每次改变角度停顿 10ms，形成平滑动画
        }

        Delay_ms(500); // 在 0 度位置停留 0.5 秒

        /* 测试动作 2：所有舵机平滑转到 180 度 (对应 2500us) */
        for (pulse = 500; pulse <= 2500; pulse += 10)
        {
            Servo_Control(1, pulse);
            Servo_Control(2, pulse);
            Servo_Control(3, pulse);
            Servo_Control(4, pulse);
            Delay_ms(10);
        }

        Delay_ms(500); // 在 180 度位置停留 0.5 秒

        /* 测试动作 3：回到中位，准备下一次循环 */
        for (pulse = 2500; pulse >= 1500; pulse -= 10)
        {
            Servo_Control(1, pulse);
            Servo_Control(2, pulse);
            Servo_Control(3, pulse);
            Servo_Control(4, pulse);
            Delay_ms(10);
        }
    }
}
void TIM1_UP_IRQHandler(void) // 1ms进入一次
{
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
