#include "PHC_HeadFile.h"
#include "PWM.h"

uint8_t KeyNum, RunFlag;//runflag表示运行启停
uint16_t time_count;
int i;

int main(void)
{
    Init_All();
    uint8_t SensorStatus = 0; // 用于存储当前传感器状态
//	Servo_dipan_360_Spd_control(50);
    while (1)
    {	
        // 0.舵机测试

        
        // 1. 获取一次当前的传感器状态，并在本轮循环中固定使用这个值
        SensorStatus = GLE_GetStatus(); 

        // 按键逻辑
        KeyNum = Key_GetNum();
        if (KeyNum == 1)
        {
            Serial_SendByte(1);
        }

        // OLED显示 (显示当前的 HEX 状态，方便调试)
        OLED_ShowBinNum(0, 0, SensorStatus, 8, OLED_8X16); 
        OLED_ShowHexNum(0, 16, SensorStatus, 2, OLED_8X16); 
        OLED_ShowNum(0, 32, KeyNum, 1, OLED_8X16);
        OLED_Update();

        /* --- 状态判断逻辑 ---
           传感器逻辑假设：1为黑线(灯灭)，0为白地(灯亮)
           二进制位分布：[Bit7 Bit6 Bit5 Bit4 Bit3 Bit2 Bit1 Bit0]
                         左 <-------------------------> 右
        */

        // --- 特殊路况：直角/大弯 (优先级最高，先判断) ---
        
        // 左直角/左弯 (左边大片感应): 000x xxxx
        // 0x07(0000 0111), 0x0F(0000 1111), 0x1F(0001 1111), 0x3F(0011 1111)
        if (SensorStatus == 0x07 || SensorStatus == 0x0F || SensorStatus == 0x1F || SensorStatus == 0x3F)
        {
             Serial_SendByte(9); // 左转指令
        }
        // 右直角/右弯 (右边大片感应): xxxx x000
        // 0xE0(1110 0000), 0xF0(1111 0000), 0xF8(1111 1000), 0xFC(1111 1100)
        else if (SensorStatus == 0xE0 || SensorStatus == 0xF0 || SensorStatus == 0xF8 || SensorStatus == 0xFC)
        {
             Serial_SendByte(8); // 右转指令
        }
        
        // --- 正常巡线逻辑 ---

        // 完美居中 (3)
        // 0xE7(1110 0111): 中间两颗
        // 0xEF(1110 1111): 仅左中一颗
        // 0xF7(1111 0111): 仅右中一颗
        else if (SensorStatus == 0xE7 || SensorStatus == 0xEF || SensorStatus == 0xF7) 
        {
            Serial_SendByte(3);
        }
        // 轻微偏右 (4) -> 车身偏右，线在车身左侧 (Wait, if sensor 2/3 is active (Right side sensors), line is to the Right)
        // 你的逻辑: 0xF3(1111 0011)是 Bit2,3 低电平，这是右侧传感器。意味着线在右边，车需要右转去追线? 
        // 通常: 右侧传感器触发 = 线在右边 = 实际上是“车身偏左”或者“前方右转”，你需要发指令让车往右修。
        // 这里沿用你的定义：0xF3 -> Send 4
        // 0xF3(1111 0011): 右侧两颗
        // 0xFB(1111 1011): 仅 Bit2 (右侧靠中)
        else if (SensorStatus == 0xF3 || SensorStatus == 0xFB) 
        {
            Serial_SendByte(4);
        }
        // 严重偏右 (5)
        // 0xFD(1111 1101): Bit1
        // 0xFE(1111 1110): Bit0 (最右侧)
        else if (SensorStatus == 0xFD || SensorStatus == 0xFE) 
        {
            Serial_SendByte(5);
        }
        // 轻微偏左 (6)
        // 0xCF(1100 1111): 左侧两颗
        // 0xDF(1101 1111): 仅 Bit5 (左侧靠中)
        else if (SensorStatus == 0xCF || SensorStatus == 0xDF) 
        {
            Serial_SendByte(6);
        }
        // 严重偏左 (7)
        // 0xBF(1011 1111): Bit6
        // 0x7F(0111 1111): Bit7 (最左侧)
        else if (SensorStatus == 0xBF || SensorStatus == 0x7F) 
        {
            Serial_SendByte(7);
        }
        // 出线/丢线 (2)
        else if (SensorStatus == 0xFF) 
        {
            Serial_SendByte(2);
        }
    }
}


void TIM1_UP_IRQHandler(void) //1ms进入一次
{
	static uint16_t LedCount = 0;

	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		//清除中断标志位
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);

		//定时器自增区
		LedCount ++;

		//按键
		Key_Tick();

		//Led
		if (LedCount == 1000)
		{
			LedCount = 0;
			LED_Turn();
		}

	}

	time_count = TIM_GetCounter(TIM1);
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
