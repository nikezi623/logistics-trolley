#include "PHC_HeadFile.h"

uint8_t KeyNum, RunFlag;//runflag表示运行启停
uint16_t time_count;

int main(void)
{
    Init_All();
    uint8_t SensorStatus = 0xE7; // 用于存储当前传感器状态
    uint8_t SensorStatus_ = 0xE7; // 用于存储当前传感器状态

    while (1)
    {


        /*注释掉传感器获取数据，改用遥杆代替传感器*/

        // 获取一次当前的传感器状态
        // SensorStatus = GLE_GetStatus(); 

        // 使用蓝牙遥杆代替巡线
		if (Serial_GetRxFlag() == 1)
        {
            SensorStatus = Serial_GetRxData();
        }

        // OLED
        // --- 1. 基础数据显示 ---
        OLED_ShowBinNum(0, 0, SensorStatus, 8, OLED_8X16); 
        OLED_ShowNum(0, 16, KeyNum, 1, OLED_8X16);

        // --- 2. 传感器状态可视化 (图形法) ---
        OLED_ClearArea(0, 32, 128, 16); 
        for (int i = 0; i < 8; i++) 
        {
            if ((SensorStatus_ & (0x80 >> i)) == 0) 
            {
                OLED_DrawRectangle(i * 16, 32, 14, 14, OLED_FILLED); 
            } 
            else 
            {
                OLED_DrawRectangle(i * 16, 32, 14, 14, OLED_UNFILLED);
            }
        }

        OLED_Update();
        
        // 左转右转指令
        if (SensorStatus == 9)
        {
             Serial_SendByte(9); // 左转指令
             SensorStatus_ = 0x07;
        }
        else if (SensorStatus == 8)
        {
             Serial_SendByte(8); // 右转指令
             SensorStatus_ = 0XE0;
        }
        
        // --- 正常巡线逻辑 ---

        // 完美居中 (3)
        // 0xE7(1110 0111): 中间两颗
        // 0xEF(1110 1111): 仅左中一颗
        // 0xF7(1111 0111): 仅右中一颗
        else if (SensorStatus == 3) 
        {
            Serial_SendByte(3);
            SensorStatus_ = 0xE7;
        }
        // 轻微偏右 (4) -> 车身偏右，线在车身左侧 (Wait, if sensor 2/3 is active (Right side sensors), line is to the Right)
        // 你的逻辑: 0xF3(1111 0011)是 Bit2,3 低电平，这是右侧传感器。意味着线在右边，车需要右转去追线? 
        // 通常: 右侧传感器触发 = 线在右边 = 实际上是“车身偏左”或者“前方右转”，你需要发指令让车往右修。
        // 这里沿用你的定义：0xF3 -> Send 4
        // 0xF3(1111 0011): 右侧两颗
        // 0xFB(1111 1011): 仅 Bit2 (右侧靠中)
        else if (SensorStatus == 4) 
        {
            Serial_SendByte(4);
            SensorStatus_ = 0xF3;
        }
        // 严重偏右 (5)
        // 0xFD(1111 1101): Bit1
        // 0xFE(1111 1110): Bit0 (最右侧)
        else if (SensorStatus == 5) 
        {
            Serial_SendByte(5);
            SensorStatus_ = 0xFD;
        }
        // 轻微偏左 (6)
        // 0xCF(1100 1111): 左侧两颗
        // 0xDF(1101 1111): 仅 Bit5 (左侧靠中)
        else if (SensorStatus == 6) 
        {
            Serial_SendByte(6);
            SensorStatus_ = 0xCF;
        }
        // 严重偏左 (7)
        // 0xBF(1011 1111): Bit6
        // 0x7F(0111 1111): Bit7 (最左侧)
        else if (SensorStatus == 7) 
        {
            Serial_SendByte(7);
            SensorStatus_ = 0xBF;
        }
        // 出线/丢线 (2)
        else if (SensorStatus == 2) 
        {
            Serial_SendByte(2);
            SensorStatus_ = 0xFF;
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
