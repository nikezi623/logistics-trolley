#include "PHC_HeadFile.h"

uint8_t KeyNum, RunFlag; // runflag表示运行启停
uint16_t time_count;

int main(void)
{
    Init_All();

    uint8_t SensorStatus; // 用于存储当前传感器状态
    int16_t dist = 0;     // 激光测距传感器数据
    int16_t raw_dist = 0; // 加一个变量存原始数据

    // 进主循环前，先触发第一次测距！
    MyLaserSensor_StartRanging();

    while (1)
    {
        // 获取一次当前的传感器状态
        SensorStatus = GLE_GetStatus();
        raw_dist = MyLaserSensor_CheckAndRead();

        if (raw_dist != -2) // 如果不等于 -2，说明有新数据了！
        {
            if (raw_dist == 8190 || raw_dist == 0xFFFF)
            {
                dist = -1; // 错误码
            }
            else
            {
                dist = raw_dist - 60; // 减去误差
                if (dist < 0)
                    dist = 0;
            }
            // 拿到新数据后，立刻触发下一次测距！
            MyLaserSensor_StartRanging();
        }

        // OLED
        // --- 1. 基础数据显示 ---
        OLED_ShowBinNum(0, 0, SensorStatus, 8, OLED_8X16);
        OLED_ShowNum(0, 16, KeyNum, 1, OLED_8X16);
        if (dist == -1)
        {
            OLED_Printf(0, 48, OLED_8X16, "Dis:ERROR"); // 错误时显示 ERROR
        }
        else
        {
            OLED_Printf(0, 48, OLED_8X16, "Dis:%04dmm", dist); // 去掉没必要的 + 号，显示四位数字
        }

        // --- 2. 传感器状态可视化 (图形法) ---
        OLED_ClearArea(0, 32, 128, 16);
        for (int i = 0; i < 8; i++)
        {
            if ((SensorStatus & (0x80 >> i)) == 0)
            {
                OLED_DrawRectangle(i * 16, 32, 14, 14, OLED_FILLED);
            }
            else
            {
                OLED_DrawRectangle(i * 16, 32, 14, 14, OLED_UNFILLED);
            }
        }

        OLED_Update();

        // =====================================
        // 1. 严格的直角指令 (半边至少 3-5 个灯全黑才算直角)
        // =====================================
        if (SensorStatus == 0x07 || SensorStatus == 0x0F || SensorStatus == 0x1F)
        {
            Serial_SendByte(9); // 真正的左直角
        }
        else if (SensorStatus == 0xE0 || SensorStatus == 0xF0 || SensorStatus == 0xF8)
        {
            Serial_SendByte(8); // 真正的右直角
        }

        // =====================================
        // 2. 正常巡线逻辑 (细分纠偏)
        // =====================================
        // --- 居中 (3) ---
        else if (SensorStatus == 0xE7)
        {
            Serial_SendByte(31);
        }
        else if (SensorStatus == 0xEF)
        {
            Serial_SendByte(32);
        }
        else if (SensorStatus == 0xF7)
        {
            Serial_SendByte(33);
        }

        // --- 偏右侧 (4, 5) ---
        else if (SensorStatus == 0xF3)
        {
            Serial_SendByte(41);
        }
        else if (SensorStatus == 0xFB)
        {
            Serial_SendByte(42);
        }
        else if (SensorStatus == 0xF9)
        {
            Serial_SendByte(43);
        }

        else if (SensorStatus == 0xFD)
        {
            Serial_SendByte(51);
        }
        else if (SensorStatus == 0xFC)
        {
            Serial_SendByte(52);
        } // 解除注释，找回严重偏右
        else if (SensorStatus == 0xFE)
        {
            Serial_SendByte(53);
        } // 解除注释，找回极度偏右

        // --- 偏左侧 (6, 7) ---
        else if (SensorStatus == 0xCF)
        {
            Serial_SendByte(61);
        }
        else if (SensorStatus == 0xDF)
        {
            Serial_SendByte(62);
        }
        else if (SensorStatus == 0x9F)
        {
            Serial_SendByte(63);
        }

        else if (SensorStatus == 0xBF)
        {
            Serial_SendByte(71);
        }
        else if (SensorStatus == 0x3F)
        {
            Serial_SendByte(72);
        } // 解除注释，找回严重偏左
        else if (SensorStatus == 0x7F)
        {
            Serial_SendByte(73);
        } // 解除注释，找回极度偏左

        // --- 异常状态 ---
        else if (SensorStatus == 0xFF) // 纯白，丢线
        {
            Serial_SendByte(2);
        }
        // 这里你的全黑逻辑也有点宽，0x81 其实是边缘两灯，0x80 和 0x01 是边缘单灯，它们通常不是全黑
        // 建议全黑只留 0x00，或者加严判断
        else if (SensorStatus == 0x00)
        {
            Serial_SendByte(1);
        }
    }
}

void TIM1_UP_IRQHandler(void) // 1ms进入一次
{
    static uint16_t LedCount = 0;

    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
    {
        // 清除中断标志位
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);

        // 定时器自增区
        LedCount++;

        // 按键
        Key_Tick();

        // Led
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
