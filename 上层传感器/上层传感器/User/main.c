#include "PHC_HeadFile.h"

extern VL53L0X_DEV Dev1;
extern VL53L0X_DEV Dev2;
extern void VL53L0X_InitAll(void);
extern int16_t VL53L0X_GetDistance_NonBlocking(VL53L0X_DEV Dev);

uint8_t KeyNum, RunFlag; // runflag表示运行启停
uint16_t time_count;

uint8_t avoid_count = 0;   // 连续检测到障碍物的次数
uint8_t Obstacle_Flag = 0; // 确认为真实障碍物的标志位

uint8_t RxCmd;

int main(void)
{
    Init_All();

    // 2. 立刻接管屏幕，显示“加载中”提示！
    OLED_Clear();
    OLED_Printf(0, 0, OLED_8X16, "System Booting..");
    OLED_Printf(0, 32, OLED_8X16, "Init VL53L0X x2");
    OLED_Update();
    // 现在上电瞬间屏幕就会亮起，你就知道单片机没死机

    // 调用全新的 API 初始化 (这会执行校准逻辑)
    VL53L0X_InitAll();
    Trigger_Middle_Init();
    // Trigger_Set_Low(GPIOA, GPIO_Pin_12);

    uint8_t SensorStatus;  // 用于存储当前传感器状态
    int16_t dist1 = 0;     // 激光测距传感器数据
    int16_t dist2 = 0;     // 激光测距传感器数据
    int16_t raw_dist1 = 0; // 加一个变量存原始数据
    int16_t raw_dist2 = 0; // 加一个变量存原始数据

    // 进主循环前，先触发第一次测距！
    //    MyLaserSensor_StartRanging();
    OLED_Clear();
    OLED_Update();
    
    while (1)
    {
        // 获取一次当前的传感器状态
        SensorStatus = GLE_GetStatus();
        // raw_dist = VL53L0X_GetDistance_NonBlocking();

        raw_dist1 = VL53L0X_GetDistance_NonBlocking(Dev1);
        raw_dist2 = VL53L0X_GetDistance_NonBlocking(Dev2);

        // 【核心修复】加入“漏桶容错”算法，防止圆柱体边缘散射导致误清零
        if (raw_dist1 != -2)
        {
            if (raw_dist1 > 0 && raw_dist1 <= 300)
            {
                dist1 = raw_dist1;
                avoid_count += 2; // 看到障碍物，快速增加置信度
                if (avoid_count > 6)
                    avoid_count = 6; // 设定一个上限
            }
            else
            {
                if (raw_dist1 > 0)
                    dist1 = raw_dist1;
                else if (raw_dist1 == -1)
                    dist1 = -1;

                if (avoid_count > 0)
                    avoid_count--; // 没看到障碍物或报错，缓慢扣分（容错）
            }

            // 分数达到 4 分（相当于看到两次），才确认有障碍物
            if (avoid_count >= 2)
            {
                Obstacle_Flag = 1;
            }
            // 分数彻底扣完，才解除障碍物警报
            else if (avoid_count == 0)
            {
                Obstacle_Flag = 0;
            }
        }

        // 【新增 D2 过滤逻辑】和 D1 一样，只有测好了才更新 dist2
        if (raw_dist2 != -2)
        {
            if (raw_dist2 > 0)
            {
                dist2 = raw_dist2; // 更新最新距离
            }
            else if (raw_dist2 == -1)
            {
                dist2 = -1; // 标记报错状态
            }
        }

        // OLED
        // --- 1. 基础数据显示 ---
        OLED_ShowBinNum(0, 0, SensorStatus, 8, OLED_8X16);
        if (dist1 == -1)
        {
            OLED_Printf(0, 16, OLED_8X16, "D1:ERROR"); // 错误时显示 ERROR
        }
        else
        {
            OLED_Printf(0, 16, OLED_8X16, "D1:%04dmm", dist1); // 去掉没必要的 + 号，显示四位数字
        }

        OLED_Printf(0, 48, OLED_8X16, "D2:%04dmm", dist2);

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

        if (Obstacle_Flag == 1)
        {
            Serial_SendByte(99);
        }
        // =====================================
        // 1. 严格的直角指令 (半边至少 3-5 个灯全黑才算直角)
        // 传感器状态：0 = 压黑线，1 = 白底白线
        // =====================================
        else if (SensorStatus == 0x07 || SensorStatus == 0x03 || SensorStatus == 0x01) // 0b 0000 0111 (左边5个探头全压线)
        {
            Serial_SendByte(91); // 真正的左直角
        }
        else if (SensorStatus == 0x0F) // 0b 0000 1111 (左边4个探头压线)
        {
            Serial_SendByte(92); // 真正的左直角
        }
        else if (SensorStatus == 0x1F) // 0b 0001 1111 (左边3个探头压线)
        {
            Serial_SendByte(93); // 真正的左直角
        }
        else if (SensorStatus == 0xE0 || SensorStatus == 0xC0 || SensorStatus == 0x80) // 0b 1110 0000 (右边5个探头全压线)
        {
            Serial_SendByte(81); // 真正的右直角
        }
        else if (SensorStatus == 0xF0) // 0b 1111 0000 (右边4个探头压线)
        {
            Serial_SendByte(82); // 真正的右直角
        }
        else if (SensorStatus == 0xF8) // 0b 1111 1000 (右边3个探头压线)
        {
            Serial_SendByte(83); // 真正的右直角
        }

        // =====================================
        // 2. 正常巡线逻辑 (细分纠偏)
        // =====================================
        // --- 居中 (3) ---
        else if (SensorStatus == 0xE7) // 0b 1110 0111 (中间2个探头压线)
        {
            Serial_SendByte(31);
        }
        else if (SensorStatus == 0xEF) // 0b 1110 1111 (中间偏左1个探头压线)
        {
            Serial_SendByte(32);
        }
        else if (SensorStatus == 0xF7) // 0b 1111 0111 (中间偏右1个探头压线)
        {
            Serial_SendByte(33);
        }

        // --- 偏右侧 (4, 5) ---
        else if (SensorStatus == 0xF3) // 0b 1111 0011 (右侧2个探头压线)
        {
            Serial_SendByte(41);
        }
        else if (SensorStatus == 0xFB) // 0b 1111 1011 (右侧偏内1个探头压线)
        {
            Serial_SendByte(42);
        }
        else if (SensorStatus == 0xF9) // 0b 1111 1001 (右侧偏外2个探头压线)
        {
            Serial_SendByte(43);
        }
        else if (SensorStatus == 0xFD) // 0b 1111 1101 (最右侧第2个探头压线)
        {
            Serial_SendByte(51);
        }
        else if (SensorStatus == 0xFC) // 0b 1111 1100 (最右侧边缘2个探头压线)
        {
            Serial_SendByte(52);
        }
        else if (SensorStatus == 0xFE) // 0b 1111 1110 (最右侧单探头压线)
        {
            Serial_SendByte(53);
        }

        // --- 偏左侧 (6, 7) ---
        else if (SensorStatus == 0xCF) // 0b 1100 1111 (左侧2个探头压线)
        {
            Serial_SendByte(61);
        }
        else if (SensorStatus == 0xDF) // 0b 1101 1111 (左侧偏内1个探头压线)
        {
            Serial_SendByte(62);
        }
        else if (SensorStatus == 0x9F) // 0b 1001 1111 (左侧偏外2个探头压线)
        {
            Serial_SendByte(63);
        }
        else if (SensorStatus == 0xBF) // 0b 1011 1111 (最左侧第2个探头压线)
        {
            Serial_SendByte(71);
        }
        else if (SensorStatus == 0x3F) // 0b 0011 1111 (最左侧边缘2个探头压线)
        {
            Serial_SendByte(72);
        }
        else if (SensorStatus == 0x7F) // 0b 0111 1111 (最左侧单探头压线)
        {
            Serial_SendByte(73);
        }

        // --- 异常状态 ---
        else if (SensorStatus == 0xFF) // 0b 1111 1111 (纯白，全丢线)
        {
            Serial_SendByte(2);
        }
        else if (SensorStatus == 0x00) // 0b 0000 0000 (纯黑，或者路口中心)
        {
            Serial_SendByte(1);
        }
    }
}

void TIM1_UP_IRQHandler(void) // 1ms进入一次
{

    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
    {
        // 清除中断标志位
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);

        if (Serial_GetRxFlag() == 1)
        {
            RxCmd = Serial_GetRxData();
        }

        if (RxCmd == 86)
        {
            Trigger_Set_High(GPIOA, GPIO_Pin_12);
        }
        // else
        // {
        //     Trigger_Set_Low(GPIOA, GPIO_Pin_12);
        // }
    }
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
