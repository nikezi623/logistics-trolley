#include "stm32f10x.h" // Device header

float Get_Vision_Error(uint8_t cmd)
{
    float error = 0;

    switch (cmd)
    {
    // === 居中微调 (3系列) ===
    // 原来只有一个 0，现在细分出线稍微偏左或偏右的情况
    case 31:
        error = 0.0;
        break; // 完美居中 (Bit3, Bit4)
    case 32:
        error = -0.2;
        break; // 极其轻微偏左 (仅 Bit4)
    case 33:
        error = 0.2;
        break; // 极其轻微偏右 (仅 Bit3)

    // === 偏右侧 (线在右，车偏左，需要右转，Error为正) ===
    case 41:
        error = 0.4;
        break; // 轻微偏右 (Bit2, Bit3)
    case 42:
        error = 0.6;
        break; // 偏右 (仅 Bit2) —— 对应旧版的 4
    case 43:
        error = 0.8;
        break; // 较明显偏右 (Bit1, Bit2)

    case 51:
        error = 1.0;
        break; // 严重偏右 (仅 Bit1)
    case 52:
        error = 1.2;
        break; // 很严重偏右 (Bit0, Bit1) —— 对应旧版的 5
    case 53:
        error = 1.4;
        break; // 极度偏右 (仅 Bit0，最右侧传感器)

    // === 偏左侧 (线在左，车偏右，需要左转，Error为负) ===
    case 61:
        error = -0.4;
        break; // 轻微偏左 (Bit4, Bit5)
    case 62:
        error = -0.6;
        break; // 偏左 (仅 Bit5) —— 对应旧版的 6
    case 63:
        error = -0.8;
        break; // 较明显偏左 (Bit5, Bit6)

    case 71:
        error = -1.0;
        break; // 严重偏左 (仅 Bit6)
    case 72:
        error = -1.2;
        break; // 很严重偏左 (Bit6, Bit7) —— 对应旧版的 7
    case 73:
        error = -1.4;
        break; // 极度偏左 (仅 Bit7，最左侧传感器)

    default:
        error = 0;
        break; // 未知情况暂按0处理
    }

    return error;
}