#include "stm32f10x.h"                  // Device header

float Get_Vision_Error(uint8_t cmd)
{
    float error = 0;
    switch (cmd)
    {
        case 3: error = 0;    break; // 完美居中
        case 4: error = 0.8;  break; // 轻微偏右 (车身偏左，需向右转) -> 实际上 RxCmd 定义一般是 "线在车的哪边"
                                     // 如果 RxCmd=4 代表"线在右边"，则 Error 应为正，让 TurnTarget > 0 (假设 TurnTarget>0 是右转)
        case 5: error = 1.6;  break; // 严重偏右
        case 6: error = -0.8; break; // 轻微偏左
        case 7: error = -1.6; break; // 严重偏左
    //    case 8: error = 14.0;  break; // 右直角/大弯
    //    case 9: error = -14.0; break; // 左直角/大弯
        default: error = 0;   break; // 未知情况暂按0处理
    }
    return error;
}
