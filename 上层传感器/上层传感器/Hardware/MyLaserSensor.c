#include "MyLaserSensor.h"
#include "MyI2C.h"
#include "Delay.h"

#define VL53L0X_DEFAULT_ADDRESS  0x52  // 卖家代码是 0x29, 左移一位刚好是 0x52

// 内部函数：向指定寄存器写一个字节
//static void VL53L0X_WriteReg(uint8_t Reg, uint8_t Value)
//{
//    MyI2C_Start();
//    MyI2C_SendByte(VL53L0X_DEFAULT_ADDRESS);
//    MyI2C_ReceiveAck();
//    MyI2C_SendByte(Reg);
//    MyI2C_ReceiveAck();
//    MyI2C_SendByte(Value);
//    MyI2C_ReceiveAck();
//    MyI2C_Stop();
//}

//// 内部函数：从指定寄存器读一个字节
//static uint8_t VL53L0X_ReadReg(uint8_t Reg)
//{
//    uint8_t Value;
//    MyI2C_Start();
//    MyI2C_SendByte(VL53L0X_DEFAULT_ADDRESS);
//    MyI2C_ReceiveAck();
//    MyI2C_SendByte(Reg);
//    MyI2C_ReceiveAck();
//    
//    MyI2C_Start();
//    MyI2C_SendByte(VL53L0X_DEFAULT_ADDRESS | 0x01); 
//    MyI2C_ReceiveAck();
//    Value = MyI2C_ReceiveByte();
//    MyI2C_SendAck(1); 
//    MyI2C_Stop();
//    
//    return Value;
//}

//// 内部函数：从指定寄存器读取两个字节（16位数据，高位在前）
//static uint16_t VL53L0X_ReadReg16(uint8_t Reg)
//{
//    uint16_t Value;
//    MyI2C_Start();
//    MyI2C_SendByte(VL53L0X_DEFAULT_ADDRESS);
//    MyI2C_ReceiveAck();
//    MyI2C_SendByte(Reg);
//    MyI2C_ReceiveAck();
//    
//    MyI2C_Start();
//    MyI2C_SendByte(VL53L0X_DEFAULT_ADDRESS | 0x01);
//    MyI2C_ReceiveAck();
//    Value = MyI2C_ReceiveByte() << 8; // 读高位
//    MyI2C_SendAck(0);                 // ACK
//    Value |= MyI2C_ReceiveByte();     // 读低位
//    MyI2C_SendAck(1);                 // NACK
//    MyI2C_Stop();
//    
//    return Value;
//}

///**
//  * @brief  激光传感器初始化 (极简版)
//  */
//void MyLaserSensor_Init(void)
//{
//    MyI2C_Init();
//    Delay_ms(50); // 上电延时，等待传感器内部初始化完成
//}

///**
//  * @brief  读取一次激光测距距离 (完全复刻卖家 Arduino 逻辑)
//  * @retval 距离值（单位：毫米 mm），如果超时返回 8190
//  */
//uint16_t MyLaserSensor_ReadDistance(void)
//{
//    uint16_t distance = 0;
//    uint8_t val = 0;
//    uint16_t timeout = 0;

//    // 1. 发送触发单次测距命令 (写 0x01 到 SYSRANGE_START 寄存器 0x00)
//    VL53L0X_WriteReg(0x00, 0x01);

//    // 2. 等待测距完成 (轮询 RESULT_RANGE_STATUS 寄存器 0x14 的 bit 0)
//    while (1)
//    {
//        Delay_ms(10); // 卖家代码每次轮询等 10ms
//        val = VL53L0X_ReadReg(0x14);
//        if (val & 0x01) // 如果 bit 0 为 1，说明就绪了
//        {
//            break; 
//        }
//        
//        timeout++;
//        if (timeout > 100) return 8190; // 等待超过 1 秒，返回错误码
//    }

//    // 3. 读取距离数据 (寄存器 0x1E 高字节 和 0x1F 低字节)
//    distance = VL53L0X_ReadReg16(0x1E);

//    return distance;
//}

//// 新增函数 1：只负责发送“开始测距”的指令，绝不等待！
//void MyLaserSensor_StartRanging(void)
//{
//    VL53L0X_WriteReg(0x00, 0x01); // 写入 0x01 开始测距
//}

//// 新增函数 2：检查有没有测完，测完就返回真实距离，没测完就返回一个特殊标记 (比如 -2)
//int16_t MyLaserSensor_CheckAndRead(void)
//{
//    uint8_t val;
//    val = VL53L0X_ReadReg(0x14); // 读状态寄存器
//    
//    if (val & 0x01) // 如果 bit0 为 1，说明测完了
//    {
//        uint16_t dist = VL53L0X_ReadReg16(0x1E); // 把数据读出来
//        
//        // 【修复点 2】清除内部中断标志，允许传感器进行下一次测距！
//        VL53L0X_WriteReg(0x0B, 0x01); 
//        
//        return (int16_t)dist;
//    }
//    else
//    {
//        return -2; // 没测完
//    }
//}
