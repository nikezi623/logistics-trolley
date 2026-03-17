#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "MyI2C.h"

#ifndef __MYLASERSENSOR_H
#define __MYLASERSENSOR_H

// 传感器默认 I2C 地址 (8位)
#define VL53L0X_DEFAULT_ADDRESS  0x52

// 函数声明
void MyLaserSensor_Init(void);
uint16_t MyLaserSensor_ReadDistance(void);

#endif
