#ifndef __GPIO_TRIGGER_H
#define __GPIO_TRIGGER_H

#include "stm32f10x.h"                  // Device header

// --- 初始化函数声明 ---
void Trigger_Middle_Init(void);  // 上层：初始化 PA12 (输出)
void Trigger_Top_Init(void);     // 天层：初始化 PA12 (输入), PA15 (输出)
void Trigger_Lower_Init(void);   // 下层：初始化 PA15 (输入)

// --- 读写控制函数声明 ---
void Trigger_Set_High(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void Trigger_Set_Low(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
uint8_t Trigger_Read_Pin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

#endif
