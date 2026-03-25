#ifndef __MYI2C_H
#define __MYI2C_H

#include "stm32f10x.h" // Device header
#include "Delay.h"
#include "vl53l0x_api.h"

void MyI2C_Init(void);
void MyI2C_Start(uint8_t bus);
void MyI2C_Stop(uint8_t bus);
void MyI2C_SendByte(uint8_t bus, uint8_t Byte);
uint8_t MyI2C_ReceiveByte(uint8_t bus);
void MyI2C_SendAck(uint8_t bus, uint8_t AckBit);
uint8_t MyI2C_ReceiveAck(uint8_t bus);
void VL53L0X_InitAll(void);
int16_t VL53L0X_GetDistance_NonBlocking(VL53L0X_DEV Dev);
#endif
