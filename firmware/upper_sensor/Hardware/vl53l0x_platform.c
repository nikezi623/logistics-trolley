#include "vl53l0x_platform.h"
#include "vl53l0x_api.h"
// 这里包含你的软件 IIC 头文件，比如 "MyI2C.h"
#include "MyI2C.h"
#include "Delay.h"

// 定义传感器 I2C 地址 (8位地址：0x29 << 1 = 0x52)
#define VL53L0X_DEFAULT_I2C_ADDR 0x52

/* --------------------------------------------------------------------------
 * 1. 连续写多个字节 (最核心的底层发送)
 * -------------------------------------------------------------------------- */
VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count)
{
    uint32_t i;
    uint8_t bus = Dev->I2cBusId; // 提取总线号

    MyI2C_Start(bus);
    MyI2C_SendByte(bus, Dev->I2cDevAddr);
    if (MyI2C_ReceiveAck(bus) != 0)
    {
        MyI2C_Stop(bus);
        return VL53L0X_ERROR_CONTROL_INTERFACE;
    }

    MyI2C_SendByte(bus, index);
    if (MyI2C_ReceiveAck(bus) != 0)
    {
        MyI2C_Stop(bus);
        return VL53L0X_ERROR_CONTROL_INTERFACE;
    }

    for (i = 0; i < count; i++)
    {
        MyI2C_SendByte(bus, pdata[i]);
        if (MyI2C_ReceiveAck(bus) != 0)
        {
            MyI2C_Stop(bus);
            return VL53L0X_ERROR_CONTROL_INTERFACE;
        }
    }
    MyI2C_Stop(bus);
    return VL53L0X_ERROR_NONE;
}

/* --------------------------------------------------------------------------
 * 2. 连续读多个字节 (最核心的底层接收)
 * -------------------------------------------------------------------------- */
VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count)
{
    uint32_t i;
    uint8_t bus = Dev->I2cBusId; // 提取总线号

    MyI2C_Start(bus);
    MyI2C_SendByte(bus, Dev->I2cDevAddr);
    if (MyI2C_ReceiveAck(bus) != 0)
    {
        MyI2C_Stop(bus);
        return VL53L0X_ERROR_CONTROL_INTERFACE;
    }

    MyI2C_SendByte(bus, index);
    if (MyI2C_ReceiveAck(bus) != 0)
    {
        MyI2C_Stop(bus);
        return VL53L0X_ERROR_CONTROL_INTERFACE;
    }

    MyI2C_Start(bus);
    MyI2C_SendByte(bus, Dev->I2cDevAddr | 0x01);
    if (MyI2C_ReceiveAck(bus) != 0)
    {
        MyI2C_Stop(bus);
        return VL53L0X_ERROR_CONTROL_INTERFACE;
    }

    for (i = 0; i < count; i++)
    {
        pdata[i] = MyI2C_ReceiveByte(bus);
        if (i == count - 1)
        {
            MyI2C_SendAck(bus, 1);
        }
        else
        {
            MyI2C_SendAck(bus, 0);
        }
    }
    MyI2C_Stop(bus);
    return VL53L0X_ERROR_NONE;
}

/* --------------------------------------------------------------------------
 * 3. 封装单字节、双字节、四字节的读写 (VL53L0X 是大端模式，高字节在前)
 * -------------------------------------------------------------------------- */
VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV Dev, uint8_t index, uint8_t data)
{
    return VL53L0X_WriteMulti(Dev, index, &data, 1);
}

VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV Dev, uint8_t index, uint16_t data)
{
    uint8_t buffer[2];
    buffer[0] = (uint8_t)(data >> 8);
    buffer[1] = (uint8_t)(data & 0xFF);
    return VL53L0X_WriteMulti(Dev, index, buffer, 2);
}

VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t data)
{
    uint8_t buffer[4];
    buffer[0] = (uint8_t)(data >> 24);
    buffer[1] = (uint8_t)((data >> 16) & 0xFF);
    buffer[2] = (uint8_t)((data >> 8) & 0xFF);
    buffer[3] = (uint8_t)(data & 0xFF);
    return VL53L0X_WriteMulti(Dev, index, buffer, 4);
}

VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV Dev, uint8_t index, uint8_t *data)
{
    return VL53L0X_ReadMulti(Dev, index, data, 1);
}

VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV Dev, uint8_t index, uint16_t *data)
{
    VL53L0X_Error Status;
    uint8_t buffer[2];
    Status = VL53L0X_ReadMulti(Dev, index, buffer, 2);
    *data = ((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1];
    return Status;
}

VL53L0X_Error VL53L0X_RdDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t *data)
{
    VL53L0X_Error Status;
    uint8_t buffer[4];
    Status = VL53L0X_ReadMulti(Dev, index, buffer, 4);
    *data = ((uint32_t)buffer[0] << 24) | ((uint32_t)buffer[1] << 16) | ((uint32_t)buffer[2] << 8) | (uint32_t)buffer[3];
    return Status;
}

VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV Dev, uint8_t index, uint8_t AndData, uint8_t OrData)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t data;

    // 先读出当前寄存器的值
    Status = VL53L0X_RdByte(Dev, index, &data);
    if (Status == VL53L0X_ERROR_NONE)
    {
        // 进行位掩码操作后重新写入
        data = (data & AndData) | OrData;
        Status = VL53L0X_WrByte(Dev, index, data);
    }
    return Status;
}

/* --------------------------------------------------------------------------
 * 补充函数 2：API 内部轮询等待时的延时函数
 * -------------------------------------------------------------------------- */
VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV Dev)
{
    // 调用你的标准库毫秒延时函数，通常 2ms 到 5ms 即可
    Delay_ms(2);
    return VL53L0X_ERROR_NONE;
}
