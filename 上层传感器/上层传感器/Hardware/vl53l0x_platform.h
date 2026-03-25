#ifndef _VL53L0X_PLATFORM_H_
#define _VL53L0X_PLATFORM_H_

#include <stdint.h>
#include <string.h> /* 提供 strcpy 支持 */
#include <stdlib.h> /* 提供 abs() 支持 */

/* 1. 打破循环依赖：先声明结构体和指针，再包含官方 def 头文件 */
typedef struct VL53L0X_Dev_t VL53L0X_Dev_t;
typedef VL53L0X_Dev_t *VL53L0X_DEV;

#include "vl53l0x_def.h"

/* 2. 定义设备结构体 */
struct VL53L0X_Dev_t
{
    VL53L0X_DevData_t Data;   /*!< ST API 强依赖这个名为 Data 的成员！ */
    uint8_t I2cDevAddr;       /*!< I2C 从机地址 */
    uint8_t comms_type;       /*!< 通信类型 */
    uint16_t comms_speed_khz; /*!< 通信速率 */
    uint8_t I2cBusId;         // <--- 新增：标识 I2C 总线号 (0或1)
};

/* 3. 屏蔽官方内部的繁琐日志打印 */
#define VL53L0X_LOG_ENABLE 0
#define TRACE_MODULE_API 0
#define _LOG_FUNCTION_START(module, fmt, ...) (void)0
#define _LOG_FUNCTION_END(module, status, ...) (void)0
#define _LOG_FUNCTION_END_FMT(module, status, fmt, ...) (void)0

/* --------------------------------------------------------------------------
 * 4. 核心大坑修复：将 ST 官方 API 需要的宏映射到我们定义的结构体上
 * -------------------------------------------------------------------------- */
#define PALDevDataGet(Dev, field) \
    (Dev->Data.field)
#define PALDevDataSet(Dev, field, data) \
    (Dev->Data.field) = (data)

//#define VL53L0X_GETDEVICESPECIFICPARAMETER(Dev, field) \
//    (Dev->Data.DeviceSpecificParameters.field)
//#define VL53L0X_SETDEVICESPECIFICPARAMETER(Dev, field, value) \
//    (Dev->Data.DeviceSpecificParameters.field) = (value)

//#define VL53L0X_GETPARAMETERFIELD(Dev, field, variable) \
//    (variable) = (Dev->Data.CurrentParameters.field)
//#define VL53L0X_SETPARAMETERFIELD(Dev, field, value) \
//    (Dev->Data.CurrentParameters.field) = (value)

//#define VL53L0X_GETARRAYPARAMETERFIELD(Dev, field, index, variable) \
//    (variable) = (Dev->Data.CurrentParameters.field[index])
//#define VL53L0X_SETARRAYPARAMETERFIELD(Dev, field, index, value) \
//    (Dev->Data.CurrentParameters.field[index]) = (value)

#define VL53L0X_COPYSTRING(str, src) strcpy(str, src)

/* 5. 底层读写接口声明 */
VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count);
VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count);
VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV Dev, uint8_t index, uint8_t data);
VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV Dev, uint8_t index, uint16_t data);
VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t data);
VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV Dev, uint8_t index, uint8_t *data);
VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV Dev, uint8_t index, uint16_t *data);
VL53L0X_Error VL53L0X_RdDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t *data);

VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV Dev, uint8_t index, uint8_t AndData, uint8_t OrData);
VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV Dev);

#endif
