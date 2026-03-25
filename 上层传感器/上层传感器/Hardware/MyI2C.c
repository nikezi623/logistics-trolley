#include "MyI2C.h"

// 声明两个全局设备结构体
VL53L0X_Dev_t vl53l0x_dev1; 
VL53L0X_Dev_t vl53l0x_dev2; 
VL53L0X_DEV   Dev1 = &vl53l0x_dev1; // 对应 PB10, PB11
VL53L0X_DEV   Dev2 = &vl53l0x_dev2; // 对应 PB6, PB7

static void I2C_Delay(void)
{
	uint8_t i = 10;
	while (i--)
		;
}

void MyI2C_W_SCL(uint8_t bus, uint8_t BitValue)
{
	if (bus == 0)
		GPIO_WriteBit(GPIOB, GPIO_Pin_10, (BitAction)BitValue);
	else
		GPIO_WriteBit(GPIOB, GPIO_Pin_6, (BitAction)BitValue);
	I2C_Delay();
}

void MyI2C_W_SDA(uint8_t bus, uint8_t BitValue)
{
	if (bus == 0)
		GPIO_WriteBit(GPIOB, GPIO_Pin_11, (BitAction)BitValue);
	else
		GPIO_WriteBit(GPIOB, GPIO_Pin_7, (BitAction)BitValue);
	I2C_Delay();
}

uint8_t MyI2C_R_SDA(uint8_t bus)
{
	uint8_t BitValue;
	if (bus == 0)
		BitValue = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11);
	else
		BitValue = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7);
	I2C_Delay();
	return BitValue;
}

void MyI2C_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_SetBits(GPIOB, GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_6 | GPIO_Pin_7);
}

void MyI2C_Start(uint8_t bus)
{
	MyI2C_W_SDA(bus, 1);
	MyI2C_W_SCL(bus, 1);
	MyI2C_W_SDA(bus, 0);
	MyI2C_W_SCL(bus, 0);
}

void MyI2C_Stop(uint8_t bus)
{
	MyI2C_W_SDA(bus, 0);
	MyI2C_W_SCL(bus, 1);
	MyI2C_W_SDA(bus, 1);
}

void MyI2C_SendByte(uint8_t bus, uint8_t Byte)
{
	uint8_t i;
	for (i = 0; i < 8; i++)
	{
		MyI2C_W_SDA(bus, !!(Byte & (0x80 >> i)));
		MyI2C_W_SCL(bus, 1);
		MyI2C_W_SCL(bus, 0);
	}
}

uint8_t MyI2C_ReceiveByte(uint8_t bus)
{
	uint8_t i, Byte = 0x00;
	MyI2C_W_SDA(bus, 1);
	for (i = 0; i < 8; i++)
	{
		MyI2C_W_SCL(bus, 1);
		if (MyI2C_R_SDA(bus))
		{
			Byte |= (0x80 >> i);
		}
		MyI2C_W_SCL(bus, 0);
	}
	return Byte;
}

void MyI2C_SendAck(uint8_t bus, uint8_t AckBit)
{
	MyI2C_W_SDA(bus, AckBit);
	MyI2C_W_SCL(bus, 1);
	MyI2C_W_SCL(bus, 0);
}

uint8_t MyI2C_ReceiveAck(uint8_t bus)
{
	uint8_t AckBit;
	MyI2C_W_SDA(bus, 1);
	MyI2C_W_SCL(bus, 1);
	AckBit = MyI2C_R_SDA(bus);
	MyI2C_W_SCL(bus, 0);
	return AckBit;
}

// 内部函数：初始化单个传感器
static void VL53L0X_InitSingle(VL53L0X_DEV Dev, uint8_t bus_id)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint32_t refSpadCount;
    uint8_t isApertureSpads, VhvSettings, PhaseCal;

    Dev->I2cDevAddr = 0x52; 
    Dev->comms_type = 1;
    Dev->comms_speed_khz = 400;
    Dev->I2cBusId = bus_id; // 绑定总线号

    Status = VL53L0X_DataInit(Dev);
    Status = VL53L0X_StaticInit(Dev);
    Status = VL53L0X_PerformRefSpadManagement(Dev, &refSpadCount, &isApertureSpads);
    Status = VL53L0X_PerformRefCalibration(Dev, &VhvSettings, &PhaseCal);

    Status = VL53L0X_SetDeviceMode(Dev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
    Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(Dev, 33000); 

    if(Status == VL53L0X_ERROR_NONE) {
        VL53L0X_StartMeasurement(Dev); 
    }
}

// 暴露给 main.c 的全局初始化函数
void VL53L0X_InitAll(void)
{
    MyI2C_Init(); // 初始化所有的 GPIO 引脚
    VL53L0X_InitSingle(Dev1, 0); // 启动第一个，分配到总线 0
    VL53L0X_InitSingle(Dev2, 1); // 启动第二个，分配到总线 1
}

// 暴露给 main.c 的读取函数，需要传入想读哪个设备
int16_t VL53L0X_GetDistance_NonBlocking(VL53L0X_DEV Dev)
{
    uint8_t dataReady = 0;
    VL53L0X_Error Status;
    
    Status = VL53L0X_GetMeasurementDataReady(Dev, &dataReady);
    
    if (Status == VL53L0X_ERROR_NONE && dataReady == 1) 
    {
        VL53L0X_RangingMeasurementData_t RangingData;
        Status = VL53L0X_GetRangingMeasurementData(Dev, &RangingData);
        VL53L0X_ClearInterruptMask(Dev, 0);
        
        if (Status == VL53L0X_ERROR_NONE && RangingData.RangeStatus == 0) {
            return RangingData.RangeMilliMeter; 
        } else {
            return -1; 
        }
    }
    else if (Status != VL53L0X_ERROR_NONE) 
    {
        VL53L0X_ClearInterruptMask(Dev, 0);
    }
    return -2; 
}