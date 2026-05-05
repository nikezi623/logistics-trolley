#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "vl53l0x_api.h"

static void I2C_Delay(void) {
    uint8_t i = 10; 
    while(i--); 
}

void MyI2C_W_SCL(uint8_t BitValue)
{
	GPIO_WriteBit(GPIOB, GPIO_Pin_10, (BitAction)BitValue);
	I2C_Delay();
}

void MyI2C_W_SDA(uint8_t BitValue)
{
	GPIO_WriteBit(GPIOB, GPIO_Pin_11, (BitAction)BitValue);
	I2C_Delay();
}

uint8_t MyI2C_R_SDA(void)
{
	uint8_t BitValue;
	BitValue = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11);
	I2C_Delay();
	return BitValue;
}

void MyI2C_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOB, GPIO_Pin_10 | GPIO_Pin_11);
}

void MyI2C_Start(void)
{
	MyI2C_W_SDA(1);
	MyI2C_W_SCL(1);
	MyI2C_W_SDA(0);
	MyI2C_W_SCL(0);
}

void MyI2C_Stop(void)
{
	MyI2C_W_SDA(0);
	MyI2C_W_SCL(1);
	MyI2C_W_SDA(1);
}

void MyI2C_SendByte(uint8_t Byte)
{
	uint8_t i;
	for (i = 0; i < 8; i ++)
	{
		MyI2C_W_SDA(!!(Byte & (0x80 >> i)));
		MyI2C_W_SCL(1);
		MyI2C_W_SCL(0);
	}
}

uint8_t MyI2C_ReceiveByte(void)
{
	uint8_t i, Byte = 0x00;
	MyI2C_W_SDA(1);
	for (i = 0; i < 8; i ++)
	{
		MyI2C_W_SCL(1);
		if (MyI2C_R_SDA()){Byte |= (0x80 >> i);}
		MyI2C_W_SCL(0);
	}
	return Byte;
}

void MyI2C_SendAck(uint8_t AckBit)
{
	MyI2C_W_SDA(AckBit);
	MyI2C_W_SCL(1);
	MyI2C_W_SCL(0);
}

uint8_t MyI2C_ReceiveAck(void)
{
	uint8_t AckBit;
	MyI2C_W_SDA(1);
	MyI2C_W_SCL(1);
	AckBit = MyI2C_R_SDA();
	MyI2C_W_SCL(0);
	return AckBit;
}

VL53L0X_Dev_t vl53l0x_dev; // 声明全局设备结构体
VL53L0X_DEV   Dev = &vl53l0x_dev;

// 1. 修改初始化函数：设置为连续模式并启动
void VL53L0X_Init(void)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;

    MyI2C_Init();

    Dev->I2cDevAddr = 0x52; 
    Dev->comms_type = 1;
    Dev->comms_speed_khz = 400;

    Status = VL53L0X_DataInit(Dev);
    Status = VL53L0X_StaticInit(Dev);
    Status = VL53L0X_PerformRefSpadManagement(Dev, &refSpadCount, &isApertureSpads);
    Status = VL53L0X_PerformRefCalibration(Dev, &VhvSettings, &PhaseCal);

    // 【修改点 1】改为连续测距模式 (CONTINUOUS_RANGING)
    Status = VL53L0X_SetDeviceMode(Dev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
    Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(Dev, 33000); 

    // 【修改点 2】直接在这里启动测量，之后它就会自己在后台不断测了
    if(Status == VL53L0X_ERROR_NONE) {
        VL53L0X_StartMeasurement(Dev); 
    }
}

// 2. 新增非阻塞读取函数：绝不卡主循环！
int16_t VL53L0X_GetDistance_NonBlocking(void)
{
    uint8_t dataReady = 0;
    VL53L0X_Error Status;
    
    // 查询是否有新数据 (瞬间返回，不阻塞)
    Status = VL53L0X_GetMeasurementDataReady(Dev, &dataReady);
    
    if (Status == VL53L0X_ERROR_NONE && dataReady == 1) 
    {
        VL53L0X_RangingMeasurementData_t RangingData;
        
        // 读取准备好的数据
        Status = VL53L0X_GetRangingMeasurementData(Dev, &RangingData);
        
        // 【关键】必须清除中断标志，传感器才会开始下一次测距
        VL53L0X_ClearInterruptMask(Dev, 0);
        
        if (RangingData.RangeStatus == 0) {
            return RangingData.RangeMilliMeter; // 成功拿到数据
        } else {
            return -1; // 数据超出量程或无效
        }
    }
    return -2; // 【关键】数据还没准备好，直接返回 -2，主循环继续跑！
}
