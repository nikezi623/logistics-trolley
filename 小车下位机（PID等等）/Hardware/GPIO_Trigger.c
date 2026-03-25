#include "GPIO_Trigger.h"

void Trigger_Middle_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; // 推挽输出
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 默认输出低电平，避免误触发
    GPIO_ResetBits(GPIOA, GPIO_Pin_12);
}

/**
  * @brief  天层 (Top) 初始化
  * @note   PA12 配置为下拉输入 (接收上层信号)
  * PA15 配置为推挽输出 (完成时通知下层)，注意 PA15 需要解除 JTAG 占用
  */
void Trigger_Top_Init(void)
{
    // 1. 开启 GPIOA 和 AFIO 时钟 (AFIO用于引脚重映射)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

    // 2. 解除 JTAG 占用，保留 SWD 下载功能 (非常关键，否则 PA15 无法使用)
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;

    // 3. 配置 PA12 为下拉输入 (没信号时保持低电平，防止悬空被干扰)
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 4. 配置 PA15 为推挽输出
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 默认 PA15 输出低电平
    GPIO_ResetBits(GPIOA, GPIO_Pin_15);
}

/**
  * @brief  下层 (Lower) 初始化
  * @note   PA15 配置为下拉输入 (接收天层完成信号)，同样需要解除 JTAG 占用
  */
void Trigger_Lower_Init(void)
{
    // 1. 开启 GPIOA 和 AFIO 时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

    // 2. 解除 JTAG 占用，保留 SWD 下载功能
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

    // 3. 配置 PA15 为下拉输入
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

// ==========================================
//                 通用读写函数
// ==========================================

/**
  * @brief  将指定引脚拉高
  */
void Trigger_Set_High(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    GPIO_SetBits(GPIOx, GPIO_Pin);
}

/**
  * @brief  将指定引脚拉低
  */
void Trigger_Set_Low(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    GPIO_ResetBits(GPIOx, GPIO_Pin);
}

/**
  * @brief  读取指定引脚的状态
  * @retval 1: 高电平, 0: 低电平
  */
uint8_t Trigger_Read_Pin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    return GPIO_ReadInputDataBit(GPIOx, GPIO_Pin);
}
