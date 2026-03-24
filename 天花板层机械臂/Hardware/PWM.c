#include "stm32f10x.h"                  // Device header

void PWM_Init(void)
{
    /* 1. 开启时钟 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    
    /* 2. 初始化 GPIO (PA0, PA1, PA2, PA3) */
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // 复用推挽输出，把控制权交给定时器
    // 将 4 个引脚全部配置上
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    /* 3. 配置定时器时钟源 */
    TIM_InternalClockConfig(TIM2);
    
    /* 4. 初始化时基单元 (核心修改区：适配舵机 50Hz) */
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    
    // 舵机标准周期为 20ms (50Hz)
    TIM_TimeBaseInitStructure.TIM_Period = 20000 - 1;       // ARR: 自动重装载值
    TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;       // PSC: 预分频器
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);
    
    /* 5. 初始化输出比较单元 (4 个通道) */
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure);                 // 先给结构体赋默认值
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;                      // CCR: 初始占空比设为0
    
    // 初始化 TIM2 的 4 个通道
    TIM_OC1Init(TIM2, &TIM_OCInitStructure); // PA0
    TIM_OC2Init(TIM2, &TIM_OCInitStructure); // PA1
    TIM_OC3Init(TIM2, &TIM_OCInitStructure); // PA2
    TIM_OC4Init(TIM2, &TIM_OCInitStructure); // PA3
    
    /* 6. 开启定时器 */
    TIM_Cmd(TIM2, ENABLE);
}

// 分别设置 4 个通道的占空比 (控制各个舵机的角度)
void PWM_SetCompare1(uint16_t Compare) { TIM_SetCompare1(TIM2, Compare); }
void PWM_SetCompare2(uint16_t Compare) { TIM_SetCompare2(TIM2, Compare); }
void PWM_SetCompare3(uint16_t Compare) { TIM_SetCompare3(TIM2, Compare); }
void PWM_SetCompare4(uint16_t Compare) { TIM_SetCompare4(TIM2, Compare); }
