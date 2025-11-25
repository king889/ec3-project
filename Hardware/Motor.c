#include "stm32f10x.h"                  // Device header
#include "PWM.h"

void Motor_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
    
    // 电机1 (PB12, PB13, TIM3)
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    
    // 电机1方向控制引脚
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // 电机2方向控制引脚 (PB14, PB15)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // 初始化PWM (假设PWM_Init()配置TIM2和TIM3)
    PWM_Init();  // 需要确保PWM_Init()配置TIM2(PA3)和TIM3
}

// 电机1控制 (PB12, PB13, TIM3)
void Motor_SetPWM1(int8_t PWM)
{
    if (PWM >= 0)
    {
        GPIO_ResetBits(GPIOB, GPIO_Pin_12);  // AIN1 = 0
        GPIO_SetBits(GPIOB, GPIO_Pin_13);    // AIN2 = 1
        PWM_SetCompare3(PWM);                // TIM3 PWM输出
    }
    else
    {
        GPIO_SetBits(GPIOB, GPIO_Pin_12);    // AIN1 = 1
        GPIO_ResetBits(GPIOB, GPIO_Pin_13);  // AIN2 = 0
        PWM_SetCompare3(-PWM);               // TIM3 PWM输出
    }
}

// 电机2控制 (PB14, PB15, PA3 - TIM2_CH3)
void Motor_SetPWM2(int8_t PWM)
{
    if (PWM >= 0)
    {
        GPIO_ResetBits(GPIOB, GPIO_Pin_14);  // BIN1 = 0
        GPIO_SetBits(GPIOB, GPIO_Pin_15);    // BIN2 = 1
        PWM_SetCompare4(PWM);                // TIM2 PWM输出到PA3
    }
    else
    {
        GPIO_SetBits(GPIOB, GPIO_Pin_14);    // BIN1 = 1
        GPIO_ResetBits(GPIOB, GPIO_Pin_15);  // BIN2 = 0
        PWM_SetCompare4(-PWM);               // TIM2 PWM输出到PA3
    }
}
