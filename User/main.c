#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "line.h"
#include "motor.h"
#include <stdint.h>
#include "PWM.h"
#include "Timer.h"
#include "Encoder.h"
#include "PID.h"
#include <stdio.h>
#include "Serial.h"

// 全局变量
int16_t Speed1, Speed2;

// 速度调整相关变量
// uint32_t SpeedAdjustTimer = 0;  // 速度调整计时器
// uint8_t SpeedIndex = 0;         // 当前速度索引
// const float SpeedTargets[] = {20.0f, 40.0f, 60.0f, 80.0f, 100.0f, 80.0f, 60.0f, 40.0f}; // 速度目标数组
// const uint8_t SpeedCount = sizeof(SpeedTargets) / sizeof(SpeedTargets[0]); // 速度目标数量

// 电机1 左手边的电机
PID_t Motor1_PID = {
    .Target = 0,
    .Actual = 0,
    .Out = 0,
    .Kp = 0.35,
    .Ki = 0.02, 
    .Kd = 0.4,
    .Error0 = 0,
    .Error1 = 0,
    .Error2 = 0,
    .OutMax = 130,
    .DeadZone = 0,
    .OutMin = -130
};

// 电机2 右手边的电机
PID_t Motor2_PID = {
    .Target = 30,
    .Actual = 0,
    .Out = 0,
    .Kp = 0.35,
    .Ki = 0.02, 
    .Kd = 0.4,
    .Error0 = 0,
    .Error1 = 0,
    .Error2 = 0,
    .OutMax = 130,
    .DeadZone = 0,
    .OutMin = -130
};

void LineFollowControl(uint8_t mask)
{
    // 基础速度
    float base_speed = 40.0f;
    
    // 根据掩码判断转向
    switch(mask)
    {
        case 0x00:  // 全黑 - 直行
        case 0x1F:  // 全白 - 直行
            Motor1_PID.Target = base_speed;
            Motor2_PID.Target = base_speed;
            break;
            
        case 0x01:  // 最右边没线 - 轻微左转
            Motor1_PID.Target = 50;
            Motor2_PID.Target = -50;
            break;
            
        case 0x03:  // 右边两边没线 - 中等左转
            Motor1_PID.Target = 80;
            Motor2_PID.Target = -80;
            break;
            
        case 0x07:  // 右边三边没线 - 较大左转
            Motor1_PID.Target = 100;
            Motor2_PID.Target = -100;
            break;
            
        case 0x10:  // 最左边没线 - 轻微右转
            Motor1_PID.Target = -50;
            Motor2_PID.Target = 50;
            break;
            
        case 0x18:  // 左边两边没线 - 中等右转
            Motor1_PID.Target = -80;
            Motor2_PID.Target = 80;
            break;
            
        case 0x1C:  // 左边三边没线 - 较大右转
            Motor1_PID.Target = 100;
            Motor2_PID.Target = -100;
            break;
            
        case 0x02:  // 中间偏右 - 小右转
        case 0x04:  // 中间 - 直行
        case 0x08:  // 中间偏左 - 小左转
        default:    // 其他情况 - 直行
            Motor1_PID.Target = base_speed;
            Motor2_PID.Target = base_speed;
            break;
    }
}

int main(void)
{
	/* 初始化 */
	LinePins_Init();
	OLED_Init();
	PWM_Init();
	Motor_Init();
	Encoder_Init();
	Timer_Init();
	Serial_Init();
	

	OLED_Clear();
	OLED_ShowString(0, 0, "Target:", OLED_8X16);
	OLED_ShowNum(64, 0, (int32_t)Motor1_PID.Target, 3, OLED_8X16);
	OLED_Update();

	char s[6];
	uint8_t mask;

	while (1)	
	{


		/* 读取 PA8..PA12 的掩码并显示 */
		mask = Read_A8_A12_mask();
		Read_A8_A12_str(s);

		// 根据掩码控制转向
		LineFollowControl(mask);

		
		OLED_ClearArea(0, 18, 128, 46);
		OLED_ShowString(0, 18, "Bits:", OLED_6X8);
		OLED_ShowString(36, 18, s, OLED_6X8);
		OLED_ShowString(0, 26, "Mask:", OLED_6X8);
		OLED_ShowBinNum(36, 26, mask, 5, OLED_6X8);
		OLED_ShowString(80, 26, "Val:", OLED_6X8);
		OLED_ShowNum(104, 26, mask, 3, OLED_6X8);
		
		// 显示速度信息
		OLED_ShowString(0, 40, "S1:", OLED_6X8);
		OLED_ShowSignedNum(18, 40, Speed1, 4, OLED_6X8);
		OLED_ShowString(60, 40, "S2:", OLED_6X8);
		OLED_ShowSignedNum(78, 40, Speed2, 4, OLED_6X8);

		OLED_Update();
		
		// 通过串口发送速度数据（添加起始标记便于识别）
		Serial_Printf("%d,%d,%.0f\r\n", Speed1, Speed2, Motor1_PID.Target);
		

	}
}

// 定时器1中断处理函数 - 用于速度环PID控制
void TIM1_UP_IRQHandler(void)
{
	static uint16_t Count = 0;

	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		Count++;
		if (Count >= 10)  // 每10ms执行一次
		{
			Count = 0;
			
			// 读取编码器速度
			Speed1 = Encoder_Get1();
			Speed2 = Encoder_Get2();
			
			// 电机1 PID控制
			Motor1_PID.Actual = -Speed1;
			PID_Update(&Motor1_PID);
			Motor_SetPWM1(-(int8_t)Motor1_PID.Out);
			
			// 电机2 PID控制
			Motor2_PID.Actual = Speed2;
			PID_Update(&Motor2_PID);
			Motor_SetPWM2(-(int8_t)Motor2_PID.Out);
		}
		
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}
}

// 根据掩码控制转向的函数
// mask: 5位掩码，bit4(PA8)=最左边, bit0(PA12)=最右边
// 1=没有线(白), 0=有线(黑)
// 控制PID速度环的目标值
