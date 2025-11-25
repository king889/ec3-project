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
int32_t Location1, Location2;

// 速度调整相关变量
uint32_t SpeedAdjustTimer = 0;  // 速度调整计时器
uint8_t SpeedIndex = 0;         // 当前速度索引
const float SpeedTargets[] = {20.0f, 40.0f, 60.0f, 80.0f, 100.0f, 80.0f, 60.0f, 40.0f}; // 速度目标数组
const uint8_t SpeedCount = sizeof(SpeedTargets) / sizeof(SpeedTargets[0]); // 速度目标数量

// 电机1 速度环PID参数
PID_t Motor1_PID = {
    .Target = 30,
    .Actual = 0,
    .Out = 0,
    .Kp = 1,
    .Ki = 0.1, 
    .Kd = 0.2,
    .Error0 = 0,
    .Error1 = 0,
    .Error2 = 0,
    .OutMax = 130,
    .DeadZone = 0,
    .OutMin = -130
};

// 电机2 速度环PID参数
PID_t Motor2_PID = {
    .Target = 30,
    .Actual = 0,
    .Out = 0,
    .Kp = 1,
    .Ki = 0.1, 
    .Kd = 0.2,
    .Error0 = 0,
    .Error1 = 0,
    .Error2 = 0,
    .OutMax = 130,
    .DeadZone = 0,
    .OutMin = -130
};

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
	
	/* 发送测试消息 */
	Delay_ms(100);
	Serial_Printf("USART Test: PB10/PB11 OK!\r\n");

	/* 显示标题和初始目标速度 */
	OLED_Clear();
	OLED_ShowString(0, 0, "Target:", OLED_8X16);
	OLED_ShowNum(64, 0, (int32_t)Motor1_PID.Target, 3, OLED_8X16);
	OLED_Update();

	char s[6];
	uint8_t mask;

	while (1)	
	{
		/* 速度调整逻辑 - 每5秒调整一次 */
		SpeedAdjustTimer += 100;  // 每次循环增加100ms
		if (SpeedAdjustTimer >= 5000)  // 5秒
		{
			SpeedAdjustTimer = 0;
			
			// 更新速度目标
			Motor1_PID.Target = SpeedTargets[SpeedIndex];
			Motor2_PID.Target = SpeedTargets[SpeedIndex];
			
			// 更新索引，循环使用
			SpeedIndex = (SpeedIndex + 1) % SpeedCount;
			
			// 在OLED上显示新的目标速度
			OLED_ClearArea(0, 0, 128, 16);
			OLED_ShowString(0, 0, "Target:", OLED_8X16);
			OLED_ShowNum(64, 0, (int32_t)Motor1_PID.Target, 3, OLED_8X16);
		}

		/* 读取 PA8..PA12 的掩码并显示 */
		mask = Read_A8_A12_mask();
		Read_A8_A12_str(s);

		/* 清除显示区域并写入最新状态 */
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
		
		Delay_ms(100);
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
			Location1 += Speed1;
			Location2 += Speed2;
			
			// 电机1 PID控制
			Motor1_PID.Actual = Speed1;
			PID_Update(&Motor1_PID);
			Motor_SetPWM1((int8_t)Motor1_PID.Out);
			
			// 电机2 PID控制
			Motor2_PID.Actual = Speed2;
			PID_Update(&Motor2_PID);
			Motor_SetPWM2(-(int8_t)Motor2_PID.Out);
		}
		
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}
}

