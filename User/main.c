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

// 巡线传感器变量 (1=黑线, 0=白线)
uint8_t X1, X2, X3, X4, X5;  // X1最右边, X5最左边

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
    .Kp = 0.5,
    .Ki = 0.02, 
    .Kd = 0.3,
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
    .Kp = 0.5,
    .Ki = 0.02, 
    .Kd = 0.3,
    .Error0 = 0,
    .Error1 = 0,
    .Error2 = 0,
    .OutMax = 130,
    .DeadZone = 0,
    .OutMin = -130
};

// 读取巡线传感器
void Read_Sensors(void)
{
    // 读取GPIO状态 (1=高电平=没线, 0=低电平=有线)
    // 但巡线逻辑中通常1表示黑线，0表示白线，所以取反
    X1 = !GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12);  // 最右边
    X2 = !GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_11);
    X3 = !GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_10);  // 中间
    X4 = !GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_9);
    X5 = !GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8);   // 最左边
}

// 设置小车速度 (使用PID速度环控制)
void Set_Car_Speed(int16_t left_speed, int16_t right_speed)
{
    // 设置PID目标速度值
    Motor1_PID.Target = (float)left_speed;
    Motor2_PID.Target = (float)right_speed;
}

// 巡线任务
void Trace_task(void) 
{
    Read_Sensors();
    
    if(X1==0 && X3==0) {
        Set_Car_Speed(30, 30);    // 直行
    }
    else if(X1==1 && X3==0) {
        Set_Car_Speed(80, -70);       // 左转 (左轮停，右轮转)
    }
    else if(X1==0 && X3==1) {
        Set_Car_Speed(-70, 80);       // 右转 (左轮转，右轮停)
    }
    else if(X2==0 && X1==1 && X3==1 && X4==1) {
        Set_Car_Speed(120, -90);       // 急左转
    }
    else if(X2==1 && X1==1 && X3==1 && X4==0) {
        Set_Car_Speed(-90, 120);       // 急右转
    }
    else {
        Set_Car_Speed(30, 30);    // 默认直行
    }
}
// {
//     if(X2==1 && X1 == 1 && X4 ==1 && X5==1 )
//     {
//         Set_Car_Speed(40,40);

//     }
    
//     else if(X1==1 && X2==0 && X4==1 && X5==1)//小拐
//     {
//         Set_Car_Speed(50,0);
//     }
    
//     else if(X1==1 && X2==1 && X4==0 && X5==1 )//小拐
//     {
//         Set_Car_Speed(0,50);
//     }
    
//     else if(X1==0 && X2==1 && X4==1 && X5==1 )//大拐
//     {
//         Set_Car_Speed(0,100);
//     }
    
//     else if(X1==1 && X2==1 && X4==1 && X5==0 )//大拐
//     {
//         Set_Car_Speed(100,0);
//     }
    
//     else if(X1==0 && X2==0 && X4==0 && X5==0 )//十字
//     {
//         Set_Car_Speed(40,40);
//     }

//     else if((X1==1 && X2==0  && X4==0 && X5==0 )||(X1==1 && X2==1 && X4==0 && X5==0 ))
//     {
//         Set_Car_Speed(0,100);
//     }
    
//     else if((X1==0 && X2==0 && X4==0 && X5==1 )||(X1==0 && X2==0 && X4==1 && X5==1 ))
//     {
//         Set_Car_Speed(100,0);
//     }

// }

// void Trace_task(void) 
// {
//     Read_Sensors();
    
//     // 1. 完全在线上的情况（仅中间传感器检测到黑线）
//     if(X1==0 && X2==0 && X3==1 && X4==0 && X5==0) {
//         Set_Car_Speed(60, 60);    // 直行
//     }
//     // 2. 轻微偏右（中右传感器检测到线）
//     else if(X1==0 && X2==0 && X3==0 && X4==1 && X5==0) {
//         Set_Car_Speed(70, 30);    // 左转修正（左轮快，右轮慢）
//     }
//     // 3. 轻微偏左（中左传感器检测到线）
//     else if(X1==0 && X2==1 && X3==0 && X4==0 && X5==0) {
//         Set_Car_Speed(30, 70);    // 右转修正（左轮慢，右轮快）
//     }
//     // 4. 明显偏右（最右传感器检测到线）
//     else if(X1==0 && X2==0 && X3==0 && X4==0 && X5==1) {
//         Set_Car_Speed(80, 0);     // 左转（右轮停，左轮转）
//     }
//     // 5. 明显偏左（最左传感器检测到线）
//     else if(X1==1 && X2==0 && X3==0 && X4==0 && X5==0) {
//         Set_Car_Speed(0, 80);     // 右转（左轮停，右轮转）
//     }
//     // 6. 严重偏右（中右+最右检测到线）
//     else if(X1==0 && X2==0 && X3==0 && X4==1 && X5==1) {
//         Set_Car_Speed(100, 0);    // 急左转（右轮停，左轮高速）
//     }
//     // 7. 严重偏左（中左+最左检测到线）
//     else if(X1==1 && X2==1 && X3==0 && X4==0 && X5==0) {
//         Set_Car_Speed(0, 100);    // 急右转（左轮停，右轮高速）
//     }
//     // 8. 十字路口或完全丢失线（所有传感器都看不到线）
//     else if(X1==0 && X2==0 && X3==0 && X4==0 && X5==0) {
//         Set_Car_Speed(50, 50);    // 保持直行
//     }
//     // 9. T字路口或特殊情况（中间三个传感器看到线）
//     else if(X1==0 && X2==1 && X3==1 && X4==1 && X5==0) {
//         Set_Car_Speed(60, 60);    // 直行通过T字路口
//     }
//     // 10. 全黑线（所有传感器都看到线）
//     else if(X1==1 && X2==1 && X3==1 && X4==1 && X5==1) {
//         Set_Car_Speed(50, 50);    // 直行
//     }
//     // 11. 极端偏右（多个右侧传感器检测到线）
//     else if((X1==0 && X2==0 && X3==1 && X4==1 && X5==1) || 
//             (X1==0 && X2==0 && X3==0 && X4==1 && X5==1)) {
//         Set_Car_Speed(120, 0);    // 极急左转
//     }
//     // 12. 极端偏左（多个左侧传感器检测到线）
//     else if((X1==1 && X2==1 && X3==1 && X4==0 && X5==0) || 
//             (X1==1 && X2==1 && X3==0 && X4==0 && X5==0)) {
//         Set_Car_Speed(0, 120);    // 极急右转
//     }
//     // 13. 默认情况
//     else {
//         Set_Car_Speed(40, 40);    // 保守直行
//     }
// }


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
	OLED_ShowString(0, 0, "Line Follow", OLED_8X16);
	OLED_Update();

	char s[6];
	uint8_t mask;

	while (1)	
	{


		/* 读取 PA8..PA12 的掩码并显示 */
		mask = Read_A8_A12_mask();
		Read_A8_A12_str(s);

		// 执行巡线任务
		Trace_task();

		
		OLED_ClearArea(0, 18, 128, 46);
		OLED_ShowString(0, 18, "Sensors:", OLED_6X8);
		OLED_ShowString(48, 18, "X1 X2 X3 X4 X5", OLED_6X8);
		OLED_ShowNum(48, 26, X1, 1, OLED_6X8);
		OLED_ShowNum(60, 26, X2, 1, OLED_6X8);
		OLED_ShowNum(72, 26, X3, 1, OLED_6X8);
		OLED_ShowNum(84, 26, X4, 1, OLED_6X8);
		OLED_ShowNum(96, 26, X5, 1, OLED_6X8);
		OLED_ShowString(0, 34, "Mask:", OLED_6X8);
		OLED_ShowBinNum(36, 34, mask, 5, OLED_6X8);
		OLED_ShowString(80, 34, "Val:", OLED_6X8);
		OLED_ShowNum(104, 34, mask, 3, OLED_6X8);
		
		// 显示速度信息
		OLED_ShowString(0, 40, "S1:", OLED_6X8);
		OLED_ShowSignedNum(18, 40, Speed1, 4, OLED_6X8);
		OLED_ShowString(60, 40, "S2:", OLED_6X8);
		OLED_ShowSignedNum(78, 40, Speed2, 4, OLED_6X8);

		OLED_Update();
		
		// 通过串口发送传感器数据
		Serial_Printf("X:%d%d%d%d%d M:%d\r\n", X5, X4, X3, X2, X1, mask);
		

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
