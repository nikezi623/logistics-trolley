#include "PHC_HeadFile.h"

#define COMMONSPEED 4

int16_t Ax, Ay, Az, Gx, Gy, Gz;
uint16_t TimerErrorFlag;
uint16_t time_count;
double AngleAcc;
double AngleGyro;//角速度积分
double Angle;//互补滤波后的稳定角度
uint8_t KeyNum, RunFlag;//runflag表示运行启停
int16_t LeftPWM, RightPWM, AvgPWM, DifPWM;
double LeftSpeed, RightSpeed, AvgSpeed, DifSpeed;
uint8_t RxCmd, LastCmd;
float Vision_Error_Integral = 0;
float VISION_KI = 0.05; // 积分系数，需要从很小的值慢慢调大
//float TURN_GAIN = 0.3; // 转向灵敏度 (将视觉误差转化为 TurnTarget 的系数)

// --- 巡线参数配置 ---
#define BASE_SPEED      2.0   // 基础直道速度 (可以比原来设高一点)
#define MIN_SPEED       0.5   // 弯道最低速度
#define TURN_GAIN       0.55   // 转向灵敏度 (将视觉误差转化为 TurnTarget 的系数)
#define SPEED_DROP_K    1.3   // 减速系数 (误差越大，减速越明显)

// --- 视觉状态映射 ---
// 将 RxCmd 映射为偏差值：负数左偏，正数右偏，0居中
// 3:中, 4:微右, 5:重右, 6:微左, 7:重左, 8:右直角, 9:左直角
float Vision_Error = 0;

uint8_t ConFlag = 0;

PID_t SpeedPID = {
	.Kp = 4.00,
	.Ki = 0.66,
	.Kd = 0.00,

	.OutMax = 100,  //速度环的输出是角度环的输入，一般角度极限给+-20
	.OutMin = -100,

	.ErrorIntMax = 150,
	.ErrorIntMin = -150,
};

PID_t TurnPID = {
	.Kp = 6.00,
	.Ki = 2.00,
	.Kd = 0.00,

	.OutMax = 100,  
	.OutMin = -100,

	.ErrorIntMax = 20,
	.ErrorIntMin = -20,
};

int main(void)
{
	Init_All();	 
	while (1)
	{
		KeyNum = Key_GetNum();

		if (KeyNum == 1)//按键1控制启停
		{
			if (RunFlag == 0)
			{
				PID_Init(&SpeedPID);//清零PID，防止启动前较大的积分累计
				PID_Init(&TurnPID);//清零PID，防止启动前较大的积分累计
				SpeedPID.Target = COMMONSPEED;
				DifPWM = 0; 
				AvgPWM = 0;
				RxCmd = 2;
				RunFlag = 1;
			}
			else
			{
				RunFlag = 0;
			}
		}

		if (RunFlag) LED_ON(); else LED_OFF();//灯亮表示pid工作

		if (RxCmd == 2) //丢线
		{
			SpeedPID.Target = MIN_SPEED;
			TurnPID.Target = 0; 
		}
		else if (RxCmd == 8) 
		{
			ConFlag = 1;
		}
		else if (RxCmd == 9) 
		{
			ConFlag = 2;
		}
		else 
		{
			LastCmd = RxCmd; 

			Vision_Error = Get_Vision_Error(RxCmd);

			// 误差累加（积分）
			Vision_Error_Integral += Vision_Error;
			// 积分限幅，防止偏离过久导致积分爆表（俗称积分饱和）
			if (Vision_Error_Integral > 100) Vision_Error_Integral = 100;
			if (Vision_Error_Integral < -100) Vision_Error_Integral = -100;
			
			double expected_speed = BASE_SPEED - (SPEED_DROP_K * fabs(Vision_Error));
			if (expected_speed < MIN_SPEED) expected_speed = MIN_SPEED;
			SpeedPID.Target = expected_speed;
			TurnPID.Target = Vision_Error * TURN_GAIN;
		}		

		OLED_Clear();		
		//OLED速度环参数显示
		OLED_Printf(0, 0, OLED_6X8, "Speed");
		OLED_Printf(0, 8, OLED_6X8, "%05.2f", SpeedPID.Kp);
		OLED_Printf(0, 16, OLED_6X8, "%05.2f", SpeedPID.Ki);
		OLED_Printf(0, 24, OLED_6X8, "%05.2f", SpeedPID.Kd);
		OLED_Printf(0, 32, OLED_6X8, "%+05.2f", SpeedPID.Target);
		OLED_Printf(0, 40, OLED_6X8, "%+05.2f", AvgSpeed);
		OLED_Printf(0, 48, OLED_6X8, "%+05.2f", SpeedPID.Out);
		
		//OLED转向环参数显示
		OLED_Printf(38, 0, OLED_6X8, "Turn");
		OLED_Printf(38, 8, OLED_6X8, "%05.2f", TurnPID.Kp);
		OLED_Printf(38, 16, OLED_6X8, "%05.2f", TurnPID.Ki);
		OLED_Printf(38, 24, OLED_6X8, "%05.2f", TurnPID.Kd);
		OLED_Printf(38, 32, OLED_6X8, "%+05.2f", TurnPID.Target);
		OLED_Printf(38, 40, OLED_6X8, "%+05.2f", DifSpeed);
		OLED_Printf(38, 48, OLED_6X8, "%+05.2f", TurnPID.Out);
		OLED_Printf(38, 56, OLED_6X8, "%+05.2f", TURN_GAIN);

		//输出编译
		// OLED_Printf(56, 56, OLED_6X8, "Offset:%02.0f", AnglePID.Offset);

		OLED_Update();		

		//蓝牙串口打印
		BlueSerial_Printf("[plot,%f, %f, %f, %f]", SpeedPID.Actual, SpeedPID.Target, TurnPID.Actual, TurnPID.Target);
		BlueSerial_Printf("SpA:%f SpT:%f TuA:%f TuT:%f", SpeedPID.Actual, SpeedPID.Target, TurnPID.Actual, TurnPID.Target);

		//蓝牙控制
		if (BlueSerial_RxFlag == 1)
		{
			char *Tag = strtok(BlueSerial_RxPacket, ",");
			if (strcmp(Tag, "key") == 0)
			{
				char *Name = strtok(NULL, ",");
				char *Action = strtok(NULL, ",");
			}
			else if (strcmp(Tag, "slider") == 0) //滑块
			{
				char *Name = strtok(NULL, ",");
				char *Value = strtok(NULL, ",");
				
				//速度环PID调参
				if (strcmp(Name, "SpeedKp") == 0)
				{
					SpeedPID.Kp = atof(Value);
				}
				else if (strcmp(Name, "SpeedKi") == 0)
				{
					SpeedPID.Ki = atof(Value);
				}
//				else if (strcmp(Name, "SpeedKd") == 0)
//				{
//					SpeedPID.Kd = atof(Value);
//				}
				
				//转向环PID调参
				else if (strcmp(Name, "TurnKp") == 0)
				{
					TurnPID.Kp = atof(Value);
				}
				else if (strcmp(Name, "TurnKi") == 0)
				{
					TurnPID.Ki = atof(Value);
				}
				else if (strcmp(Name, "TurnKd") == 0)
				{
					TurnPID.Kd = atof(Value);
				}

				//转向灵敏度调参
//				else if (strcmp(Name, "TurnL") == 0)
//				{
//					TURN_GAIN = atof(Value);
//				}
				//输出偏移
				// else if (strcmp(Name, "Offset") == 0)
				// {
				// 	AnglePID.Offset = atof(Value);
				// }
			}
				if (strcmp(Tag, "joystick") == 0) //遥杆
				{
					int8_t LH = atoi(strtok(NULL, ","));
					int8_t LV = atoi(strtok(NULL, ","));
					int8_t RH = atoi(strtok(NULL, ","));
					int8_t RV = atoi(strtok(NULL, ","));
			
					SpeedPID.Target = LV / 25.0;
					TurnPID.Target = RH / 25.0;
				}
			
			BlueSerial_RxFlag = 0;
		}
	}
}

void TIM1_UP_IRQHandler(void) //1ms进入一次
{
	static uint8_t CountSpeed = 0, CountTurn = 0, CountControl = 0;

	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		//清除中断标志位
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);

		//计数器累加
		// CountAngle ++;
		CountSpeed ++;
		CountTurn ++;
		CountControl ++;

		//按键
		Key_Tick();
	
		//速度环(定时读取编码器并进行外环PID调控)
		if (CountSpeed >= 35)
		{
			CountSpeed = 0;

			LeftSpeed = Encoder_Get(1) / 44.0 / 0.05 / 9.27666;//左电机转速，单位：转/秒
			RightSpeed = Encoder_Get(2) / 44.0 / 0.05 / 9.27666;//右电机转速，单位：转/秒

			AvgSpeed = (LeftSpeed + RightSpeed) / 2.0;
			DifSpeed = LeftSpeed - RightSpeed;
			if (RunFlag)
			{
				SpeedPID.Actual = AvgSpeed;
				PID_Update(&SpeedPID);
				AvgPWM = SpeedPID.Out;

				LeftPWM = AvgPWM + DifPWM / 2;
				RightPWM = AvgPWM - DifPWM / 2;

				if (LeftPWM >= 100) LeftPWM = 100; else if (LeftPWM <= -100) LeftPWM = -100;
				if (RightPWM >= 100) RightPWM = 100; else if (RightPWM <= -100) RightPWM = -100;

				Motor_SetPWM(1, LeftPWM);
				Motor_SetPWM(2, RightPWM);
			}
			else
			{
				Motor_SetPWM(1, 0);
				Motor_SetPWM(2, 0);
			}
		}
	
		//转向环(外环PID调控)
		if (CountTurn >= 35)
		{
			CountTurn = 0;
			if (RunFlag)
			{
				TurnPID.Actual = DifSpeed;
				PID_Update(&TurnPID);
				DifPWM = TurnPID.Out;
			}
		}
	
        // 处理下位机通信信息 (提高频率，每 10ms 或 20ms 处理一次)
        if (CountControl >= 1) 
        {
            CountControl = 0;
			//接收下位机通信信息
			if (Serial_GetRxFlag() == 1)
			{
				RxCmd = Serial_GetRxData();
			}
        }
		 time_count = TIM_GetCounter(TIM1);
	}
}

