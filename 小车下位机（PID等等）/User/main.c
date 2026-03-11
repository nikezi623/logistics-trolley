#include "PHC_HeadFile.h"

// ==========================================
// 1. 系统状态与标志位
// ==========================================
uint8_t RunFlag = 0;			// 运行启停标志位 (0:停止, 1:运行)
uint8_t ConFlag = 0;			// 转向状态 (0:正常巡线, 1:右直角, 2:左直角, 3:全黑)
uint8_t all_black_flag = 0;		// 全黑任务状态机阶段
uint8_t send_item_flag = 0;		// 投放货物触发标志
uint8_t KeyNum = 0;				// 按键键值
uint16_t time_count = 0;		// 定时器计数
uint8_t have_turned_flag = 0;	// 已经转过直角弯的标志位
uint8_t line_end_fine_flag = 0; // 巡线结束标志位

// ==========================================
// 2. 视觉传感器参数
// ==========================================
uint8_t RxCmd = 2;				 // 串口接收的传感器状态 (默认2:丢线)
float Vision_Error = 0;			 // 当前视觉误差
float Last_Vision_Error = 0;	 // 上一次视觉误差 (用于求微分)
float Vision_Error_Integral = 0; // 视觉误差积分

// ==========================================
// 3. 电机与速度变量
// ==========================================
int16_t LeftPWM, RightPWM, AvgPWM, DifPWM;
double LeftSpeed, RightSpeed, AvgSpeed, DifSpeed;

// ==========================================
// 4. PID 与 巡线控制参数
// ==========================================
#define COMMONSPEED 4
#define BASE_SPEED 3.86 // 基础直道速度
#define MIN_SPEED 1.86	// 弯道最低速度

float VISION_KI = 0;	  // 视觉误差积分系数 (从0慢慢调)
float VISION_KD = 2.5;	  // 视觉误差微分系数
float TURN_GAIN = 0.5;	  // 转向环灵敏度增益
float SPEED_DROP_K = 5.0; // 弯道减速系数

PID_t SpeedPID = {
	.Kp = 4.00,
	.Ki = 0.66,
	.Kd = 0.00,
	.OutMax = 100,
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

		// --- 按键启动逻辑 ---
		if (KeyNum == 1)
		{
			if (RunFlag == 0)
			{
				PID_Init(&SpeedPID);
				PID_Init(&TurnPID);
				Vision_Error_Integral = 0;
				Last_Vision_Error = 0;
				Encoder_Get(1);
				Encoder_Get(2);

				ConFlag = 0;
				SpeedPID.Target = COMMONSPEED;
				DifPWM = 0;
				AvgPWM = 0;
				RxCmd = 2;
				RunFlag = 1;
				all_black_flag = 0;
				send_item_flag = 0;

				have_turned_flag = 0;
				line_end_fine_flag = 0;
			}
			else
			{
				RunFlag = 0;
			}
		}

		if (RunFlag)
			LED_ON();
		else
			LED_OFF();

		if (line_end_fine_flag == 0)
		{
			// --- 核心运动控制目标计算 ---
			if (ConFlag == 1) // 右直角转弯
			{
				SpeedPID.Target = 0;
				TurnPID.Target = 2;
			}
			else if (ConFlag == 2) // 左直角转弯
			{
				SpeedPID.Target = 0;
				TurnPID.Target = -2;
			}
			else // 正常寻路模式
			{
				if (RxCmd == 2) // 真·丢线
				{
					SpeedPID.Target = MIN_SPEED;
					TurnPID.Target = 0;
				}
				else if (RxCmd == 1) // 遇到全黑
				{
					ConFlag = 3;
				}
				else // 正常的 P+I+D 巡线逻辑
				{
					Vision_Error = Get_Vision_Error(RxCmd);

					// 误差累加（积分）及限幅
					Vision_Error_Integral += Vision_Error;
					if (Vision_Error_Integral > 100)
						Vision_Error_Integral = 100;
					if (Vision_Error_Integral < -100)
						Vision_Error_Integral = -100;

					// 计算 D (微分)
					float Vision_Derivative = Vision_Error - Last_Vision_Error;
					Last_Vision_Error = Vision_Error;

					// 计算速度目标值 (根据偏差减速)
					double expected_speed = BASE_SPEED - (SPEED_DROP_K * fabs(Vision_Error));
					if (expected_speed < MIN_SPEED)
						expected_speed = MIN_SPEED;
					SpeedPID.Target = expected_speed;

					// 计算转向环目标值
					TurnPID.Target = (Vision_Error * TURN_GAIN) +
									 (Vision_Error_Integral * VISION_KI) +
									 (Vision_Derivative * VISION_KD);
				}
			}
		}
		else
		{
			SpeedPID.Target = MIN_SPEED;
			TurnPID.Target = 0;
		}

		// --- OLED 屏幕显示 ---
		OLED_Clear();
		OLED_Printf(0, 0, OLED_6X8, "Speed");
		OLED_Printf(0, 8, OLED_6X8, "%05.2f", SpeedPID.Kp);
		OLED_Printf(0, 16, OLED_6X8, "%05.2f", SpeedPID.Ki);
		OLED_Printf(0, 24, OLED_6X8, "%05.2f", SpeedPID.Kd);
		OLED_Printf(0, 32, OLED_6X8, "%+05.2f", SpeedPID.Target);
		OLED_Printf(0, 40, OLED_6X8, "%+05.2f", AvgSpeed);
		OLED_Printf(0, 48, OLED_6X8, "%+05.2f", SpeedPID.Out);

		OLED_Printf(38, 0, OLED_6X8, "Turn");
		OLED_Printf(38, 8, OLED_6X8, "%05.2f", TurnPID.Kp);
		OLED_Printf(38, 16, OLED_6X8, "%05.2f", TurnPID.Ki);
		OLED_Printf(38, 24, OLED_6X8, "%05.2f", TurnPID.Kd);
		OLED_Printf(38, 32, OLED_6X8, "%+05.2f", TurnPID.Target);
		OLED_Printf(38, 40, OLED_6X8, "%+05.2f", DifSpeed);
		OLED_Printf(38, 48, OLED_6X8, "%+05.2f", TurnPID.Out);
		OLED_Printf(38, 56, OLED_6X8, "%+05.2f", TURN_GAIN);

		OLED_Printf(74, 40, OLED_6X8, "%+05.2f", VISION_KD);
		OLED_Printf(74, 48, OLED_6X8, "%+05.2f", TURN_GAIN);
		OLED_Printf(74, 56, OLED_6X8, "%+05.2f", SPEED_DROP_K);
		OLED_Printf(76, 0, OLED_8X16, "Rxd:%d", RxCmd);
		OLED_Update();

		// --- 蓝牙数据打印与接收解析 ---
		BlueSerial_Printf("[plot,%f, %f, %f, %f]", SpeedPID.Actual, SpeedPID.Target, TurnPID.Actual, TurnPID.Target);
		BlueSerial_Printf("SpA:%f SpT:%f TuA:%f TuT:%f", SpeedPID.Actual, SpeedPID.Target, TurnPID.Actual, TurnPID.Target);

		if (BlueSerial_RxFlag == 1)
		{
			char *Tag = strtok(BlueSerial_RxPacket, ",");
			if (strcmp(Tag, "slider") == 0)
			{
				char *Name = strtok(NULL, ",");
				char *Value = strtok(NULL, ",");

				if (strcmp(Name, "SpeedKp") == 0)
					SpeedPID.Kp = atof(Value);
				else if (strcmp(Name, "SpeedKi") == 0)
					SpeedPID.Ki = atof(Value);
				else if (strcmp(Name, "TurnKp") == 0)
					TurnPID.Kp = atof(Value);
				else if (strcmp(Name, "TurnKi") == 0)
					TurnPID.Ki = atof(Value);
				else if (strcmp(Name, "TurnKd") == 0)
					TurnPID.Kd = atof(Value);
				else if (strcmp(Name, "VISION_KD") == 0)
					VISION_KD = atof(Value);
				else if (strcmp(Name, "TURN_GAIN") == 0)
					TURN_GAIN = atof(Value);
				else if (strcmp(Name, "SPEED_DROP_K") == 0)
					SPEED_DROP_K = atof(Value);
			}
			else if (strcmp(Tag, "joystick") == 0)
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

void TIM1_UP_IRQHandler(void) // 1ms进入一次
{
	static uint16_t CountSpeedTurn = 0, CountControl = 0, DelayCount = 0;

	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);

		CountSpeedTurn++;
		CountControl++;
		Key_Tick();

		if (send_item_flag == 1)
		{
			DelayCount ++;

		}

		// --- 1. 速度环与转向环控制 (50ms 周期) ---
		if (CountSpeedTurn >= 50)
		{
			CountSpeedTurn = 0;

			LeftSpeed = Encoder_Get(1) / 44.0 / 0.05 / 9.27666;
			RightSpeed = Encoder_Get(2) / 44.0 / 0.05 / 9.27666;
			AvgSpeed = (LeftSpeed + RightSpeed) / 2.0;
			DifSpeed = LeftSpeed - RightSpeed;

			if (RunFlag)
			{
				SpeedPID.Actual = AvgSpeed;
				PID_Update(&SpeedPID);
				AvgPWM = SpeedPID.Out;

				TurnPID.Actual = DifSpeed;
				PID_Update(&TurnPID);
				DifPWM = TurnPID.Out;

				LeftPWM = AvgPWM + DifPWM / 2;
				RightPWM = AvgPWM - DifPWM / 2;

				// PWM 限幅
				if (LeftPWM >= 100)
					LeftPWM = 100;
				else if (LeftPWM <= -100)
					LeftPWM = -100;
				if (RightPWM >= 100)
					RightPWM = 100;
				else if (RightPWM <= -100)
					RightPWM = -100;

				Motor_SetPWM(1, LeftPWM);
				Motor_SetPWM(2, RightPWM);
			}
			else
			{
				Motor_SetPWM(1, 0);
				Motor_SetPWM(2, 0);
			}
		}

		// --- 2. 视觉指令解析与状态机 (1ms 周期) ---
		if (CountControl >= 1)
		{
			CountControl = 0;

			if (Serial_GetRxFlag() == 1)
			{
				RxCmd = Serial_GetRxData();

				// 2.1 全黑任务状态机
				if (RxCmd == 1 && all_black_flag == 0)
					all_black_flag = 1;
				else if (RxCmd == 1 && all_black_flag == 1)
					SpeedPID.Target = BASE_SPEED;
				else if (RxCmd != 1 && (all_black_flag == 1))
					all_black_flag = 2;
				else if ((RxCmd == 1 || line_end_fine_flag == 1) && (all_black_flag == 2))
				{
					send_item_flag = 1;
					PID_Init(&SpeedPID);
					PID_Init(&TurnPID);
					
					if (DelayCount >= 2000) // 预留投放逻辑
					{
						send_item_flag = 0;
						all_black_flag = 3;
					}
				}
				else if (RxCmd == 1 && all_black_flag == 3)
					SpeedPID.Target = MIN_SPEED;
				else if (RxCmd != 1 && all_black_flag == 3)
					all_black_flag = 4;
				else if (RxCmd == 1 && all_black_flag == 4)
					RunFlag = 0;

				// 2.2 判断是否该退出直角转弯 (重新居中)
				if (ConFlag != 0 && (RxCmd == 31 || RxCmd == 32 || RxCmd == 33))
				{
					ConFlag = 0;
					Vision_Error_Integral = 0;
					Last_Vision_Error = 0;
					PID_Init(&SpeedPID);
					PID_Init(&TurnPID);
				}

				// 2.3 捕捉进入直角转弯信号
				if (ConFlag == 0 && have_turned_flag == 0)
				{
					if (RxCmd == 8)
					{
						ConFlag = 1; // 右直角
						have_turned_flag = 1;
						PID_Init(&SpeedPID);
						PID_Init(&TurnPID);
					}
					else if (RxCmd == 9)
					{
						ConFlag = 2; // 左直角
						have_turned_flag = 1;
						PID_Init(&SpeedPID);
						PID_Init(&TurnPID);
					}
				}
				else if (ConFlag == 0 && (RxCmd == 8 || RxCmd == 9 || RxCmd == 1) && have_turned_flag == 1)
				{
					line_end_fine_flag = 1;
				}
			}
		}
		time_count = TIM_GetCounter(TIM1);
	}
}