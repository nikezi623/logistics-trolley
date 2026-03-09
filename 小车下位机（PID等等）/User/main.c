#include "PHC_HeadFile.h"

#define COMMONSPEED 4

int16_t Ax, Ay, Az, Gx, Gy, Gz;
uint16_t TimerErrorFlag;
uint16_t time_count;
double AngleAcc;
double AngleGyro;		 // 角速度积分
double Angle;			 // 互补滤波后的稳定角度
uint8_t KeyNum, RunFlag; // runflag表示运行启停
int16_t LeftPWM, RightPWM, AvgPWM, DifPWM;
double LeftSpeed, RightSpeed, AvgSpeed, DifSpeed;
uint8_t RxCmd, LastCmd;
float Vision_Error_Integral = 0;
float Last_Vision_Error = 0; // 👇 新增：记录上一次的误差
float VISION_KI = 0;		 // 积分系数，需要从很小的值慢慢调大
float VISION_KD = 2.5;		 // 👇 新增：微分系数 (一般比 P 小一点)
float TURN_GAIN = 0.5;
float SPEED_DROP_K = 5.0;

// --- 巡线参数配置 ---
#define BASE_SPEED 1.86 // 基础直道速度 (可以比原来设高一点)
#define MIN_SPEED 1.0  // 弯道最低速度
// #define TURN_GAIN       0.5   // 转向灵敏度 (将视觉误差转化为 TurnTarget 的系数)
// #define SPEED_DROP_K    1.0   // 减速系数 (误差越大，减速越明显)

// --- 视觉状态映射 ---
// 将 RxCmd 映射为偏差值：负数左偏，正数右偏，0居中
// 3:中, 4:微右, 5:重右, 6:微左, 7:重左, 8:右直角, 9:左直角
float Vision_Error = 0;

uint8_t ConFlag = 0; // 0: 正常直线寻路, 1: 正在右直角转, 2: 正在左直角转

PID_t SpeedPID = {
	.Kp = 4.00,
	.Ki = 0.66,
	.Kd = 0.00,

	.OutMax = 100, // 速度环的输出是角度环的输入，一般角度极限给+-20
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

		if (KeyNum == 1) // 按键1控制启停
		{
			if (RunFlag == 0)
			{
				PID_Init(&SpeedPID); // 清零PID，防止启动前较大的积分累计
				PID_Init(&TurnPID);	 // 清零PID，防止启动前较大的积分累计
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

		if (RunFlag)
			LED_ON();
		else
			LED_OFF(); // 灯亮表示pid工作

		// --- 2. 根据当前状态，决定车子怎么动 ---
		if (ConFlag == 1)
		{
			// 正在右转
			SpeedPID.Target = 0;
			TurnPID.Target = 2;
		}
		else if (ConFlag == 2)
		{
			// 正在左转
			SpeedPID.Target = 0;
			TurnPID.Target = -2;
		}
		else
		{
			// 【ConFlag == 0 时，执行正常寻路逻辑】

			// 只有在没转弯的时候，才去接收新的直角指令或处理丢线
			if (RxCmd == 8)
			{
				ConFlag = 1; // 触发右转 (下个循环立马开始转)
			}
			else if (RxCmd == 9)
			{
				ConFlag = 2; // 触发左转 (下个循环立马开始转)
			}
			else if (RxCmd == 2) // 真·丢线
			{
				SpeedPID.Target = MIN_SPEED;
				TurnPID.Target = 0;
			}
			else
			{
				// 正常的 P+I+D 巡线逻辑
				Vision_Error = Get_Vision_Error(RxCmd);

				// 误差累加（积分）
				Vision_Error_Integral += Vision_Error;
				if (Vision_Error_Integral > 100)
					Vision_Error_Integral = 100;
				if (Vision_Error_Integral < -100)
					Vision_Error_Integral = -100;

				// 计算 D (微分)
				float Vision_Derivative = Vision_Error - Last_Vision_Error;
				Last_Vision_Error = Vision_Error;

				// 速度目标值
				double expected_speed = BASE_SPEED - (SPEED_DROP_K * fabs(Vision_Error));
				if (expected_speed < MIN_SPEED)
					expected_speed = MIN_SPEED;
				SpeedPID.Target = expected_speed;

				// 转向环目标值
				TurnPID.Target = (Vision_Error * TURN_GAIN) +
								 (Vision_Error_Integral * VISION_KI) +
								 (Vision_Derivative * VISION_KD);
			}
		}

		OLED_Clear();
		// OLED速度环参数显示
		OLED_Printf(0, 0, OLED_6X8, "Speed");
		OLED_Printf(0, 8, OLED_6X8, "%05.2f", SpeedPID.Kp);
		OLED_Printf(0, 16, OLED_6X8, "%05.2f", SpeedPID.Ki);
		OLED_Printf(0, 24, OLED_6X8, "%05.2f", SpeedPID.Kd);
		OLED_Printf(0, 32, OLED_6X8, "%+05.2f", SpeedPID.Target);
		OLED_Printf(0, 40, OLED_6X8, "%+05.2f", AvgSpeed);
		OLED_Printf(0, 48, OLED_6X8, "%+05.2f", SpeedPID.Out);

		// OLED转向环参数显示
		OLED_Printf(38, 0, OLED_6X8, "Turn");
		OLED_Printf(38, 8, OLED_6X8, "%05.2f", TurnPID.Kp);
		OLED_Printf(38, 16, OLED_6X8, "%05.2f", TurnPID.Ki);
		OLED_Printf(38, 24, OLED_6X8, "%05.2f", TurnPID.Kd);
		OLED_Printf(38, 32, OLED_6X8, "%+05.2f", TurnPID.Target);
		OLED_Printf(38, 40, OLED_6X8, "%+05.2f", DifSpeed);
		OLED_Printf(38, 48, OLED_6X8, "%+05.2f", TurnPID.Out);
		OLED_Printf(38, 56, OLED_6X8, "%+05.2f", TURN_GAIN);

		// OLED其他参数显示
		OLED_Printf(74, 40, OLED_6X8, "%+05.2f", VISION_KD);
		OLED_Printf(74, 48, OLED_6X8, "%+05.2f", TURN_GAIN);
		OLED_Printf(74, 56, OLED_6X8, "%+05.2f", SPEED_DROP_K);

		// 输出编译
		//  OLED_Printf(56, 56, OLED_6X8, "Offset:%02.0f", AnglePID.Offset);

		OLED_Update();

		// 蓝牙串口打印
		BlueSerial_Printf("[plot,%f, %f, %f, %f]", SpeedPID.Actual, SpeedPID.Target, TurnPID.Actual, TurnPID.Target);
		BlueSerial_Printf("SpA:%f SpT:%f TuA:%f TuT:%f", SpeedPID.Actual, SpeedPID.Target, TurnPID.Actual, TurnPID.Target);

		// 蓝牙控制
		if (BlueSerial_RxFlag == 1)
		{
			char *Tag = strtok(BlueSerial_RxPacket, ",");
			if (strcmp(Tag, "key") == 0)
			{
				char *Name = strtok(NULL, ",");
				char *Action = strtok(NULL, ",");
			}
			else if (strcmp(Tag, "slider") == 0) // 滑块
			{
				char *Name = strtok(NULL, ",");
				char *Value = strtok(NULL, ",");

				// 速度环PID调参
				if (strcmp(Name, "SpeedKp") == 0)
				{
					SpeedPID.Kp = atof(Value);
				}
				else if (strcmp(Name, "SpeedKi") == 0)
				{
					SpeedPID.Ki = atof(Value);
				}

				// 转向环PID调参
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
				else if (strcmp(Name, "TurnKd") == 0)
				{
					TurnPID.Kd = atof(Value);
				}

				// 其他参数
				else if (strcmp(Name, "VISION_KD") == 0)
				{
					VISION_KD = atof(Value);
				}
				else if (strcmp(Name, "TURN_GAIN") == 0)
				{
					TURN_GAIN = atof(Value);
				}
				else if (strcmp(Name, "SPEED_DROP_K") == 0)
				{
					SPEED_DROP_K = atof(Value);
				}
			}
			if (strcmp(Tag, "joystick") == 0) // 遥杆
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
	static uint8_t CountSpeedTurn = 0, CountControl = 0;

	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		// 清除中断标志位
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);

		// 计数器累加
		CountSpeedTurn++;
		CountControl++;

		// 按键
		Key_Tick();

		// 速度环&转向环(定时读取编码器并进行外环PID调控)
		if (CountSpeedTurn >= 50)
		{
			CountSpeedTurn = 0;

			LeftSpeed = Encoder_Get(1) / 44.0 / 0.05 / 9.27666;	 // 左电机转速，单位：转/秒
			RightSpeed = Encoder_Get(2) / 44.0 / 0.05 / 9.27666; // 右电机转速，单位：转/秒
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

		// 处理下位机通信信息 (提高频率，每 10ms 或 20ms 处理一次)
		if (CountControl >= 1)
		{
			CountControl = 0;
			// 接收下位机通信信息，并进行直角弯初始化
			if (Serial_GetRxFlag() == 1)
			{
				RxCmd = Serial_GetRxData();

				// --- 1. 最高优先级：判断是否该退出直角转弯 ---
				if (ConFlag != 0 && RxCmd == 3)
				{
					ConFlag = 0;

					// ⚠️ 这一步千万别漏了！清空旧的历史积分，不然恢复直行时车头会猛烈哆嗦
					Vision_Error_Integral = 0;
					Last_Vision_Error = 0;
					PID_Init(&SpeedPID); // 👈 只在这里初始化一次！
					PID_Init(&TurnPID);
				}

				// 👇 在收到数据的一瞬间捕捉直角信号
				if (ConFlag == 0) // 只有在直线状态下才检测
				{
					if (RxCmd == 8)
					{
						ConFlag = 1;
						PID_Init(&SpeedPID); // 👈 只在这里初始化一次！
						PID_Init(&TurnPID);
					}
					else if (RxCmd == 9)
					{
						ConFlag = 2;
						PID_Init(&SpeedPID); // 👈 只在这里初始化一次！
						PID_Init(&TurnPID);
					}
				}
			}
		}
		time_count = TIM_GetCounter(TIM1);
	}
}
