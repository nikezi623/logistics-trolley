#include "PHC_HeadFile.h"

// ==========================================
// 1. 系统状态与标志位
// ==========================================
uint8_t RunFlag = 0;					  // 运行启停标志位 (0:停止, 1:运行)
uint8_t ConFlag = 0;					  // 转向状态 (0:正常巡线, 1:右直角, 2:左直角, 3:全黑)
uint8_t all_black_flag = 0;				  // 全黑任务状态机阶段
uint8_t send_item_flag = 0;				  // 投放货物触发标志
uint8_t KeyNum = 0;						  // 按键键值
uint16_t time_count = 0;				  // 定时器计数
uint8_t have_turned_flag = 0;			  // 已经转过直角弯的标志位
uint8_t line_end_fine_flag = 0;			  // 巡线结束标志位
uint8_t have_avoid_flag = 0;			  // 避障结束标志位
uint8_t avoid_flag = 0;					  // 避障状态机
uint16_t DelayCount_avoid = 0;			  // 避障计时器
uint8_t RightTurn_flag = 0;				  // 直角弯状态机
uint16_t DelayCount_right_angle_turn = 0; // 直角弯计时器

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
#define BASE_SPEED 3.56 // 基础直道速度
#define MIN_SPEED 0.00	// 弯道最低速度

float VISION_KI = 0;		  // 视觉误差积分系数 (从0慢慢调)
float VISION_KD = 2.5;		  // 视觉误差微分系数
float TURN_GAIN = 1.05;		  // 转向环灵敏度增益
float SPEED_DROP_K = 5.5;	  // 弯道减速系数
int16_t GyroZ_Offset = 0;	  // 零偏值
float YawAngle = 0.0f;		  // 偏航角
uint8_t Gyro_Cal_Done = 0;	  // 校准完成标志位 (0:未完成, 1:已完成)
float Target_YawAngle = 0.0f; // 【新增】视觉给出的目标角度
float real_gz = 0;
float Base_Yaw = 0.0f;					// 【新增】当前行驶的基准方向 (0, 90, -90 等)
static float avoid_original_yaw = 0.0f; // 在文件开头或 main 函数上方定义一个静态变量，记录避障前的原始偏航角

PID_t SpeedPID = {
	.Kp = 4.00,
	.Ki = 0.66,
	.Kd = 0.00,
	.OutMax = 100,
	.OutMin = -100,
	.ErrorIntMax = 150,
	.ErrorIntMin = -150,
};

// 用于正常巡线的视觉差速 PID (采用你第一版的参数)
PID_t TurnPID_Vision = {
	.Kp = 6.00,
	.Ki = 2.00,
	.Kd = 4.00,
	.OutMax = 100,
	.OutMin = -100,
	.ErrorIntMax = 20,
	.ErrorIntMin = -20,
};

// 用于起步锁定和直角转弯的角度 PID (采用你第二版的参数)
PID_t TurnPID_Gyro = {
	.Kp = 1.86,
	.Ki = 0.5,
	.Kd = 0.50,
	.OutMax = 100,
	.OutMin = -100,
	.ErrorIntMax = 20,
	.ErrorIntMin = -20,
};

// 新增起步控制相关的标志位
uint8_t is_startup_flag = 0;	  // 起步阶段标志位
uint16_t startup_timer = 0;		  // 起步计时器
float Vision_DifSpeed_Target = 0; // 暂存视觉计算出的差速目标值

// 参数显示函数
void Show_parameter(void)
{
	OLED_Clear();
	OLED_Printf(0, 0, OLED_6X8, "Speed");
	OLED_Printf(0, 8, OLED_6X8, "%05.2f", SpeedPID.Kp);
	OLED_Printf(0, 16, OLED_6X8, "%05.2f", SpeedPID.Ki);
	OLED_Printf(0, 24, OLED_6X8, "%05.2f", SpeedPID.Kd);
	OLED_Printf(0, 32, OLED_6X8, "%+05.2f", SpeedPID.Target);
	OLED_Printf(0, 40, OLED_6X8, "%+05.2f", AvgSpeed);
	OLED_Printf(0, 48, OLED_6X8, "%+05.2f", SpeedPID.Out);

	OLED_Printf(38, 0, OLED_6X8, "Turn");
	OLED_Printf(38, 8, OLED_6X8, "%05.2f", TurnPID_Gyro.Kp);
	OLED_Printf(38, 16, OLED_6X8, "%05.2f", TurnPID_Gyro.Ki);
	OLED_Printf(38, 24, OLED_6X8, "%05.2f", TurnPID_Gyro.Kd);
	OLED_Printf(38, 32, OLED_6X8, "%+05.2f", TurnPID_Gyro.Target);
	OLED_Printf(38, 40, OLED_6X8, "%+05.2f", DifSpeed);
	OLED_Printf(38, 48, OLED_6X8, "%+05.2f", TurnPID_Gyro.Out);
	OLED_Printf(38, 56, OLED_6X8, "%+05.2f", TURN_GAIN);

	OLED_Printf(74, 40, OLED_6X8, "%+05.2f", VISION_KD);
	OLED_Printf(74, 48, OLED_6X8, "%+05.2f", TURN_GAIN);
	OLED_Printf(74, 56, OLED_6X8, "%+05.2f", SPEED_DROP_K);

	OLED_Printf(76, 0, OLED_8X16, "Rxd:%d", RxCmd);
	OLED_Printf(76, 16, OLED_8X16, "Rxd:%d", ConFlag);
	OLED_Update();
}

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
				PID_Init(&TurnPID_Vision); // ⚠️ 初始化视觉 PID
				PID_Init(&TurnPID_Gyro);   // ⚠️ 初始化陀螺仪 PID
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
				YawAngle = 0.0f;		// 起跑瞬间，当前角度认为是绝对 0 度
				Target_YawAngle = 0.0f; // 目标角度也是 0 度（直走）
				Base_Yaw = 0.0f;
				real_gz = 0;

				// 【新增】激活起步锁头模式
				is_startup_flag = 0;
				startup_timer = 0;
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
			if (ConFlag == 1 && RightTurn_flag == 11) // 右直角转弯
			{
				SpeedPID.Target = 0.86;
				RightTurn_flag = 2;
				// TurnPID_Vision.Target = 3;
			}
			else if (RightTurn_flag == 2 && DelayCount_right_angle_turn >= 1000)
			{
				SpeedPID.Target = 0;
				RightTurn_flag = 3;
				DelayCount_right_angle_turn = 0;
			}
			else if (RightTurn_flag == 3 && DelayCount_right_angle_turn >= 500)
			{
				PID_Init(&SpeedPID);
				PID_Init(&TurnPID_Vision);
				PID_Init(&TurnPID_Gyro);
				is_startup_flag = 1;
				DelayCount_right_angle_turn = 0;
				RightTurn_flag = 4;
			}
			else if (RightTurn_flag == 4 && DelayCount_right_angle_turn >= 1000)
			{
				PID_Init(&SpeedPID);
				PID_Init(&TurnPID_Vision);
				PID_Init(&TurnPID_Gyro);
				is_startup_flag = 0;
				DelayCount_right_angle_turn = 0;
				RightTurn_flag = 5;
			}

			if (ConFlag == 2 && RightTurn_flag == 12) // 左直角转弯
			{
				SpeedPID.Target = 0;
				// TurnPID_Vision.Target = -3;
			}
			else if (RightTurn_flag == 2 && DelayCount_right_angle_turn >= 1000)
			{
				SpeedPID.Target = 0;
				RightTurn_flag = 3;
				DelayCount_right_angle_turn = 0;
			}
			else if (RightTurn_flag == 3 && DelayCount_right_angle_turn >= 500)
			{
				PID_Init(&SpeedPID);
				PID_Init(&TurnPID_Vision);
				PID_Init(&TurnPID_Gyro);
				is_startup_flag = 1;
				DelayCount_right_angle_turn = 0;
				RightTurn_flag = 4;
			}
			else if (RightTurn_flag == 4 && DelayCount_right_angle_turn >= 1000)
			{
				PID_Init(&SpeedPID);
				PID_Init(&TurnPID_Vision);
				PID_Init(&TurnPID_Gyro);
				is_startup_flag = 0;
				DelayCount_right_angle_turn = 0;
				RightTurn_flag = 5;
			}
			else if (ConFlag == 4 || ConFlag == 3)
			{
				// 处于避障或全黑状态机，主循环挂机，把控制权完全交给定时器中断
			}
			else // 正常寻路模式
			{
				if (RxCmd == 2) // 真·丢线
				{
					SpeedPID.Target = MIN_SPEED;
					TurnPID_Vision.Target = 0;
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
					TurnPID_Vision.Target = (Vision_Error * TURN_GAIN) +
											(Vision_Error_Integral * VISION_KI) +
											(Vision_Derivative * VISION_KD);
				}
			}
		}

		// --- OLED 屏幕显示 ---
		Show_parameter();
	}
}

void TIM1_UP_IRQHandler(void) // 1ms进入一次
{
	static uint16_t CountSpeedTurn = 0, CountControl = 0, DelayCount_send_item = 0;

	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
		CountSpeedTurn++;
		CountControl++;
		Key_Tick();

		int16_t ax, ay, az, gx, gy, gz;
		MPU6050_GetData(&ax, &ay, &az, &gx, &gy, &gz);

		if (send_item_flag == 1)
		{
			DelayCount_send_item++;
		}
		else if (avoid_flag == 2 || avoid_flag == 3 || avoid_flag == 4 || avoid_flag == 6)
		{
			DelayCount_avoid++;
		}
		else if (RightTurn_flag == 2 || RightTurn_flag == 3 || RightTurn_flag == 4)
		{
			DelayCount_right_angle_turn++;
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

				// 【核心修改】动态控制逻辑切换
				// 如果处于起步阶段，或者正在进行直角/全黑任务 -> 使用陀螺仪角度环
				if (is_startup_flag == 1)
				{
					TurnPID_Gyro.Target = Target_YawAngle + Base_Yaw; // 目标角度 (直角转弯时更新)
					TurnPID_Gyro.Actual = YawAngle;					  // 实时角度
					PID_Update(&TurnPID_Gyro);

					DifPWM = -TurnPID_Gyro.Out; // 陀螺仪闭环输出
					LeftPWM = AvgPWM + DifPWM / 2;
					RightPWM = AvgPWM - DifPWM / 2;

					int16_t PWMMax = 33;

					// PWM 限幅
					if (LeftPWM >= PWMMax)
						LeftPWM = PWMMax;
					else if (LeftPWM <= -PWMMax)
						LeftPWM = -PWMMax;

					if (RightPWM >= PWMMax)
						RightPWM = PWMMax;
					else if (RightPWM <= -PWMMax)
						RightPWM = -PWMMax;
				}
				else
				{
					// TurnPID_Vision.Target = Vision_DifSpeed_Target; // 视觉算出的预期差速
					TurnPID_Vision.Actual = DifSpeed; // 编码器实时差速
					PID_Update(&TurnPID_Vision);

					DifPWM = TurnPID_Vision.Out; // 视觉闭环输出
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
				}

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

			// 扣除零偏，得到真实的角速度
			real_gz = gz - GyroZ_Offset;

			// 死区过滤：剔除极其微小的底盘震动静差
			if (real_gz >= -2 && real_gz <= 2)
			{
				real_gz = 0;
			}

			// 转化为角速度并积分 (假设满量程 ±2000°/s，灵敏度 16.4 LSB/°/s)
			YawAngle += ((float)real_gz / 16.4f) * 0.001f;

			if (Serial_GetRxFlag() == 1)
			{
				RxCmd = Serial_GetRxData();
				// ==========================================
				// 优先处理：是否正在进行【强制避障】？
				// 如果在避障，只跑避障状态机，屏蔽其他所有视觉指令！
				// ==========================================
			}

			if (ConFlag == 4)
			{
				if (avoid_flag == 1)
				{
					PID_Init(&TurnPID_Gyro);
					PID_Init(&SpeedPID);
					SpeedPID.Target = 0; // 先原地转向，不往前跑

					// 记录避障那一刻的绝对航向角，作为后续回归的基准！
					avoid_original_yaw = Base_Yaw;
					// 左转（或右转）45度避开障碍
					Base_Yaw = avoid_original_yaw + 45.0f;
					avoid_flag = 2;
					DelayCount_avoid = 0;
				}
				else if (avoid_flag == 2 && DelayCount_avoid >= 850) // 1秒足够原地转45度了
				{
					DelayCount_avoid = 0;
					SpeedPID.Target = 0.86; // 慢速直线绕开圆柱
					avoid_flag = 3;
				}
				else if (avoid_flag == 3 && DelayCount_avoid >= 3386) // 斜向直行的时间（根据实际距离调）
				{
					DelayCount_avoid = 0;
					SpeedPID.Target = 0; // 再次停车准备转向

					// 车头指向赛道：+45度减去90度 = -45度
					Base_Yaw = avoid_original_yaw - 40.0f;
					avoid_flag = 4;
				}
				else if (avoid_flag == 4 && DelayCount_avoid >= 850) // 等待车头转好
				{
					DelayCount_avoid = 0;
					SpeedPID.Target = 0.86; // 慢速往回开，主动去“撞”黑线
					avoid_flag = 5;
				}
				// ⚠️ 核心修改：斜角效应解法
				// 不再苛求 RxCmd == 83/93，只要脱离了丢线状态（RxCmd != 2），就说明压到线了！
				else if (avoid_flag == 5 && RxCmd != 2)
				{
					DelayCount_avoid = 0;
					SpeedPID.Target = 0.6; // 压线的瞬间，立刻刹车！

					// 不要急着把控制权给视觉！
					// 用陀螺仪把车头掰回避障前的方向（完美平行于黑线）
					Base_Yaw = avoid_original_yaw - 25;
					avoid_flag = 6;
				}
				else if (avoid_flag == 6 && DelayCount_avoid >= 1000) // 给800ms让车头在黑线上原地转正
				{
					// 此时车身已经和黑线平行，并且就在黑线上，完美交接！
					have_avoid_flag = 2;
					is_startup_flag = 0; // 退出陀螺仪锁头
					DelayCount_avoid = 0;
					PID_Init(&SpeedPID);
					PID_Init(&TurnPID_Vision); // 视觉PID接手时，误差已经很小了
					ConFlag = 0;			   // 恢复正常巡线状态
				}
			}
			// ⚠️ 重点：如果处于避障状态，后面的直角、巡线逻辑全部跳过不执行！
			// 不写 else，直接让它往下走，但底下的代码都用 else if 包起来了，所以不会执行。

			// ==========================================
			// 没在避障，正常处理视觉指令
			// ==========================================
			else
			{
				// --- 1. 全黑任务状态机 ---
				if (RxCmd == 1 && all_black_flag == 0)
					all_black_flag = 1;
				else if (RxCmd == 1 && all_black_flag == 1)
				{
					is_startup_flag = 1;
					SpeedPID.Target = BASE_SPEED;
					TurnPID_Gyro.Target = 0;
				}
				else if (RxCmd != 1 && (all_black_flag == 1))
				{
					is_startup_flag = 0;
					all_black_flag = 2;
					ConFlag = 0;
					Vision_Error_Integral = 0;
					Last_Vision_Error = 0;
					PID_Init(&SpeedPID);
					PID_Init(&TurnPID_Vision);
				}
				else if ((RxCmd == 1 || line_end_fine_flag == 1) && (all_black_flag == 2))
				{
					send_item_flag = 1;
					is_startup_flag = 1;
					PID_Init(&SpeedPID);
					PID_Init(&TurnPID_Gyro);

					if (DelayCount_send_item >= 2000)
					{
						DelayCount_send_item = 0;
						send_item_flag = 0;
						all_black_flag = 3;
						SpeedPID.Target = 3.86;
						Base_Yaw = 83;
					}
				}
				else if (RxCmd == 2 && all_black_flag == 3)
					all_black_flag = 4;
				else if (RxCmd == 1 && all_black_flag == 4)
					RunFlag = 0;

				// --- 2. 判断退出直角转弯 ---
				// 修改点：只在直角转弯时 (ConFlag == 1 或 2) 允许被居中指令打断
				if ((ConFlag == 1 || ConFlag == 2) && (RxCmd == 31 || RxCmd == 32 || RxCmd == 33))
				{
					ConFlag = 0;
					is_startup_flag = 0;
					Vision_Error_Integral = 0;
					Last_Vision_Error = 0;
					PID_Init(&SpeedPID);
					PID_Init(&TurnPID_Vision);
				}

				// --- 3. 捕捉进入直角转弯 ---
				if (ConFlag == 0 && have_turned_flag == 0)
				{
					if (RxCmd == 81 || RxCmd == 82 || RxCmd == 83)
					{
						ConFlag = 1;
						// is_startup_flag = 0;
						RightTurn_flag = 11;
						have_turned_flag = 1;
						Base_Yaw = -83;
						PID_Init(&SpeedPID);
						PID_Init(&TurnPID_Vision);
					}
					else if (RxCmd == 91 || RxCmd == 92 || RxCmd == 93)
					{
						ConFlag = 2;
						// is_startup_flag = 0;
						RightTurn_flag = 12;
						have_turned_flag = 1;
						Base_Yaw = 83;
						PID_Init(&SpeedPID);
						PID_Init(&TurnPID_Vision);
					}
				}
				else if (ConFlag == 0 && (RxCmd == 1 || RxCmd == 81 || RxCmd == 91) && have_turned_flag == 1)
				{
					line_end_fine_flag = 1;
					Base_Yaw = 83;
					is_startup_flag = 1;
				}

				// --- 4. 捕捉进入避障 ---
				if (ConFlag == 0 && have_avoid_flag == 0)
				{
					if (RxCmd == 99)
					{
						ConFlag = 4;
						is_startup_flag = 1;
						have_avoid_flag = 1;
						avoid_flag = 1;
						PID_Init(&SpeedPID);
						PID_Init(&TurnPID_Gyro);
					}
				}
			} // 结束没在避障的 else
		}
	}
	time_count = TIM_GetCounter(TIM1);
}