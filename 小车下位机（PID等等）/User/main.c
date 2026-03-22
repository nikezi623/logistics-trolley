#include "PHC_HeadFile.h"

// ==========================================
// 1. 系统状态与标志位
// ==========================================
uint8_t RunFlag = 0;               // 运行启停标志位 (0:停止, 1:运行)
uint8_t has_started_line_flag = 0; // 起跑确认标志 (防止开局默认RxCmd=2导致直接停车)
uint8_t KeyNum = 0;                // 按键键值
uint16_t time_count = 0;           // 定时器计数

// ==========================================
// 2. 视觉传感器参数
// ==========================================
uint8_t RxCmd = 2;               // 串口接收的传感器状态 (默认2:丢线)
float Vision_Error = 0;          // 当前视觉误差
float Last_Vision_Error = 0;     // 上一次视觉误差 (用于求微分)
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
#define MIN_SPEED 0.86  // 最低速度

float VISION_KI = 0;      // 视觉误差积分系数
float VISION_KD = 2.5;    // 视觉误差微分系数
float TURN_GAIN = 0.76;   // 转向环灵敏度增益
float SPEED_DROP_K = 5.5; // 弯道减速系数

int16_t GyroZ_Offset = 0;     // 零偏值
float YawAngle = 0.0f;        // 偏航角
uint8_t Gyro_Cal_Done = 0;    // 校准完成标志位
float Target_YawAngle = 0.0f; // 视觉给出的目标角度
float real_gz = 0;
float Base_Yaw = 0.0f;        // 当前行驶的基准方向

PID_t SpeedPID = {
    .Kp = 4.00,
    .Ki = 0.66,
    .Kd = 0.00,
    .OutMax = 100,
    .OutMin = -100,
    .ErrorIntMax = 150,
    .ErrorIntMin = -150,
};

// 用于正常巡线的视觉差速 PID
PID_t TurnPID_Vision = {
    .Kp = 6.00,
    .Ki = 2.00,
    .Kd = 4.00,
    .OutMax = 100,
    .OutMin = -100,
    .ErrorIntMax = 20,
    .ErrorIntMin = -20,
};

// 用于起步锁定和直角转弯的角度 PID
PID_t TurnPID_Gyro = {
    .Kp = 1.86,
    .Ki = 0.5,
    .Kd = 0.50,
    .OutMax = 100,
    .OutMin = -100,
    .ErrorIntMax = 20,
    .ErrorIntMin = -20,
};

// 起步控制相关的标志位
uint8_t is_startup_flag = 0;      // 起步陀螺仪锁头阶段标志位
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
                PID_Init(&TurnPID_Vision); 
                PID_Init(&TurnPID_Gyro);  

                Vision_Error_Integral = 0;
                Last_Vision_Error = 0;
                Encoder_Get(1);
                Encoder_Get(2);

                SpeedPID.Target = COMMONSPEED;
                DifPWM = 0;
                AvgPWM = 0;
                RxCmd = 2; // 默认丢线
                RunFlag = 1;
                has_started_line_flag = 0; // 重置起跑确认标志

                YawAngle = 0.0f;        // 起跑瞬间，当前角度认为是绝对 0 度
                Target_YawAngle = 0.0f; // 目标角度也是 0 度（直走）
                Base_Yaw = 0.0f;
                real_gz = 0;

                is_startup_flag = 0; 
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

        if (RunFlag == 1)
        {
            // 一旦接收到非丢线状态（全黑或正常寻线），标记车子正式压上赛道
            if (RxCmd != 2) {
                has_started_line_flag = 1;
            }

            // --- 核心运动控制目标计算 ---
            if (RxCmd == 2) // 丢线状态
            {
                if (has_started_line_flag == 1) {
                    // 确认上过赛道后发生丢线，判定为终点，停车
                    RunFlag = 0; 
                    SpeedPID.Target = 0;
                    TurnPID_Vision.Target = 0;
                } else {
                    // 还没上赛道（例如刚按下按键），保持速度为0等待
                    SpeedPID.Target = 0;
                    TurnPID_Vision.Target = 0;
                }
            }
            else if (RxCmd == 1) // 遇到全黑
            {
                if (is_startup_flag == 0) {
                    // 切换为锁头模式，重置PID
                    is_startup_flag = 1;
                    PID_Init(&SpeedPID);
                    PID_Init(&TurnPID_Gyro);
                }
                SpeedPID.Target = BASE_SPEED;
                TurnPID_Gyro.Target = Base_Yaw; // 保持 0 度目标，陀螺仪直行
            }
            else // 正常的视觉 P+I+D 巡线逻辑
            {
                if (is_startup_flag == 1) {
                    // 退出全黑区域，切换回视觉接管
                    is_startup_flag = 0;
                    Vision_Error_Integral = 0;
                    Last_Vision_Error = 0;
                    PID_Init(&TurnPID_Vision);
                }

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

        // --- OLED 屏幕显示 ---
        Show_parameter();
    }
}

void TIM1_UP_IRQHandler(void) // 1ms进入一次
{
    static uint16_t CountSpeedTurn = 0, CountControl = 0;

    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
    {
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);

        CountSpeedTurn++;
        CountControl++;
        Key_Tick();

        int16_t ax, ay, az, gx, gy, gz;
        MPU6050_GetData(&ax, &ay, &az, &gx, &gy, &gz);

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

                // 如果处于起步全黑阶段 -> 使用陀螺仪角度环
                if (is_startup_flag == 1)
                {
                    TurnPID_Gyro.Target = Target_YawAngle + Base_Yaw; 
                    TurnPID_Gyro.Actual = YawAngle;                  
                    PID_Update(&TurnPID_Gyro);
                    DifPWM = -TurnPID_Gyro.Out; // 陀螺仪闭环输出

                    LeftPWM = AvgPWM + DifPWM / 2;
                    RightPWM = AvgPWM - DifPWM / 2;

                    int16_t PWMMax = 33;
                    // PWM 限幅
                    if (LeftPWM >= PWMMax) LeftPWM = PWMMax;
                    else if (LeftPWM <= -PWMMax) LeftPWM = -PWMMax;
                    if (RightPWM >= PWMMax) RightPWM = PWMMax;
                    else if (RightPWM <= -PWMMax) RightPWM = -PWMMax;
                }
                else
                {
                    // 正常视觉巡线环
                    TurnPID_Vision.Actual = DifSpeed;
                    PID_Update(&TurnPID_Vision);
                    DifPWM = TurnPID_Vision.Out; // 视觉闭环输出

                    LeftPWM = AvgPWM + DifPWM / 2;
                    RightPWM = AvgPWM - DifPWM / 2;

                    // PWM 限幅
                    if (LeftPWM >= 100) LeftPWM = 100;
                    else if (LeftPWM <= -100) LeftPWM = -100;
                    if (RightPWM >= 100) RightPWM = 100;
                    else if (RightPWM <= -100) RightPWM = -100;
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

        // --- 2. 视觉传感器解析与航向角积分 (1ms 周期) ---
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
            }
            
            // 之前的冗杂状态机被清理，逻辑判断统一交由 main() 循环处理
        }
    }
    time_count = TIM_GetCounter(TIM1);
}