/*******************************************************/
/*					圆梦小车StepbyStep 程序之5								 */	
/*								—— 教小车“走路”											 */
/*								20070715 by Dingqi									 */											
/*******************************************************/
// 注：以下文档的 TAB 为 2 个字符！

/*-----------------------------------------------------
	这段程序是以StepByStep之四的程序YM1_Prog-4.C
	为基础，增加轨迹检测，使小车有一个“认路”的“眼睛”
	并借助这个“眼睛”学会”在路上走“。
	
	------------- 070609 ---------------
	因为程序简单，故所有内容合并在一个文件中。
	硬件资源分配：
	Timer0 仍然作为 1ms 定时器，作为工作的时基；
	Timer1 作为UART的波特率发生器。
	
	------------ 070615 ----------------------
	通讯协议：
	字节格式  ——  19200  8  N  1
	数据帧格式：
	帧头（2字节） 接收方地址（1字节） 发送方地址（1字节） 帧长（1字节） 命令（1字节） 数据域（N字节） 校验和（1字节）
	
	帧头 —— 由2个特殊的字节 0x55 0xAA构成；
	接收方地址 —— 通讯对象的“名字”，在有线通讯时也许多余，但无线时就需要了。
	发送方地址 —— 告诉接收方，是谁和你“说话”，便于接收方回答。
	帧长 —— 从命令开始到数据域结束的字节数
	校验和 —— 数据帧中从命令开始到数据域结束所有字节的算术和，取最低字节的反码。

 	----- 070618 -------------
	电机驱动所用资源：
	PCA2 —— 产生左侧电机的PWM信号
	PCA3 —— 产生右侧电机的PWM信号
	PWM 时钟源 —— Fosc/12
	PWM频率 —— 7200
	因为PWM为 8 位的，将有效 PWM 值定为  1 - 250，0 作为刹车控制， 255 作为惰行控制
	
	P2.0 —— 左侧电机PWM控制
	P2.1、P2.2  ——左侧电机工作状态控制 
	
	P2.4 —— 右侧电机PWM控制
	P2.5、P2.6  ——右侧电机工作状态控制
	
	将P2口设置为强输出模式。
	 
	补充定义命令：电机PWM控制命令
	命令字 —— 0x03
	数据域 —— 左电机PWM值（2字节，先低后高） 右电机PWM值
	返回数据帧：
	帧头 发送方地址 自己的地址  帧长 命令 电机控制输出（P2） 校验和

 	---------- 070622 -----------------------
	码盘监测所用资源:
	PCA0 —— 右侧电机码盘信号输入
	PCA1 —— 左侧电机码盘信号输入
	
	将PCA0、PCA1 设置为正、负沿脉冲捕获模式，允许中断。
	
	走直线控制逻辑：
	1、初级：
	如果右侧快，则将右侧置为惰行
	如果左侧快，则将左侧置为惰行
	如果相等，则均置为启动的PWM值
	
	补充定义命令： 走直线
	命令字 —— 0x04
	数据域 —— 行走距离（2字节）PWM 基准值（2字节，先低后高）
	其中行走距离为左侧车轮的脉冲计数值，按目前的几何尺寸，最大值65535 应该可以走 655圈，约合86m，为“0" 时连续行走。
	
	返回数据帧：
	帧头 发送方地址 自己的地址  帧长 命令 校验和

 	---------- 070715 -----------------------
	轨迹检测所用资源：
	P1.0 - P1.3 ——  4路轨迹信号输入，模拟方式输入，使用前  4 通道A/D
	P3.3(INT1) —— 控制背景采样和信号采样，推挽输出
	注意，P3 口的状态寄存器作了修改，请参照原理图理解！！！
	
 	采样器对应关系：
	 顶视：  			左  ——  右
	 采样器  			1  2  3  4
	 PORT P1.			3  2  1  0

	因为小车速度不快,通常小于 0.5m/s，所以将采样设计为 2 ms 一次，对应移动只有1mm，
	安排在中断外执行。

	具体处理步骤为：
	设计一个采样标志位，每1ms中断取反一次，为真时处理；
	在中断时，如果标志为真，则打开采样LED；
	在主程序中检测Timer0，如果打开LED时间超过一定值，则启动AD，
	完成后关闭采样LED，结束一个采样周期。

	采样处理将轨迹信号转换为逻辑值，存放在一个状态字中，每位对应一个传感器，0 —— 不在轨迹，1 —— 在轨迹，
	默认轨迹为深色（黑色）。
	
	之后再根据这个状态字控制小车行走，因为是示例，所以只处理中间两个轨迹传感器的状态，两侧的留给读者去发挥!
	
 	控制逻辑：
	 只处理 2、3号传感器，即位 b1、b2,
	 两个均在轨迹上，左右电机为同样值，
	 b1 不在，右侧电机惰行，
	 b2 不在，左侧电机惰行。
	 和走直线类似，只是控制变量从码盘计数差变为轨迹采样的状态。

	补充定义命令： 走轨迹
	命令字 —— 0x05
	数据域 —— 行走距离（2字节）PWM 基准值（2字节，先低后高）
	其中行走距离为左侧车轮的脉冲计数值，按目前的几何尺寸，最大值65535 应该可以走 655圈，约合86m，为“0" 时连续行走。
	
	返回数据帧：
	帧头 发送方地址 自己的地址  帧长 命令 校验和

-------------------------------------------------------*/

#include 	<STC12C5410AD.h>				/* STC12C5410AD 的头文件,为MCU中各个硬件寄存器定个名称，以便C语言中使用*/

sbit			Work_Display = P3^4;		// 根据硬件设计用与“主控工作指示”接近的名称取代硬件，使程序减少与硬件的相关性
sbit			g_bSample = P3^3;				// 轨迹采样控制端， 070715

//  --------- 常数定义 --------------------------
#define		TRUE		1
#define		FALSE		0

#define		LIGHT		0								// 亮逻辑值，用直观的符号化常数替换与硬件密切相关的逻辑值，增加程序的可读性、可移植性。
#define		DARK		1								// 暗逻辑值

#define		LIGHT_TIME		1000			// 亮时间，使用符号化常数，便于修改，增加程序可读性
#define		DARK_TIME			1000			// 暗时间

#define		P3MODE0				0xB0			/* 1011 0000，P3.0 P3.1 P3.2 标准51口，P3.3 推挽输出 ，P3.4 OC输出, P3.5 P3.7 输入*/
#define		P3MODE1				0x18			/* 0001 1000   ----- 070715 ----- */

/* 定时器参数定义 */

#define		T0MODE0 0x00		// 0000 0000，Timer0工作在模式0 ，13位定时；
#define		T0MODE1 0x01		// 0000 0001，Timer0工作在模式1 ，16位定时；
#define		T0MODE2 0x02		// 0000 0010，Timer0工作在模式2 ，8 位自动重加载定时；
#define		T0MODE3 0x03		// 0000 0011，Timer0工作在模式3 

#define		T0_TIMER 0x00		// Timer0 工作在定时器模式
#define		T0_COUNTER 0x04	// Timer0 工作在计数器模式
#define		T0_DISGATE 0x00	// Timer0 禁止INT0引脚控制
#define		T0_ENGATE 0x08	// Timer0 允许INT0引脚控制

#define		T1MODE0 0x00		// 0000 0000，Timer0工作在模式0 ，13位定时；
#define		T1MODE1 0x10		// 0000 0001，Timer0工作在模式1 ，16位定时；
#define		T1MODE2 0x20		// 0000 0010，Timer0工作在模式2 ，8 位自动重加载定时；
#define		T1MODE3 0x30		// 0000 0011，Timer0工作在模式3 

#define		T1_TIMER 0x00		// Timer1 工作在定时器模式
#define		T1_COUNTER 0x40	// Timer1 工作在计数器模式
#define		T1_DISGATE 0x00	// Timer1 禁止INT1引脚控制
#define		T1_ENGATE 0x80	// Timer1 允许INT1引脚控制

#define		SET_T0X12_C 0x80		// or AUXR
#define		CLR_T0X12_C 0x7F		// and AUXR

#define		SET_T1X12_C 0x40		// or AUXR
#define		CLR_T1X12_C 0xBF		// and AUXR

#define		TIME1ms_C		0xF8D0	/* 1ms 定时的加载值字节，对应 22.1184MHz 定时器12分频 */		
#define		TIME1msH_C	0xF8		/* 1ms 定时的加载值高字节 */
#define		TIME1msL_C	0xD0		/* 1ms 定时的加载值低字节 */

/* 中断处理参数定义 */
#define		EnINT0_C 	0x01
#define		EnT0_C 		0x02
#define		EnINT1_C	0x04
#define		EnT1_C 		0x08
#define		EnUART_C	0x10
#define		EnADCSPI_C 	0x20
#define		EnPCALVD_C 	0x40

#define		INT0_DOWN	0x01		// TCON 中对INT0中断信号的控制，下降沿触发；
#define		INT0_LOW	0x00		// TCON 中对INT0中断信号的控制，低电平触发；

#define		INT1_DOWN	0x04		// TCON 中对INT1中断信号的控制，下降沿触发；
#define		INT1_LOW	0x00		// TCON 中对INT1中断信号的控制，低电平触发；

#define		NOIP_C		0x00		/* 无优先级 */

#define		INT0_HIGH 	0x01	// IP 寄存器中的优先级设置，IPH暂不处理。
#define		T0_HIGH			0x02
#define		INT1_HIGH		0x04
#define		T1_HIGH			0x08
#define		UART_HIGH		0x10
#define		ADCSPI_HIGH	0x20
#define		PCALVD_HIGH 0x40	

// 显示摩尔斯电码用
#define		MAX_CHAR_NUM  30					// 最多允许显示字符数
#define		MAX_GAP_NUM		9						// 一个字母最多所需的亮、暗变化次数
#define		BASE_TIME			200					// 莫尔斯电码的基本时间，暂定 200ms

/* 莫尔斯电码表，将大写字母转换为一个亮、暗序列表 */
unsigned char code ga_ucMorseCode[26][9]={1,2,3,7,0,0,0,0,0,
																					3,2,1,2,1,2,1,7,0,
																					3,2,1,2,3,2,1,7,0,
																					3,2,1,2,1,7,0,0,0,
																					1,7,0,0,0,0,0,0,0,
																					1,2,1,2,3,2,1,7,0,
																					3,2,3,2,1,2,1,7,0,
																					1,2,1,2,1,2,1,7,0,
																					1,2,1,7,0,0,0,0,0,
																					1,2,3,2,3,2,3,7,0,
																					3,2,1,2,3,7,0,0,0,
																					1,2,3,2,1,2,1,7,0,
																					3,2,3,7,0,0,0,0,0,
																					3,2,1,7,0,0,0,0,0,
																					3,2,3,2,3,7,0,0,0,
																					1,2,3,2,3,2,1,7,0,
																					3,2,3,2,1,2,3,7,0,
																					1,2,3,2,1,7,0,0,0,
																					1,2,1,2,1,7,0,0,0,
																					3,7,0,0,0,0,0,0,0,
																					1,2,1,2,3,7,0,0,0,
																					1,2,1,2,1,2,3,7,0,
																					1,2,3,2,3,7,0,0,0,
																					3,2,1,2,1,2,3,7,0,
																					3,2,1,2,3,2,3,7,0,
																					3,2,3,2,1,2,1,7,0};
																					
//————————————————————————————————————————————————————																				  
// 以下为StepByStep之二所增加的通讯程序用常量， 070615

/* 串口参数 */
#define		B_57600		4
#define		B_38400		3
#define 	B_19200		2
#define 	B_9600		1
#define 	B_4800		0

// 在 22.1184Hz 下用 T1 作波特率发生器， 1 分频。

#define		B57600_C  244
#define   B38400_C  238
#define 	B19200_C	220		
#define 	B9600_C		184		
#define 	B4800_C		144		

#define		UART_MODE1_C 0x40		// SM0,SM1= 01
#define		EN_RCV_C	0x10			// REN=1

// 允许串口中断
#define		EnUART_C	0x10

// 串口中断优先级 
#define		UART_HIGH		0x10

// 数据接收用常数
#define 	MaxRcvByte_C	32  		// 接收缓冲区的大小，此值必须对应一定位数的二进制数，便于利用屏蔽高位的方式处理指针。
#define		MaxTxdByte_C	32

// 帧命令字
#define 	READ_MEMORY  		0x01		// 读内存命令字
#define		WRITE_MEMORY  	0x02		// 写内存命令字
#define		MOTOR_PWM_CTRL 	0x03		// 电机PWM控制命令，之三增加， 070618
#define		RUN_STRAIGHT 		0x04		// 走直线命令， 之四增加，  070622
#define 	RUN_ON_LINE			0x05		// 走轨迹， 之五增加， 070715

// 自己的设备地址
#define		MY_ADDR 				0x01

//————————————————————————————————————————————————————																				  
// 以下为StepByStep之三所增加的电机控制程序用常量， 070618

// 马达控制口
#define 	MotorDrv		P2			// P2.0 - P2.2 控制左电机， P2.4 - P2.6 控制右电机，输出

#define		P2MODE0			0x00			/* 0000 0000，P2 口作为马达控制，均作为推挽输出*/
#define		P2MODE1			0xFF			/* 1111 1111，*/

// PCA 初始化常数，
#define		STARTPCA_C 	0x40;		// or CCON 中的CR位控制 启动 PCA
#define		STOPPCA_C 	0xBF;		// and CCON 中的CR位控制 停止 PCA

#define		FOSCdiv12_C 0x00
#define		FOSCdiv2_C 	0x02	
#define		T0Over_C 		0x04		// T0 溢出
#define		ECI_C 			0x06		// 外部脉冲输入

#define		EnCF_C		0x01			// 允许PCA溢出中断

#define		EnCMP_C		0x40			// 0100 0000, CCAPMn 中控制位6， 允许比较器
#define		EnCAPP_C	0x20			// 0010 0000, CCAPMn 中控制位5， 允许上升沿捕获
#define		EnCAPN_C	0x10			// 0001 0000, CCAPMn 中控制位4， 允许下降沿捕获
#define		EnMAT_C		0x08			// 0000 1000, CCAPMn 中控制位3， 允许匹配或捕获后置位 CCFn
#define		EnTOG_C		0x04			// 0000 0100, CCAPMn 中控制位2， 允许匹配或捕获后触发输出翻转
#define		EnPWM_C		0x02			// 0000 0010, CCAPMn 中控制位1， 允许对应的端子输出PWM信号
#define		EnCCFI_C	0x01			// 0000 0001, CCAPMn 中控制位0， 允许CCF产生中断


// PCA 中断
#define		EnPCALVD_C 	0x40

// PCA 优先级
#define		PCALVD_HIGH 0x40	

#define		MOTOR_L			0
#define		MOTOR_R			1

#define		BRAKE_PWM		0			// 刹车 PWM 控制值
#define		FLOAT_PWM		255		// 惰行 PWM 控制值

// 电机控制输出

/* 控制位对应关系：
电机1（左）
P2.0 - CtrlL1, H 桥的左、右上臂，1 电平输出导通 0 电平输出截止，只有在对应下臂截止时有效
P2.1 - CtrlL2, H 桥的左下臂，1 电平输出导通 0 电平输出截止
P2.2 - CtrlL3, H 桥的右下臂，0 电平输出导通 1 电平输出截止


电机 2（右）
P2.4 - CtrlR1, H 桥的左、右上臂，1 电平输出导通 0 电平输出截止，只有在对应下臂截止时有效
P2.5 - CtrlR2, H 桥的左下臂，1 电平输出导通 0 电平输出截止
P2.6 - CtrlR3, H 桥的右下臂，0 电平输出导通 1 电平输出截止

控制逻辑：
Ctrl1		Ctrl2		Ctrl3		Drv1		Drv2		Drv3		Drv4		电机状态
	X				0				0			0/截止	0/导通	0/截止	0/导通		刹车
	PWM			1				0			PWM			1/截止	0/截止	0/导通		正转
	PWM			0				1			0/截止	0/导通	PWM			1/截止		反转
	0				1				1			0/截止	1/截止	0/截止	1/截止		惰行
	1				1				1			1/导通	1/截止	1/导通	1/截止		刹车

电机 2 控制逻辑相同，只是对应高 3 位。

*/

// 左电机控制
#define 	ML_FORWARD	0x02			// 0000 0010 前进，左上、右下导通，

#define 	ML_BACK			0x04			// 0000 0100 后退，左下、右上导通

#define 	ML_FLOAT		0x06			// 0000 0110 浮空，4 个臂都截止，此时 Ctrl1 应停止PWM输出，维持“0”。
#define 	ML_BRAKE		0x00			// 0000 0000 刹车，两个下臂导通，上臂截止

#define		ML_MASK			0xF8			// 清除原来输出值，保留另一个电机的输出

// 右电机控制
#define 	MR_FORWARD	0x20			// 0010 0000 前进，左上、右下导通，

#define 	MR_BACK			0x40			// 0100 0000 后退，左下、右上导通

#define 	MR_FLOAT		0x60			// 0110 0000 行进控制浮空，4 个臂都截止
#define 	MR_BRAKE		0x00			// 0000 0000 行进控制刹车，两个下臂导通，上臂截止

#define		MR_MASK			0x8F			// 清除原来输出值，保留另一个电机的输出

//————————————————————————————————————————————————————																				  
// 以下为StepByStep之四所增加的走直线用常量， 070622
// 无！

//————————————————————————————————————————————————————																				  
// 以下为StepByStep之五所增加的走轨迹用常量， 070715

#define			P1MODE0			0xFF			/* 1111 1111，P1口均作为高阻输入，使用模拟功能 */ 
#define			P1MODE1			0x00			/* 0000 0000 */

// AD 转换器初始化常数
#define			ADPWRON_C		0x80
#define			ADSPEED3_C 	0x60			// AD速度分为 4 级，0 最低  3 最高， 210 个时钟周期转换一次
#define			ADSPEED2_C 	0x40
#define			ADSPEED1_C 	0x20
#define			ADSPEED0_C 	0x00

#define			CLRADFLAG_C 0xEF			// and AD_CONTR
#define			GETADFLAG_C 0x10			// and AD_CONTR, 得到AD结束标志

#define			STARTAD_C		0x08			// OR AD_CONTR

unsigned char code DEFAULT_THRES[4] = {127,150,127,127};	// 默认判断阈值
#define			DEFAULT_THRES_DELTA 	20											// 默认判断回差

#define			EN_SAMPLE  1 												// 采样控制，此时为允许采样。
#define			DIS_SAMPLE 0

#define			SAMP_LED_ONTIME		600								// 打开采样 LED 后的延时时间，约 300us


// --- 全局变量定义 --------------------
bit					 	g_b1msFlag;					// 1ms中断标志

unsigned char ga_ucDispBuf[MAX_CHAR_NUM+1];		// 显示缓冲区，存放要显示的字符，为“0”表示结束
unsigned char gi_ucGetCharPtr;								// 从显示缓冲区取字符指针

unsigned char ga_ucCharDispBuf[MAX_GAP_NUM];	// 一个字符显示时亮、暗显示序列，存放显示基本时间单位的个数，为“0”表示结束
unsigned char	gi_ucGetDispNum;								// 取亮、暗基本时间数指针

unsigned char	gc_ucDispNumCnt;								// 基本时间数计数器
unsigned int	gc_uiBaseTime;									// 基本时间计数器, 1ms 计数

unsigned char code 	DISP_CONTENT[MAX_CHAR_NUM+1] = {"HELLOWORLD"};		// 要显示的内容,因为莫尔斯电码没有定义空格

// -------- 以下为StepByStep之二增加的变量 070615 ------------------

// 数据接收用 
unsigned char xdata	ga_ucRcvBuf[MaxRcvByte_C];		// 接收缓冲区
unsigned char data	gi_ucSavePtr;									// 存数指针，每收到一个字节保存到缓冲区后加“1”。

unsigned char	data	gi_ucGetPtr;									// 从缓冲区中取数的指针，每取出一个字节后加“1”。
unsigned char	idata	gi_ucStartPtr;								// 帧起始位置，指向数据区开始。
unsigned char	idata	gi_ucEndPtr;									// 帧结束位置。
unsigned char	idata	gc_ucDataLen;									// 帧长，即数据区字节数。
bit									g_bNewData;										// 串口收到一个字节标志,为减少变量交互。
bit									g_bStartRcv;									// 开始接收数据帧标志。

unsigned char	xdata	ga_ucTxdBuf[MaxTxdByte_C];		// 发送缓冲区，用于返回转速值等。
unsigned char	data	gi_ucTxdPtr;									// 发送指针
unsigned char	data	gc_ucTxdCnt;									// 发送字节计数

// -------- 以下为StepByStep之三增加的变量 070618 ------------------

unsigned char code 	ga_ucForward[2] = {ML_FORWARD,MR_FORWARD};		// 前进控制
unsigned char code 	ga_ucBack[2] = {ML_BACK,MR_BACK};							// 后退控制

unsigned char code 	ga_ucFloat[2] = {ML_FLOAT,MR_FLOAT};					// 惰行控制
unsigned char code 	ga_ucBrake[2] = {ML_BRAKE,MR_BRAKE};					// 刹车控制

unsigned char code 	ga_ucMask[2] = {ML_MASK,MR_MASK};							// 屏蔽字，为了输出新的控制信号，清除原来的控制值

unsigned int				ga_uiMotorCtrl[2];						// 电机控制参数，高字节控制方向， 0 - 前进 0xFF - 后退，低字节PWM值 

// -------- 以下为StepByStep之四增加的变量 070622 ------------------

unsigned char data 	gac_ucWheel_Cnt[2];						// 码盘输入信号计数，用计数方式而非标志，是为了避免丢失。
unsigned int	idata gac_uiRunCnt[2];							// 对车轮转动的码盘信号计数，以达到计算行走距离的目的

unsigned int	idata	g_uiBase_PWM;									// 基准 PWM 值

unsigned int	idata	g_uiLeftRunNum;								// 左侧车轮行走计数设定值
unsigned int  data	gc_uiLeftRunCnt;							// 左轮行走减计数器
bit									g_bStopRunStraight;						// 停止运行标志

// -------- 以下为StepByStep之五增加的变量 070715 ------------------

bit									g_bEnSample;									// 允许采样标志
bit									g_bSampleStart;								// 通知采样

unsigned char	idata	ga_ucSampleVal[5];						// 4 路采样值，暂时取 8 位， 第 5 个单元存放转换后的逻辑状态
																									// 存放4个传感器的状态，一个对应一位，“1”为在轨迹上
																									
unsigned char idata	ga_ucThreshold[4];						// 4 路黑白判断值
unsigned char	idata	g_ucThresDelta;								// 判断回差,用于消除判断时的临界毛刺

bit									g_bStopRunOnLine;							// 停止运行标志


// ------------- 函数声明 -----------------
void print(unsigned char *p_ucDispConPtr);
bit getNextGap(void);
void getNextChar(void);

// -------- 以下为StepByStep之二增加的函数 070615 ------------------
void init_SIO(unsigned char baud);
bit dataFrame_OK(void);
void do_Command(void);

// -------- 以下为StepByStep之三增加的函数 070618 ------------------
unsigned char DriveMotor(unsigned char No,unsigned int uiPWM_Val);

// -------- 以下为StepByStep之四增加的函数 070622 ------------------
void run_Straight(void);
void StopRunStraight(void);

// -------- 以下为StepByStep之五增加的函数 070715 ------------------
unsigned char lineSamp_proc(unsigned char ucOldSenStat);
void run_Online(unsigned char ucSensorStat);
void StopRunOnLine(void);

/*********** 主程序 *****************/
void main(void)
{
	unsigned char i;	
	
	// ----------- 初始化硬件 ---------------
	P3M0 = P3MODE0;									// 因为只涉及 P3 口，所以此处只初始化 P3
	P3M1 = P3MODE1;

	/* 初始化定时器 */
	
	TMOD = T0MODE1|T1MODE2;		// Timer0工作在模式1 ，16位定时,Timer1 工作在模式 2 ，8位重加载，作为波特率发生器；
	AUXR = AUXR&CLR_T0X12_C;	// Timer0 工作在12分频
	
	
	TCON = 0;									/* 未使用外部中断，所以不用定义中断的触发方式 */
		
	TH0 = TIME1msH_C;
	TL0 = TIME1msH_C;
	TR0 = TRUE;
	
	/* 初始化中断 */
	IE = EnT0_C;							// 此处只允许 Timer0 中断，
	
	IPH = NOIP_C;							// 此处不设优先级
	IP = NOIP_C;					
	
	// ----- StepByStep之二增加的硬件初始化 070615 ------------------
	
	//初始化串口 070615
	init_SIO(B_19200);
	
	// 初始化相关中断 070615
	IE = IE|EnUART_C;								// 允许 UART 中断

	// ----- StepByStep之三增加的硬件初始化 070618 ------------------
	
	P2M0 = P2MODE0;												// 电机涉及 P2 口，初始化 P2
	P2M1 = P2MODE1;
	
	// 初始化 PCA
	CMOD = FOSCdiv12_C;										// PCA 时钟源为 Fosc/12 , 不允许 count 溢出中断 CF。休眠时 PCA 工作
	
	CCAPM2 = EnCMP_C|EnPWM_C;							// PCA 的模块 2 用于左电机控制，8 位PWM模式；
	PCA_PWM2 = 0x03;											// 初始化为PWM恒输出 0， 以便进入惰行状态。
	CCAP2L = 0xFF;
	CCAP2H = 0xFF;

	CCAPM3 = EnCMP_C|EnPWM_C;							// PCA 的模块 3 用于右电机控制，8 位PWM 模式;
	PCA_PWM3 = 0x03;											// 初始化为PWM恒输出 0， 以便进入惰行状态。
	CCAP3L = 0xFF;
	CCAP3H = 0xFF;
	
	CL = 0;
	CH = 0;
	CCON = CCON|STARTPCA_C;								// 启动 PCA

	// ----- StepByStep之四增加的硬件初始化 070622 ------------------

	// 初始化 PCA0、1 用于码盘采样 ---- 070622
	CCAPM0 = EnCAPP_C|EnCAPN_C|EnCCFI_C;	// PCA 的模块 0 正、负跳均捕获，允许中断	,右侧码盘输入
	CCAPM1 = EnCAPP_C|EnCAPN_C|EnCCFI_C;	// PCA 的模块 1 正、负跳均捕获，允许中断	，左侧码盘输入

	// 初始化相关中断
	IE = IE|EnPCALVD_C;										// PCA 中断
	IP = IP|PCALVD_HIGH;									// PCA置为优先级 1 

	// ----- StepByStep之五增加的硬件初始化 070715 ------------------
	
	P1M0 = P1MODE0;												// 采样涉及 P1 口，初始化 P1
	P1M1 = P1MODE1;
	
	/* 初始化 AD 用于轨迹采样  ----*/	
	
	ADC_CONTR	 = ADPWRON_C|ADSPEED3_C;
	
	// ------ 初始化起始变量 ----------------

	g_b1msFlag = FALSE;

	print(DISP_CONTENT);						// 显示内容初始化
	gi_ucGetCharPtr = MAX_CHAR_NUM;	// 为了启动第一次显示
	
	Work_Display = DARK;						// 熄灭
	gi_ucGetDispNum = MAX_GAP_NUM;	// 为了启动第一次显示
	gc_ucDispNumCnt = 1;
	gc_uiBaseTime = 1;
	
	// ----- StepByStep之二增加的变量初始化 070615 ------------------
	
	gi_ucSavePtr=0;
	gi_ucGetPtr=0;
	
	g_bNewData = FALSE;
	g_bStartRcv = FALSE;

	// ----- StepByStep之三增加的变量初始化 070618 ------------------
	ga_uiMotorCtrl[MOTOR_L] = FLOAT_PWM;
	ga_uiMotorCtrl[MOTOR_R] = FLOAT_PWM;
	
	DriveMotor(MOTOR_L,ga_uiMotorCtrl[MOTOR_L]);
	DriveMotor(MOTOR_R,ga_uiMotorCtrl[MOTOR_R]);			// 上电时将电机置于惰行状态
	
	// ----- StepByStep之四增加的变量初始化 070622 ------------------
	
	gac_ucWheel_Cnt[MOTOR_L] = 0;
	gac_ucWheel_Cnt[MOTOR_R] = 0;					
	
	gac_uiRunCnt[MOTOR_L] = 0;
	gac_uiRunCnt[MOTOR_R] = 0;
	
	g_bStopRunStraight = TRUE;
	g_bStopRunOnLine = TRUE;
	
	// ----- StepByStep之五增加的变量初始化 070715 ------------------
	
	g_bSample = DIS_SAMPLE;											// 采样器初始化为采背景光状态，关闭发射管
	
	g_bEnSample = FALSE;
	g_bSampleStart = FALSE;
	
	for(i=0;i<4;i++)
	{
		ga_ucSampleVal[i] = 0;									// 轨迹信号模拟值清“0”
		ga_ucThreshold[i] = DEFAULT_THRES[i];		// 轨迹判断阈值初始化
	}

	g_ucThresDelta = DEFAULT_THRES_DELTA;			// 轨迹判断回差初始化

	ga_ucSampleVal[4] = 0;										// 逻辑状态清“0”
		
	// --------- 主循环 ------------------------
	
	EA = TRUE;														// 启动中断，开始正常工作
	
	while(1)
	{
		if(g_b1msFlag == TRUE)
		{
			g_b1msFlag = FALSE;
			
			// ----- 主控工作指示灯控制 ----------- 070609
			gc_uiBaseTime--;						// 基本时间计时
			if(gc_uiBaseTime ==0)
			{
				gc_uiBaseTime = BASE_TIME;
				
				if(getNextGap())					// 当前字符显示，如果显示完则取下一个字符
				{
					getNextChar();					// 取下一个字符
				}
			}			
		}

		// ----- 小车走直线控制 ----------  070622
		if(gac_ucWheel_Cnt[MOTOR_L]>0)
		{
			// 左侧码盘计数
			gac_uiRunCnt[MOTOR_L]++;
			gac_ucWheel_Cnt[MOTOR_L]--;
			if(g_bStopRunStraight == FALSE)
			{
				run_Straight();
			}
			
			// 行走距离控制
			if(gc_uiLeftRunCnt>0)
			{
				gc_uiLeftRunCnt--;
				if(gc_uiLeftRunCnt == 0)
				{
					if(g_bStopRunStraight == FALSE)
					{
						StopRunStraight();							// 停止直线行走
					}
					
					if(g_bStopRunOnLine == FALSE)
					{
						StopRunOnLine();								// 停止走轨迹			070715
					}
				}
			}
		}
		
		if(gac_ucWheel_Cnt[MOTOR_R]>0)
		{
			// 右侧码盘计数
			gac_uiRunCnt[MOTOR_R]++;
			gac_ucWheel_Cnt[MOTOR_R]--;
			if(g_bStopRunStraight == FALSE)
			{
				run_Straight();
			}
		}
		
		// -------- 走轨迹处理  --------- 070715
		if(g_bSampleStart)
		{
			union
			{
				unsigned int all;
				unsigned char b[2];
			}uiTemp;
			
			uiTemp.b[0] = TH0;
			uiTemp.b[1] = TL0;
			if(uiTemp.all > (TIME1ms_C + SAMP_LED_ONTIME))
			{
				g_bSampleStart = FALSE;
				ga_ucSampleVal[4] = lineSamp_proc(ga_ucSampleVal[4]);	// 处理轨迹采样
				g_bSample = DIS_SAMPLE;																// 退出采样状态，回到背景采样
				if(g_bStopRunOnLine == FALSE)
				{
					run_Online(ga_ucSampleVal[4]);											// 走轨迹处理
				}
			}			
		}

		// -------- 通讯处理 --------- 070618
		if(g_bNewData)
		{
			g_bNewData = FALSE;
			if(dataFrame_OK())					// 串口收到数据后的处理，与PC程序中的函数 DataFrame_OK() 相当
			{
				do_Command();							// 执行数据帧中的命令
			}						
		}		
	}																// 不断循环, 在所有嵌入式应用的主程序中,都有这样一个无限循环.
}


/************** 函数 **************************************/

/********************************************/
/* 将要显示的字符送显示缓冲区 							*/
/********************************************/

void print(unsigned char *p_ucDispConPtr)
{
	unsigned char i;
	i = 0;
	
	while(*p_ucDispConPtr != 0)
	{
		ga_ucDispBuf[i] = *p_ucDispConPtr;
		i++;
		p_ucDispConPtr++;
	}
	ga_ucDispBuf[i] = 0;				// 字符串结束符
}

/********************************************/
/* 		取下一个显示间隔 											*/
/* 返回为 FALSE - 表示继续显示此字符				*/
/* 返回为 TRUE -  表示要取下一个显示字符		*/ 
/********************************************/

bit getNextGap(void)
{
	bit flag = FALSE;
	
	gc_ucDispNumCnt--;
	if(gc_ucDispNumCnt == 0)
	{
		gi_ucGetDispNum++;
		if(gi_ucGetDispNum >MAX_GAP_NUM)
		{
			gi_ucGetDispNum = 0;					// 如果已经超过最后界限，则准备取下一个字符
			flag = TRUE;
		}
		else
		{
			if(ga_ucCharDispBuf[gi_ucGetDispNum] == 0)
			{
				gi_ucGetDispNum = 0;					// 如果显示基本时间数为“0”，则准备取下一个字符
				flag = TRUE;
			}
			else
			{
				gc_ucDispNumCnt = ga_ucCharDispBuf[gi_ucGetDispNum];
				if(gi_ucGetDispNum&0x01)
				{
					Work_Display = DARK;			// gucGetDispNum 奇数位置：暗
				}
				else
				{
					Work_Display = LIGHT;			// gucGetDispNum 偶数位置：亮
				}
				flag = FALSE;
			}
		}
	}
	
	return(flag);
}

/********************************************/
/* 						取下一个显示字符 							*/
/********************************************/
void getNextChar(void)
{
	unsigned char ucCharOrder,i;
	
	gi_ucGetCharPtr++;
	if((gi_ucGetCharPtr > MAX_CHAR_NUM)||(ga_ucDispBuf[gi_ucGetCharPtr]==0))
	{
		gi_ucGetCharPtr = 0;								// 已取字符到结尾，回到头。
	}
	
	ucCharOrder = ga_ucDispBuf[gi_ucGetCharPtr] - 0x41;			// 将大写字母ASCII吗转换为字母顺序，暂时只支持大写字母
	for(i=0;i<MAX_GAP_NUM;i++)
	{
		ga_ucCharDispBuf[i] = ga_ucMorseCode[ucCharOrder][i];	// 取出该字符对应的亮、暗序列
	}
	
	Work_Display = LIGHT;							// 每个字符都是从“亮”开始
	gi_ucGetDispNum = 0;
	gc_ucDispNumCnt = ga_ucCharDispBuf[gi_ucGetDispNum];
}


/********************************************/
/*          定时器 0 中断服务               */ 
/* 说明:  1ms 中断一次，	 									*/
/********************************************/

void  Timer0_Int(void) interrupt 1 using 1
{
	
	TH0 = TIME1msH_C;
	TL0 = TIME1msL_C;
			
	g_b1msFlag = TRUE;

	//  ------- 以下为StepByStep之五增加的处理 070715 ------------------
	
	g_bEnSample = ~g_bEnSample;
	if(g_bEnSample)
	{
		g_bSample = EN_SAMPLE;											// 打开采样LED
		g_bSampleStart = TRUE;
	}	
}

// -------- 以下为StepByStep之二增加的函数 070615 ------------------

/********************************************/
/* 名称：init_SIO														*/
/* 用途：初始化串口, 												*/
/* 参数: 波特率 , 模式固定为:1 							*/
/* 		1 START 8 DATA 1 STOP 								*/
/********************************************/

void init_SIO(unsigned char baud)
{
	// 波特率表
	unsigned char	code	TH_Baud[5]={B4800_C,B9600_C,B19200_C,B38400_C,B57600_C};
	
	AUXR = AUXR|SET_T1X12_C;
	TH1 = TH_Baud[baud];
	TL1 = TH_Baud[baud];
	TR1 = TRUE;
	
	SCON	=	UART_MODE1_C|EN_RCV_C;	// 8 位模式( MODE 1)
}


																				  
/********************************************/
/*          串口中断服务     	            	*/ 
/* 说明:  将收到的数据保存到接收缓冲区			*/
/********************************************/

void  SioInt(void)  interrupt 4  using 1
{
	if(RI==TRUE)
	{
		RI=FALSE;
		ga_ucRcvBuf[gi_ucSavePtr]=SBUF;												// 将数据填入缓冲区
		gi_ucSavePtr=(gi_ucSavePtr+1)&(MaxRcvByte_C-1);				// 利用屏蔽高位的方式实现指针的环形处理
		g_bNewData = TRUE;
	}
	
	if(TI == TRUE)
	{
		TI = FALSE;																						// 处理发送
		gc_ucTxdCnt--;																				// 发送计数
		if(gc_ucTxdCnt>0)
		{
			gi_ucTxdPtr++;
			SBUF = ga_ucTxdBuf[gi_ucTxdPtr];										// 取下一字节
		}
	}	
}

/********************************************/
/*名称:	dataFrame_OK												*/
/*用途: 检测接收缓冲区数据，     						*/
/*说明:	如果收到正确的数据帧则返回真				*/
/********************************************/

bit dataFrame_OK(void)
{
	unsigned char i,j,k;
	bit flag;
	
	flag = FALSE;
	
	while(gi_ucGetPtr != gi_ucSavePtr)
	{
		if(g_bStartRcv == FALSE)
		{
			/*  检测帧头 0x55 0xAA LEN */
			k = 0;			
			
			i = (gi_ucGetPtr - 5)&(MaxRcvByte_C-1);			
			if(ga_ucRcvBuf[i]==0x55)
			{
				k++;
			}
			
			i = (gi_ucGetPtr - 4)&(MaxRcvByte_C-1);			
			if(ga_ucRcvBuf[i]==0xAA)
			{
				k++;
			}
			
			i = (gi_ucGetPtr - 3)&(MaxRcvByte_C-1);			
			if(ga_ucRcvBuf[i]==MY_ADDR)
			{
				k++;
			}
			
			if(k == 3)
			{
				//帧头正确，启动数据区接收 
				g_bStartRcv=TRUE;	
				i=(gi_ucGetPtr-1)&(MaxRcvByte_C-1);						
				gc_ucDataLen = ga_ucRcvBuf[i];
				gi_ucStartPtr = gi_ucGetPtr;
				gi_ucEndPtr= (gi_ucGetPtr + gc_ucDataLen)&(MaxRcvByte_C-1); 
			}
		}
		else
		{
			//开始接收数据处理
			if(gi_ucGetPtr==gi_ucEndPtr)
			{
				/* 数据帧接收完 */
				g_bStartRcv=FALSE;
			
				j=gi_ucStartPtr;	
				k= 0;
				for(i=0;i<gc_ucDataLen;i++)
				{
					// 计算CS
					k +=ga_ucRcvBuf[j];		
					j=(j+1)&(MaxRcvByte_C-1);
				}
			
				// 取校验和
				k = ~k;
				if(k==ga_ucRcvBuf[j])
				{
					flag = TRUE;							// 数据校验正确
				}
			}
		}
		gi_ucGetPtr=(gi_ucGetPtr+1)&(MaxRcvByte_C-1);		
	}	
	return (flag);
}																					

/********************************************/
/*名称:	do_Command													*/
/*用途: 根据收到的数据帧命令做相应处理			*/
/********************************************/
/*
地址与硬件的对应关系：
0x0000 — 0x00FF —— 对应STC12LE5412的256字节内部RAM（idata）；
0x0100 — 0x01FF —— 对应STC12LE5412的256字节外部RAM（xdata）；
0x0200 — 0x7F7F —— 保留，为大RAM的单片机预留；
0x7F80 — 0x7FFF —— 对应STC12LE5412的128字节SFR；
0x8000 — 0xAFFF —— 对应STC12LE5412的12K FlashROM；
0xB000 — 0xFFFF —— 保留，为大ROM的单片机预留；
*/

// 增加电机PWM控制命令处理， 070618

void do_Command(void)
{
	unsigned char ucCommand,i,j,sum,n;
	
	union
	{
		unsigned int all;
		unsigned char b[2];
	}uitemp;
	
	unsigned char idata *ucI_Ptr;
	unsigned char xdata *ucX_Ptr;
	unsigned char code	*ucC_Ptr;

	ucCommand = ga_ucRcvBuf[gi_ucStartPtr]; 			// 取出数据帧中的命令
	
	switch (ucCommand)
	{
		case READ_MEMORY:
		{
			// 读内存数据处理
			i = (gi_ucStartPtr + 1)&(MaxRcvByte_C-1);		
			uitemp.b[1] = ga_ucRcvBuf[i];								// 取读数据地址, 注意 C51 中的多字节数据与PC中相反
			i =(i+1)&(MaxRcvByte_C-1);
			uitemp.b[0] = ga_ucRcvBuf[i];
			i =(i+1)&(MaxRcvByte_C-1);
			n = ga_ucRcvBuf[i];													// 取读数据长度
			if(n>(MaxTxdByte_C - 10))
			{
				n = (MaxTxdByte_C - 10);									// 受发送缓冲区限制，减 10 个字节对应： 
																									//	帧头2 设备地址2 长度1 命令1 数据地址2 字节数1 .... 校验和1 
			}
			
			ga_ucTxdBuf[0] = 0x55;
			ga_ucTxdBuf[1] = 0xAA;											// 帧头
			i = (gi_ucStartPtr-2)&(MaxRcvByte_C-1);			// 取发送方地址
			ga_ucTxdBuf[2] = ga_ucRcvBuf[i];						// 作为接收方地址发送
			ga_ucTxdBuf[3] = MY_ADDR;										// 自己的地址作为发送方送出
			ga_ucTxdBuf[4] = n + 4;											// 帧长
			ga_ucTxdBuf[5] = READ_MEMORY;								// 返回命令
			ga_ucTxdBuf[6] = uitemp.b[1];								// 将要读数据的地址和长度返回
			ga_ucTxdBuf[7] = uitemp.b[0];
			ga_ucTxdBuf[8] = n;
			sum = ga_ucTxdBuf[5]+ga_ucTxdBuf[6]+ga_ucTxdBuf[7]+ga_ucTxdBuf[8];

			i = 9;																			// 数据区起始指针
			
			if(uitemp.b[0] == 0x00)
			{
				// 如果高地址为 0 ，则读IDATA内容
				ucI_Ptr = uitemp.b[1];										
				for(j=0;j<n;j++)
				{
					ga_ucTxdBuf[i] = *ucI_Ptr;
					sum += ga_ucTxdBuf[i];
					i++;
					ucI_Ptr++;
				}
			}

			if(uitemp.b[0] == 0x01)
			{
				// 如果高地址为“0x01”，则读XDATA内容
				ucX_Ptr = uitemp.b[1];										// 因为只有256字节的XDATA，所以只取低字节。
				for(j=0;j<n;j++)
				{
					ga_ucTxdBuf[i] = *ucX_Ptr;
					sum += ga_ucTxdBuf[i];
					i++;
					ucX_Ptr++;
				}
			}

			// 读 SFR 暂不支持，读者可以思考一下如何添加？ 
			
			if(uitemp.b[0] >= 0x80)
			{
				// 如果高地址大于“0x80”，则读code(程序区)内容
				ucC_Ptr = uitemp.all - 0x8000;								
				for(j=0;j<n;j++)
				{
					ga_ucTxdBuf[i] = *ucC_Ptr;		// 注意，此功能将使你的程序泄密 :P
					sum += ga_ucTxdBuf[i];
					i++;
					ucC_Ptr++;
				}
			}
			
			ga_ucTxdBuf[i] = ~sum;						// 校验和
				
			gc_ucTxdCnt = i+1;								// 发送字节计数
			gi_ucTxdPtr = 0;									// 发送指针
			SBUF = ga_ucTxdBuf[0];						// 启动发送
			
			break;
		}
		
		case WRITE_MEMORY:
		{
			// 写内存数据处理
			i = (gi_ucStartPtr + 1)&(MaxRcvByte_C-1);		
			uitemp.b[1] = ga_ucRcvBuf[i];								// 取读数据地址
			i =(i+1)&(MaxRcvByte_C-1);
			uitemp.b[0] = ga_ucRcvBuf[i];
			i =(i+1)&(MaxRcvByte_C-1);
			n = ga_ucRcvBuf[i];													// 取读数据长度
			i =(i+1)&(MaxRcvByte_C-1);									// 数据区起始指针
			
			j = 0;																			// 返回实际写的字节数
			
			if(uitemp.b[0] == 0x00)
			{
				// 如果高地址为 0 ，则写IDATA内容
				ucI_Ptr = uitemp.b[1];										
				for(j=0;j<n;j++)
				{
					*ucI_Ptr = ga_ucRcvBuf[i];							// 注意，此功能会导致程序崩溃 :(
					i = (i+1)&(MaxRcvByte_C-1);						
					ucI_Ptr++;
				}
			}

			if(uitemp.b[0] == 0x01)
			{
				// 如果高地址为“0x01”，则写XDATA内容
				ucX_Ptr = uitemp.b[1];										// 因为只有256字节的XDATA，所以只取低字节。
				for(j=0;j<n;j++)
				{
					 *ucX_Ptr = ga_ucRcvBuf[i];
					i = (i+1)&(MaxRcvByte_C-1);						
					ucX_Ptr++;
				}
			}

			// 写 SFR和程序区暂不支持，读者可自己添加，看看有什么难度 :D
			
			ga_ucTxdBuf[0] = 0x55;
			ga_ucTxdBuf[1] = 0xAA;											// 帧头
			i = (gi_ucStartPtr-2)&(MaxRcvByte_C-1);			// 取发送方地址
			ga_ucTxdBuf[2] = ga_ucRcvBuf[i];						// 作为接收方地址发送
			ga_ucTxdBuf[3] = MY_ADDR;										// 自己的地址作为发送方送出
			ga_ucTxdBuf[4] = 4;													// 帧长
			ga_ucTxdBuf[5] = WRITE_MEMORY;							// 返回命令
			ga_ucTxdBuf[6] = uitemp.b[1];								// 将要读数据的地址和长度返回
			ga_ucTxdBuf[7] = uitemp.b[0];
			ga_ucTxdBuf[8] = j;													// 返回写成功的字节数
			sum = ga_ucTxdBuf[5]+ga_ucTxdBuf[6]+ga_ucTxdBuf[7]+ga_ucTxdBuf[8];
			
			ga_ucTxdBuf[9] = ~sum;						// 校验和
				
			gc_ucTxdCnt = 10;									// 发送字节计数
			gi_ucTxdPtr = 0;									// 发送指针
			SBUF = ga_ucTxdBuf[0];						// 启动发送
			
			break;
		}
		
		case MOTOR_PWM_CTRL:
		{
			// 电机PWM控制  StepbyStep之三增加 070618
			i = (gi_ucStartPtr + 1)&(MaxRcvByte_C-1);		
			uitemp.b[1] = ga_ucRcvBuf[i];								// 取左侧电机数据, 注意 C51 中的多字节数据与PC中相反
			i =(i+1)&(MaxRcvByte_C-1);
			uitemp.b[0] = ga_ucRcvBuf[i];
			ga_uiMotorCtrl[MOTOR_L] = uitemp.all;
			
			i =(i+1)&(MaxRcvByte_C-1);
			uitemp.b[1] = ga_ucRcvBuf[i];								// 取右侧电机数据, 注意 C51 中的多字节数据与PC中相反
			i =(i+1)&(MaxRcvByte_C-1);
			uitemp.b[0] = ga_ucRcvBuf[i];
			ga_uiMotorCtrl[MOTOR_R] = uitemp.all;

			j = DriveMotor(MOTOR_L,ga_uiMotorCtrl[MOTOR_L]);	// 输出电机控制 
			j = DriveMotor(MOTOR_R,ga_uiMotorCtrl[MOTOR_R]);	
			
			ga_ucTxdBuf[0] = 0x55;
			ga_ucTxdBuf[1] = 0xAA;											// 帧头
			i = (gi_ucStartPtr-2)&(MaxRcvByte_C-1);			// 取发送方地址
			ga_ucTxdBuf[2] = ga_ucRcvBuf[i];						// 作为接收方地址发送
			ga_ucTxdBuf[3] = MY_ADDR;										// 自己的地址作为发送方送出
			ga_ucTxdBuf[4] = 2;													// 帧长
			ga_ucTxdBuf[5] = MOTOR_PWM_CTRL;						// 返回命令
			ga_ucTxdBuf[6] = j;													// 返回 P2 控制字
			
			sum = ga_ucTxdBuf[5]+ga_ucTxdBuf[6];
			
			ga_ucTxdBuf[7] = ~sum;						// 校验和
				
			gc_ucTxdCnt = 8;									// 发送字节计数
			gi_ucTxdPtr = 0;									// 发送指针
			SBUF = ga_ucTxdBuf[0];						// 启动发送
		
			break;
		}
		
		case RUN_STRAIGHT:
		{
			// 启动走直线  StepbyStep之四增加 070622
			
			i = (gi_ucStartPtr + 1)&(MaxRcvByte_C-1);		
			uitemp.b[1] = ga_ucRcvBuf[i];								// 取左轮行走设定值, 注意 C51 中的多字节数据与PC中相反
			i =(i+1)&(MaxRcvByte_C-1);
			uitemp.b[0] = ga_ucRcvBuf[i];
			g_uiLeftRunNum = uitemp.all;
			gc_uiLeftRunCnt = g_uiLeftRunNum;						// 启动行走距离控制
			
			i =(i+1)&(MaxRcvByte_C-1);
			uitemp.b[1] = ga_ucRcvBuf[i];								// 取基准PWM值, 注意 C51 中的多字节数据与PC中相反
			i =(i+1)&(MaxRcvByte_C-1);
			uitemp.b[0] = ga_ucRcvBuf[i];
			g_uiBase_PWM = uitemp.all;
						
			ga_uiMotorCtrl[MOTOR_L] = g_uiBase_PWM;			// 保存电机控制 PWM 值
			ga_uiMotorCtrl[MOTOR_R] = g_uiBase_PWM;

			gac_uiRunCnt[MOTOR_L] = 0;									// 启动时清除行走计数值
			gac_uiRunCnt[MOTOR_R] = 0;			

			if(g_uiBase_PWM == BRAKE_PWM)
			{
				g_bStopRunStraight = TRUE;
			}
			else
			{
				g_bStopRunStraight = FALSE;									// 启动
			}
			
			DriveMotor(MOTOR_L,ga_uiMotorCtrl[MOTOR_L]);	// 输出电机控制 
			DriveMotor(MOTOR_R,ga_uiMotorCtrl[MOTOR_R]);	
			
						
			ga_ucTxdBuf[0] = 0x55;
			ga_ucTxdBuf[1] = 0xAA;											// 帧头
			i = (gi_ucStartPtr-2)&(MaxRcvByte_C-1);			// 取发送方地址
			ga_ucTxdBuf[2] = ga_ucRcvBuf[i];						// 作为接收方地址发送
			ga_ucTxdBuf[3] = MY_ADDR;										// 自己的地址作为发送方送出
			ga_ucTxdBuf[4] = 1;													// 帧长
			ga_ucTxdBuf[5] = RUN_STRAIGHT;							// 返回命令
			
			sum = ga_ucTxdBuf[5];
			
			ga_ucTxdBuf[6] = ~sum;						// 校验和
				
			gc_ucTxdCnt = 7;									// 发送字节计数
			gi_ucTxdPtr = 0;									// 发送指针
			SBUF = ga_ucTxdBuf[0];						// 启动发送
			
			break;
		}

		case RUN_ON_LINE:
		{
			// 启动走轨迹  StepbyStep之五增加 070715
			
			i = (gi_ucStartPtr + 1)&(MaxRcvByte_C-1);		
			uitemp.b[1] = ga_ucRcvBuf[i];								// 取左轮行走设定值, 注意 C51 中的多字节数据与PC中相反
			i =(i+1)&(MaxRcvByte_C-1);
			uitemp.b[0] = ga_ucRcvBuf[i];
			g_uiLeftRunNum = uitemp.all;
			gc_uiLeftRunCnt = g_uiLeftRunNum;						// 启动行走距离控制
			
			i =(i+1)&(MaxRcvByte_C-1);
			uitemp.b[1] = ga_ucRcvBuf[i];								// 取基准PWM值, 注意 C51 中的多字节数据与PC中相反
			i =(i+1)&(MaxRcvByte_C-1);
			uitemp.b[0] = ga_ucRcvBuf[i];
			g_uiBase_PWM = uitemp.all;
						
			ga_uiMotorCtrl[MOTOR_L] = g_uiBase_PWM;			// 保存电机控制 PWM 值
			ga_uiMotorCtrl[MOTOR_R] = g_uiBase_PWM;

			gac_uiRunCnt[MOTOR_L] = 0;									// 启动时清除行走计数值
			gac_uiRunCnt[MOTOR_R] = 0;			

			if(g_uiBase_PWM == BRAKE_PWM)
			{
				g_bStopRunOnLine = TRUE;
			}
			else
			{
				g_bStopRunOnLine = FALSE;													// 启动
			}
			
			DriveMotor(MOTOR_L,ga_uiMotorCtrl[MOTOR_L]);	// 输出电机控制 
			DriveMotor(MOTOR_R,ga_uiMotorCtrl[MOTOR_R]);	
			
						
			ga_ucTxdBuf[0] = 0x55;
			ga_ucTxdBuf[1] = 0xAA;											// 帧头
			i = (gi_ucStartPtr-2)&(MaxRcvByte_C-1);			// 取发送方地址
			ga_ucTxdBuf[2] = ga_ucRcvBuf[i];						// 作为接收方地址发送
			ga_ucTxdBuf[3] = MY_ADDR;										// 自己的地址作为发送方送出
			ga_ucTxdBuf[4] = 1;													// 帧长
			ga_ucTxdBuf[5] = RUN_ON_LINE;								// 返回命令
			
			sum = ga_ucTxdBuf[5];
			
			ga_ucTxdBuf[6] = ~sum;						// 校验和
				
			gc_ucTxdCnt = 7;									// 发送字节计数
			gi_ucTxdPtr = 0;									// 发送指针
			SBUF = ga_ucTxdBuf[0];						// 启动发送
			
			break;
		}
		
		default: break;
	}
}

// -------- 以下为StepByStep之三增加的函数 070618 ------------------

/********************************************/
/*名称:	DriveMotor													*/
/*用途: 根据得到的参数计算电机控制输出值，	*/
/********************************************/
//输入 No ——  电机序号， iPWM_Val —— 电机控制值 
// 返回电机控制信号

unsigned char DriveMotor(unsigned char No,unsigned int uiPWM_Val)
{
	unsigned char ucCtrl_Out,ucCtrl_Buf,ucPCA_CCAP,ucPCA_MODE;
	union
	{
		unsigned int all;
		unsigned char b[2];
	}uiPWM_Buf;
	
	uiPWM_Buf.all = uiPWM_Val;
	ucCtrl_Out = MotorDrv;								// 读回电机控制输出，注意，此操作会影响P2.3和P2.7口的状态！！！
	
	switch(uiPWM_Buf.b[1])
	{
		case FLOAT_PWM:
		{
			// 惰行命令
			ucCtrl_Buf = ga_ucFloat[No];
			ucPCA_MODE = 0x03;								// PWM恒输出 0， 
			ucPCA_CCAP = 0xFF;
			break;
		}
		
		case BRAKE_PWM:
		{		
			// 刹车命令
			ucCtrl_Buf = ga_ucBrake[No];
			ucPCA_MODE = 0x03;								// PWM恒输出 0， 
			ucPCA_CCAP = 0xFF;
			break;
		}
		
		default:
		{
			// 调功处理
			if(uiPWM_Buf.b[0] == 0)
			{
				// 前进
				ucCtrl_Buf = ga_ucForward[No];
			}
			else
			{
				// 后退控制
				ucCtrl_Buf = ga_ucBack[No];
			}
			
			ucPCA_MODE = 0x00;
			ucPCA_CCAP = 255 - uiPWM_Buf.b[1];			// PWM值, 注意：需要用 255 减，否则值越大功率越小！！ 070622
			
			break;
		}
	}	
	
	ucCtrl_Out = (ucCtrl_Out&ga_ucMask[No])|ucCtrl_Buf;
	MotorDrv = ucCtrl_Out;							// 	输出控制信号， 如果使用P2.3和P2.7 则要特殊处理
	
	if(No == MOTOR_L)
	{
		PCA_PWM2 = ucPCA_MODE;
		CCAP2L = ucPCA_CCAP;
		CCAP2H = ucPCA_CCAP;
	}
	else
	{
		PCA_PWM3 = ucPCA_MODE;
		CCAP3L = ucPCA_CCAP;
		CCAP3H = ucPCA_CCAP;
	}
	
	return (ucCtrl_Out);								// 将电机控制信号返回
}

// -------- 以下为StepByStep之四增加的函数 070622 ------------------

/********************************************/
/*          PCA 中断服务  			            */ 
/* 说明: CCF0、1 用于码盘输入								*/
/*			 CCF2、3 用于电机控制								*/
/********************************************/

void  PCA_Int(void) interrupt 6 using 2
{	
	if(CF == TRUE)
	{
		CF = FALSE;												// 	出错保护
	}
	
	if(CCF0 == TRUE)
	{
		// 右侧码盘信号输入
		CCF0 = FALSE;
		gac_ucWheel_Cnt[MOTOR_R]++;				// 每来一个脉冲 +1
	}
	
	if(CCF1 == TRUE)
	{
		// 左侧码盘信号输入
		CCF1 = FALSE;
		gac_ucWheel_Cnt[MOTOR_L]++;				// 每来一个脉冲 +1
	}
	
	if(CCF2 == TRUE)
	{
		CCF2 = FALSE;										// 	出错保护
	}
	
	if(CCF3 == TRUE)
	{
		CCF3 = FALSE;										// 	出错保护
	}		
}

/********************************************/
/*名称:	run_Straight												*/
/*用途: 根据车轮行走计数修正电机输出值，		*/
/********************************************/
/*
涉及如下全局变量：
gac_uiRunCnt[2] —— 两轮的行走计数值
ga_uiMotorCtrl[MOTOR_R] ——  右侧电机的PWM控制值
*/

void run_Straight(void)
{
	bit flag;
		
	flag = FALSE;
			
	if(gac_uiRunCnt[MOTOR_R]!= gac_uiRunCnt[MOTOR_L])
	{
		if(gac_uiRunCnt[MOTOR_R]> gac_uiRunCnt[MOTOR_L])
		{
			// 右轮快，右轮置为惰行
			ga_uiMotorCtrl[MOTOR_R] = FLOAT_PWM;					
		}
		else
		{
			// 左轮快，左轮置为惰行
			ga_uiMotorCtrl[MOTOR_L] = FLOAT_PWM;					
		}
		
		flag =TRUE;				
	}
	else
	{
		if(ga_uiMotorCtrl[MOTOR_L] == FLOAT_PWM)
		{
			ga_uiMotorCtrl[MOTOR_L] = g_uiBase_PWM;
			flag = TRUE;
		}
		
		if(ga_uiMotorCtrl[MOTOR_R] == FLOAT_PWM)
		{
			ga_uiMotorCtrl[MOTOR_R] = g_uiBase_PWM;
			flag = TRUE;
		}
	}
	
	if(flag == TRUE)
	{
		DriveMotor(MOTOR_R,ga_uiMotorCtrl[MOTOR_R]);	// 输出电机控制
		DriveMotor(MOTOR_L,ga_uiMotorCtrl[MOTOR_L]);		
	}
}

/********************************************/
/*名称:	StopRunStraight											*/
/*用途: 停止直线行走状态										*/
/********************************************/

void StopRunStraight(void)
{
	g_bStopRunStraight = TRUE;														// 停止走直线控制
	
	DriveMotor(MOTOR_R,BRAKE_PWM);								// 输出电机控制
	DriveMotor(MOTOR_L,BRAKE_PWM);	
}

// -------- 以下为StepByStep之五增加的函数 070715 ------------------

/********************************************/
/* 名称：lineSamp_proc											*/
/* 用途：采样并处理													*/
/********************************************/
/* 采样器对应关系：
	 顶视：  			左  ——  右
	 采样器  			1  2  3  4
	 PORT P1.			3  2  1  0
	 SampleVal		1  2  3  4
	 SensorStat   b0 b1 b2 b3
	 OldSenStat   b0 b1 b2 b3
*/

unsigned char lineSamp_proc(unsigned char ucOldSenStat)
{
	unsigned char i,temp;
	
	// 启动 AD 并读数, 大约占用 56 us, 对应 22.1184MHz

	for(i = 0; i <4; i++)
	{
		// 循环采样
		ADC_CONTR = ADPWRON_C|ADSPEED3_C|STARTAD_C|i;			// 启动 AD
		
		temp = ADC_CONTR & GETADFLAG_C;										// 读转换结束标志
		while(temp==0)
		{
			temp = ADC_CONTR & GETADFLAG_C;									// 读转换结束标志
		}

		ga_ucSampleVal[3-i] = ADC_DATA;										// 获取AD值, 暂时取 8 位值, 此处将顺序反过来！！！
		
		ADC_CONTR = ADC_CONTR & CLRADFLAG_C;
	}
		
	//--- 根据判断值将数据转换为逻辑值 -----
	
	temp = 0;
	ucOldSenStat = ucOldSenStat<<1;				// 为了循环预处理
	
	for(i=0;i<4;i++)
	{
		temp = temp>>1;
		ucOldSenStat = ucOldSenStat>>1;
		
		if(ucOldSenStat&0x01)
		{
			// 原来在轨迹上
			if(ga_ucSampleVal[i]<(ga_ucThreshold[i]+ g_ucThresDelta))	// 大于回差才算离开轨迹
			{
				temp = temp|0x08;																				// 小于则仍看成在轨迹上
			}
		}
		else
		{
			// 原来不在轨迹上
			if(ga_ucSampleVal[i]<(ga_ucThreshold[i]-g_ucThresDelta))	// 则必须小于回差才算进入轨迹
			{
				temp = temp|0x08;
			}
		}																						// 这一段有些绕，可以借助PC界面琢磨一下 ^-^
	}
		
	return(temp);																	// 得到采样器的逻辑状态
}

/********************************************/
/* 名称：run_Online													*/
/* 用途: 根据	ucSensorStat控制小车				*/
/********************************************/

/*
	涉及全局变量：
	ga_uiMotorCtrl[2]

	采样器对应关系：( 万向轮为小车前）
	 顶视：  			左  ——  右
	 采样器  			1  2  3  4
	 SensorStat   b0 b1 b2 b3
	 控制逻辑：
	 只处理 2、3号传感器，即位 b1、b2,
	 两个均在轨迹上，左右电机为同样值，
	 b1 不在，右侧电机惰行，
	 b2 不在，左侧电机惰行。
	 和走直线类似，只是控制变量从码盘计数差变为轨迹采样的状态。
*/

void run_Online(unsigned char ucSensorStat)
{
	bit flag;
		
	flag = FALSE;
			
	if((ucSensorStat&0x06) != 0x06)
	{
		// 有一个不在轨迹上
		if((ucSensorStat&0x02) != 0x02)
		{
			// 2号（左侧）传感器不在，右轮置为刹车
			ga_uiMotorCtrl[MOTOR_R] = BRAKE_PWM;					
		}
		else
		{
			// 3号（左侧）传感器不在，左轮置为刹车
			ga_uiMotorCtrl[MOTOR_L] = BRAKE_PWM;					
		}
		
		flag =TRUE;				
	}
	else
	{
		// 2 个都在，将刹车电机恢复
		if(ga_uiMotorCtrl[MOTOR_L] == BRAKE_PWM)
		{
			ga_uiMotorCtrl[MOTOR_L] = g_uiBase_PWM;
			flag = TRUE;
		}
		
		if(ga_uiMotorCtrl[MOTOR_R] == BRAKE_PWM)
		{
			ga_uiMotorCtrl[MOTOR_R] = g_uiBase_PWM;
			flag = TRUE;
		}
	}
	
	if(flag == TRUE)
	{
		DriveMotor(MOTOR_R,ga_uiMotorCtrl[MOTOR_R]);	// 输出电机控制
		DriveMotor(MOTOR_L,ga_uiMotorCtrl[MOTOR_L]);		
	}	
}

/********************************************/
/*名称:	StopRunOnLine												*/
/*用途: 停止走轨迹													*/
/********************************************/

void StopRunOnLine(void)
{
	g_bStopRunOnLine = TRUE;											// 停止走轨迹
	
	DriveMotor(MOTOR_R,BRAKE_PWM);								// 输出电机控制
	DriveMotor(MOTOR_L,BRAKE_PWM);	
}

