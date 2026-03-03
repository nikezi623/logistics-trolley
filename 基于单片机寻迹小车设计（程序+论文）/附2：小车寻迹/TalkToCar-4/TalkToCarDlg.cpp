// TalkToCarDlg.cpp : implementation file
//

#include "stdafx.h"
#include "TalkToCar.h"
#include "TalkToCarDlg.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

// 符号化常数定义：

// 小车通讯协议中的收、发方地址
const BYTE CAR_ADDR = 0x01;				// 暂定的小车地址
const BYTE PC_ADDR = 0xFE;				// PC机的地址，0xFF留作广播地址

// 小车通讯协议中的命令字
const BYTE READ_MEMORY = 1;				// 读内存命令字
const BYTE WRITE_MEMORY = 2;			// 写内存命令字
const BYTE MOTOR_PWM_CTRL = 3;		// 电机PWM控制
const BYTE RUN_STRAIGHT = 4;			// 走直线
const BYTE RUN_ON_LINE = 5;				// 走轨迹

// PC程序中自己用的命令代号
const int READ_BYTE = 1;					// 读一个字节命令
const int WRITE_BYTE = 2;					// 写一个字节命令
const int MOTOR_PWM = 3;					// 电机PWM控制
const int RUN_STR = 4;						// 走直线测试
const int READ_SAMPLE = 5;				// 读取轨迹采样数据

// 接收处理状态
const int NO_RCV = 0;
const int WAIT_FRAME_END = 1;

// 电机控制选项
const int FORWARD = 0;						// 对应下拉选项的定义
const int BACKWARD = 1;
const int FLOAT_C = 2;
const int BRAKE_C = 3;

// 特殊PWM 控制值定义
const BYTE FLOAT_PWM = 255;
const BYTE BRAKE_PWM =0;

// 读内存的内容说明 （因为需要用读内存命令处理多种需求）
const int GENERAL_DATA = 0;				// 无特殊意义的数据
const int SAMPLE_DATA = 1;				// 轨迹采样数据
		 
/////////////////////////////////////////////////////////////////////////////
// CAboutDlg dialog used for App About

class CAboutDlg : public CDialog
{
public:
	CAboutDlg();

// Dialog Data
	//{{AFX_DATA(CAboutDlg)
	enum { IDD = IDD_ABOUTBOX };
	//}}AFX_DATA

	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CAboutDlg)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:
	//{{AFX_MSG(CAboutDlg)
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialog(CAboutDlg::IDD)
{
	//{{AFX_DATA_INIT(CAboutDlg)
	//}}AFX_DATA_INIT
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CAboutDlg)
	//}}AFX_DATA_MAP
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialog)
	//{{AFX_MSG_MAP(CAboutDlg)
		// No message handlers
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CTalkToCarDlg dialog

CTalkToCarDlg::CTalkToCarDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CTalkToCarDlg::IDD, pParent)
{
	//{{AFX_DATA_INIT(CTalkToCarDlg)
	m_ucWriteData = 0;
	m_ucMotorL_PWM = 0;
	m_ucMotorR_PWM = 0;
	m_ucBase_PWM = 0;
	m_uiLeftRunNum = 0;
	m_iReadData = 0;
	m_szDataAddr = _T("");
	m_ucSampleVal1 = 0;
	m_ucSampleVal2 = 0;
	m_ucSampleVal3 = 0;
	m_ucSampleVal4 = 0;
	m_ucSensorStat = 0;
	m_szSampleAddr = _T("");
	m_szCommandBack = _T("");
	//}}AFX_DATA_INIT
	// Note that LoadIcon does not require a subsequent DestroyIcon in Win32
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CTalkToCarDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CTalkToCarDlg)
	DDX_Control(pDX, IDC_RUN_MODE, m_ctrlRunMode);
	DDX_Control(pDX, IDC_DATA_TYPE, m_ctrlDataType);
	DDX_Control(pDX, IDC_RUN_DIR, m_ctrlRunDir);
	DDX_Control(pDX, IDC_MOTORR_CTRL, m_ctrlMotor_R);
	DDX_Control(pDX, IDC_MOTORL_CTRL, m_ctrlMotor_L);
	DDX_Control(pDX, IDC_SEL_COMMPORT, m_ctrlSelCommPort);
	DDX_Control(pDX, IDC_MSCOMM1, m_ctrlCommToCar);
	DDX_Text(pDX, IDC_WRITE_DATA, m_ucWriteData);
	DDX_Text(pDX, IDC_MOTORL_PWM, m_ucMotorL_PWM);
	DDV_MinMaxByte(pDX, m_ucMotorL_PWM, 0, 250);
	DDX_Text(pDX, IDC_MOTORR_PWM, m_ucMotorR_PWM);
	DDV_MinMaxByte(pDX, m_ucMotorR_PWM, 0, 250);
	DDX_Text(pDX, IDC_BASE_PWM, m_ucBase_PWM);
	DDX_Text(pDX, IDC_LEFT_RUN_NUM, m_uiLeftRunNum);
	DDV_MinMaxUInt(pDX, m_uiLeftRunNum, 0, 65535);
	DDX_Text(pDX, IDC_READ_DATA, m_iReadData);
	DDX_Text(pDX, IDC_DATA_ADDR, m_szDataAddr);
	DDX_Text(pDX, IDC_SAMPLE_VAL1, m_ucSampleVal1);
	DDX_Text(pDX, IDC_SAMPLE_VAL2, m_ucSampleVal2);
	DDX_Text(pDX, IDC_SAMPLE_VAL3, m_ucSampleVal3);
	DDX_Text(pDX, IDC_SAMPLE_VAL4, m_ucSampleVal4);
	DDX_Text(pDX, IDC_SENSOR_STAT, m_ucSensorStat);
	DDX_Text(pDX, IDC_SAMPLE_ADDR, m_szSampleAddr);
	DDX_Text(pDX, IDC_COMMAND_BACK, m_szCommandBack);
	//}}AFX_DATA_MAP
}

BEGIN_MESSAGE_MAP(CTalkToCarDlg, CDialog)
	//{{AFX_MSG_MAP(CTalkToCarDlg)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_CBN_SELCHANGE(IDC_SEL_COMMPORT, OnSelCommPort)
	ON_BN_CLICKED(IDC_READ, OnRead)
	ON_BN_CLICKED(IDC_WRITE, OnWrite)
	ON_BN_CLICKED(IDC_PWM_OUT, OnPWM_Out)
	ON_BN_CLICKED(IDC_BRAKE, OnBrake)
	ON_BN_CLICKED(IDC_FLOAT, OnFloat)
	ON_BN_CLICKED(IDC_RUN_STRAIGHT, OnRunStraight)
	ON_BN_CLICKED(IDC_READ_SAMPLE, OnReadSample)
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CTalkToCarDlg message handlers

BOOL CTalkToCarDlg::OnInitDialog()
{
	CDialog::OnInitDialog();

	// Add "About..." menu item to system menu.

	// IDM_ABOUTBOX must be in the system command range.
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		CString strAboutMenu;
		strAboutMenu.LoadString(IDS_ABOUTBOX);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// Set the icon for this dialog.  The framework does this automatically
	//  when the application's main window is not a dialog
	SetIcon(m_hIcon, TRUE);			// Set big icon
	SetIcon(m_hIcon, FALSE);		// Set small icon
	
	// TODO: Add extra initialization here
	GetDlgItem(IDC_READ) -> EnableWindow(false);				// 初始化时将两个按钮禁止，等串口有效后再允许。
	GetDlgItem(IDC_WRITE) -> EnableWindow(false);
	GetDlgItem(IDC_FLOAT) -> EnableWindow(false);
	GetDlgItem(IDC_BRAKE) -> EnableWindow(false);
	GetDlgItem(IDC_PWM_OUT) -> EnableWindow(false);
	GetDlgItem(IDC_RUN_STRAIGHT) -> EnableWindow(false);
	GetDlgItem(IDC_READ_SAMPLE) -> EnableWindow(false);

	m_ucGetBack = 0;
	m_ucSaveBack = 0;																		// 初始化存取数指针，等效于缓冲区为空

	return TRUE;  // return TRUE  unless you set the focus to a control
}

void CTalkToCarDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialog::OnSysCommand(nID, lParam);
	}
}

// If you add a minimize button to your dialog, you will need the code below
//  to draw the icon.  For MFC applications using the document/view model,
//  this is automatically done for you by the framework.

void CTalkToCarDlg::OnPaint() 
{
	if (IsIconic())
	{
		CPaintDC dc(this); // device context for painting

		SendMessage(WM_ICONERASEBKGND, (WPARAM) dc.GetSafeHdc(), 0);

		// Center icon in client rectangle
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// Draw the icon
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialog::OnPaint();
	}
}

// The system calls this to obtain the cursor to display while the user drags
//  the minimized window.
HCURSOR CTalkToCarDlg::OnQueryDragIcon()
{
	return (HCURSOR) m_hIcon;
}

// 选择串口处理，测试所选择的串口是否存在、可用，使用时再打开。

void CTalkToCarDlg::OnSelCommPort() 
{
	// TODO: Add your control notification handler code here
	int PortNo;

	if(m_ctrlCommToCar.GetPortOpen())							//如果串口开着
	{
		m_ctrlCommToCar.SetPortOpen(false);
	}

	PortNo = m_ctrlSelCommPort.GetCurSel() + 1;

	m_ctrlCommToCar.SetCommPort(PortNo);					//打开串口

	if(!m_ctrlCommToCar.GetPortOpen())						//判断打开是否成功,不成功的话系统会自动弹出错误信息
	{
		m_ctrlCommToCar.SetPortOpen(true);
	}


	GetDlgItem(IDC_READ) -> EnableWindow(true);
	GetDlgItem(IDC_WRITE) -> EnableWindow(true);
	GetDlgItem(IDC_FLOAT) -> EnableWindow(true);
	GetDlgItem(IDC_BRAKE) -> EnableWindow(true);
	GetDlgItem(IDC_PWM_OUT) -> EnableWindow(true);
	GetDlgItem(IDC_RUN_STRAIGHT) -> EnableWindow(true);
	GetDlgItem(IDC_READ_SAMPLE) -> EnableWindow(true);


	m_ctrlCommToCar.SetPortOpen(false);						// 此处只是确认一下口是否可用，到实际用时再打开！	
}

BEGIN_EVENTSINK_MAP(CTalkToCarDlg, CDialog)
    //{{AFX_EVENTSINK_MAP(CTalkToCarDlg)
	ON_EVENT(CTalkToCarDlg, IDC_MSCOMM1, 1 /* OnComm */, OnData_T_R, VTS_NONE)
	//}}AFX_EVENTSINK_MAP
END_EVENTSINK_MAP()

// 串口收到数据处理
void CTalkToCarDlg::OnData_T_R() 
{
	// TODO: Add your control notification handler code here
	VARIANT rcv_temp;
	COleSafeArray rcv_temp1;
	long len,k;

	union
	{
		UINT all;
		BYTE b[4];
	}uitemp;

	int	iCommand;

	if(m_ctrlCommToCar.GetCommEvent() == 2)
	{
		// 收到数据
		rcv_temp = m_ctrlCommToCar.GetInput();
		rcv_temp1 = rcv_temp;										// 转换类型
		len = rcv_temp1.GetOneDimSize();				// 得到接收的字节数

		for(k=0;k<len;k++)											// 将数据复制到接收缓冲区
		{
			rcv_temp1.GetElement(&k,&m_ucBackBuf[m_ucSaveBack]);
			m_ucSaveBack++;
		}

		
		iCommand = DataFrame_OK();							// 检测是否为有效帧，

		if(iCommand != -1)
		{
			m_szCommandBack = "成功";							// 增加此显示以便判断小车是否有返回帧
		
			switch(iCommand)												// 因为现在命令较少，故直接在此处理
			{
				case READ_MEMORY:
				{
					uitemp.all = 0;
					uitemp.b[0] = m_ucBackBuf[m_ucStartPtr+1];
					uitemp.b[1] = m_ucBackBuf[m_ucStartPtr+2];

					switch(m_iReadMemContent)
					{
						case GENERAL_DATA:
						{
							gatGeneralData();	
							break;
						}

						case SAMPLE_DATA:
						{
							getSampleData();
							break;
						}

						default: break;
					}

					break;
				}

				case WRITE_MEMORY:
				{
					uitemp.all = 0;
					uitemp.b[0] = m_ucBackBuf[m_ucStartPtr+1];
					uitemp.b[1] = m_ucBackBuf[m_ucStartPtr+2];
					if(m_ucBackBuf[m_ucStartPtr+3] == 0)
					{
						m_ucWriteData = 0xFF;											// 如果写失败，则将写数据显示为 255
					}
					break;
				}

				default: break;
			}

			this->UpdateData(false);										// s刷新显示
		}
	}
}


// 检测接收缓冲区中是否收到有效帧，如果是，则返回帧中的命令字；
// 没有收到合法帧， 则返回“0”；
// 收到检验和错误帧则返回 “-1”。

// 帧格式：
// 帧头（2字节） 接收方地址（1字节） 发送方地址（1字节） 帧长（1字节） 
// 命令（1字节） 数据域（N字节） 校验和（1字节）
// 校验和 -- 数据帧中从命令开始到数据域结束所有字节的算术和，取最低字节的反码。

int CTalkToCarDlg::DataFrame_OK()
{
	int returnval = -1;
	int rcv_chk,i;
	BYTE sum,j;
	
	while(m_ucGetBack != m_ucSaveBack)
	{
		switch (m_iRcvStat)
		{
			case NO_RCV:
			{		
				// 检测帧头
				rcv_chk = 0;
				if(m_ucBackBuf[m_ucGetBack-5] == 0x55)
				{
					rcv_chk++;
				}
			
				if(m_ucBackBuf[m_ucGetBack-4] == 0xAA)
				{
					rcv_chk++;
				}

				if(m_ucBackBuf[m_ucGetBack-3] == PC_ADDR)
				{
					rcv_chk++;
				}

				if(rcv_chk == 3)
				{
					m_ucStartPtr = m_ucGetBack;
					m_ucEndPtr = m_ucGetBack + m_ucBackBuf[m_ucGetBack-1];
					m_iRcvStat = WAIT_FRAME_END;	
				}
				break;
			}

			case WAIT_FRAME_END:
			{
				if(m_ucEndPtr == m_ucGetBack)
				{
					m_iRcvStat = NO_RCV;											// 恢复初始状态

					sum =0;																		// 检查校验和
					j=m_ucStartPtr;
					for(i=0;i<m_ucBackBuf[m_ucStartPtr-1];i++)
					{
						sum +=m_ucBackBuf[j];
						j++;
					}
					sum = ~sum;
					if(sum == m_ucBackBuf[j])
					{
						returnval = m_ucBackBuf[m_ucStartPtr];		// 返回命令字
					}
					else
					{
						returnval = -1;
					}

					m_ctrlCommToCar.SetPortOpen(false);				// 收完数据帧后关闭串口，释放设备。
				}
				break;
			}

			default:	break;
		}

		m_ucGetBack++;
	}

	return(returnval);
}


// 读按钮操作，打开串口，构建命令帧，启动发送

void CTalkToCarDlg::OnRead() 
{
	// TODO: Add your control notification handler code here

	this->UpdateData(true);				// 将对话框中的地址等数据取回

	if(open_CommPort())
	{
		m_iReadMemContent = GENERAL_DATA;
		makeSendFrame(READ_BYTE, &m_baSendFrame);									// 构建发送帧

		m_iRcvStat = NO_RCV;																			// 初始化接收状态
		m_ctrlCommToCar.SetOutput((COleVariant)m_baSendFrame);		// 启动发送
		m_baSendFrame.RemoveAll();
	}
	else
	{
		m_ctrlCommToCar.SetPortOpen(false);
	}	
}

// 写按钮操作，打开串口，构建命令帧，启动发送

void CTalkToCarDlg::OnWrite() 
{
	// TODO: Add your control notification handler code here
	this->UpdateData(true);				// 将对话框中的地址等数据取回

	if(open_CommPort())
	{
		makeSendFrame(WRITE_BYTE, &m_baSendFrame);							// 构建发送帧

		m_iRcvStat = NO_RCV;																		// 初始化接收状态
		m_ctrlCommToCar.SetOutput((COleVariant)m_baSendFrame);	// 启动发送
		m_baSendFrame.RemoveAll();
	}
	else
	{
		m_ctrlCommToCar.SetPortOpen(false);
	}		
}


// 这是一个自己添加的函数，用鼠标选中类“CTalkToCarDlg”，
// 右键中选择“Add Member Function..."即可

// 函数功能：打开 m_ctrlSelCommPort 中所选的串口，成功返回真。

bool CTalkToCarDlg::open_CommPort()
{
	CString CommConfig;

	int PortNo;

	this->UpdateData(true);	

	
	if(m_ctrlCommToCar.GetPortOpen())							//如果串口开着
	{
		m_ctrlCommToCar.SetPortOpen(false);
	}

	PortNo = m_ctrlSelCommPort.GetCurSel() + 1;

	m_ctrlCommToCar.SetCommPort(PortNo);			//打开串口

	if(!m_ctrlCommToCar.GetPortOpen())				//判断打开是否成功,不成功的话系统会自动弹出错误信息
	{
		m_ctrlCommToCar.SetPortOpen(true);
	}

	CommConfig = _T("19200,N,8,1");						//传输率,无校验,8个数据位,1个停止位
	m_ctrlCommToCar.SetSettings(CommConfig);				//设置串口
	
	//参数1表示接收缓冲中有1个或多于1个字符时,引发一个接收数据的OnComm事件
	m_ctrlCommToCar.SetRThreshold(1);
	
	m_ctrlCommToCar.SetRTSEnable(true);				// 设置RTS为可以发送
	m_ctrlCommToCar.SetInputMode(1);					// 0 表示以文本格式接收,1表示以二进制方式接收
	m_ctrlCommToCar.SetInputLen(0);						// 设置当前接收区数据长度为0
	m_ctrlCommToCar.GetInput();								// 先预读缓冲区以清除残留数据	

	m_ucGetBack = m_ucSaveBack;								// 初始化接收指针
	return(true);
}

// 根据命令取相应数据，构建发送数据帧

void CTalkToCarDlg::makeSendFrame(int iCommand, CByteArray *sendFrame)
{
	union
	{
		UINT all;
		BYTE b[4];
	}uitemp;
  
	int i;
	unsigned short ushDataAddr;

	BYTE sum = 0;

	sendFrame->RemoveAll();
	sendFrame->Add(0x55);
	sendFrame->Add(0xAA);										// 初始化帧头
	sendFrame->Add(CAR_ADDR);
	sendFrame->Add(PC_ADDR);

	switch (iCommand)
	{
		case READ_BYTE:
		{
			sendFrame->Add(0x04);								// 帧长
			sendFrame->Add(READ_MEMORY);				// 命令字
			sum += READ_MEMORY;

			// 将输入的 16 进制地址转换， 因为M51文件中的变量地址通常为 16进制
			m_szDataAddr.MakeUpper();
			ushDataAddr = HexString2ush(m_szDataAddr);

			uitemp.all = ushDataAddr;						// 数据地址
			sendFrame->Add(uitemp.b[0]);				// 低字节
			sum += uitemp.b[0];						
			sendFrame->Add(uitemp.b[1]);				// 高字节, 只取低 2 字节
			sum += uitemp.b[1];						

			i = m_ctrlDataType.GetCurSel();			// 读数据类型处理
			if( i<= 1)
			{
				sendFrame->Add(0x01);							// 读有无符号字节的字节数
				sum += 0x01;
			}
			else
			{
				if(i <= 3)
				{
					sendFrame->Add(0x02);						// 读有无符号整形的字节数
					sum += 0x02;
				}
				else
				{
					sendFrame->Add(0x04);						// 读有符号长整的字节数
					sum += 0x04;
				}
			}

			sendFrame->Add(~sum);								// 校验和

			break;
		}

		case WRITE_BYTE:
		{
			sendFrame->Add(0x05);								// 帧长
			sendFrame->Add(WRITE_MEMORY);				// 命令字
			sum += WRITE_MEMORY;

			// 将输入的 16 进制地址转换， 因为M51文件中的变量地址通常为 16进制
			m_szDataAddr.MakeUpper();
			ushDataAddr = HexString2ush(m_szDataAddr);

			uitemp.all = ushDataAddr;						// 数据地址
			sendFrame->Add(uitemp.b[0]);				// 低字节
			sum += uitemp.b[0];						
			sendFrame->Add(uitemp.b[1]);				// 高字节
			sum += uitemp.b[1];						
			sendFrame->Add(0x01);								// 字节数
			sum += 0x01;
			sendFrame->Add(m_ucWriteData);			// 要写的字节
			sum += m_ucWriteData;
			sendFrame->Add(~sum);								// 校验和
			break;
		}

		case MOTOR_PWM:
		{
			sendFrame->Add(0x05);								// 帧长
			sendFrame->Add(MOTOR_PWM_CTRL);			// 命令字
			sum += MOTOR_PWM_CTRL;
			uitemp.all = m_uiPWM_Val[0];				// 左侧电机数据
			sendFrame->Add(uitemp.b[0]);				// 低字节
			sum += uitemp.b[0];						
			sendFrame->Add(uitemp.b[1]);				// 高字节
			sum += uitemp.b[1];	
			uitemp.all = m_uiPWM_Val[1];				// 右侧电机数据
			sendFrame->Add(uitemp.b[0]);				// 低字节
			sum += uitemp.b[0];						
			sendFrame->Add(uitemp.b[1]);				// 高字节
			sum += uitemp.b[1];	
			sendFrame->Add(~sum);								// 校验和
			break;
		}

		case RUN_STR:
		{
			sendFrame->Add(0x05);								// 帧长
			if(m_ctrlRunMode.GetCurSel() == 0)
			{
				sendFrame->Add(RUN_STRAIGHT);				// 命令字
				sum += RUN_STRAIGHT;
			}
			else
			{
				sendFrame->Add(RUN_ON_LINE);				// 命令字
				sum += RUN_ON_LINE;
				m_ctrlRunDir.SetCurSel(0);					// 走轨迹只支持前进
			}

			
			
			CString szButton;	

			GetDlgItem(IDC_RUN_STRAIGHT) -> GetWindowText(szButton);
			
			if(szButton == "启动")
			{
				// 启动处理
				GetDlgItem(IDC_RUN_STRAIGHT) -> SetWindowText("停止");

				uitemp.all = m_uiLeftRunNum;				// 取行走脉冲定值
				sendFrame->Add(uitemp.b[0]);
				sum += uitemp.b[0];

				sendFrame->Add(uitemp.b[1]);				// 只取低 2 字节
				sum += uitemp.b[1];

				sendFrame->Add(m_ucBase_PWM);
				sum += m_ucBase_PWM;

				if(m_ctrlRunDir.GetCurSel() == 0)
				{
					sendFrame ->Add(0);								//	设置 PWM 值方向：前进
				}
				else
				{
					sendFrame ->Add(0xFF);						//	设置 PWM 值方向：后退
					sum += 0xFF;
				}				
			}
			else
			{
				// 停止处理
				GetDlgItem(IDC_RUN_STRAIGHT) -> SetWindowText("启动");

				sendFrame ->Add(0);
				sendFrame ->Add(0);
				sendFrame ->Add(BRAKE_PWM);			// 发送刹车PWM值作为基值
				sum +=BRAKE_PWM;
				sendFrame ->Add(0);							// PWM值高字节为0
			}

			sendFrame->Add(~sum);							// 校验和

			break;
		}

		case READ_SAMPLE:
		{
			sendFrame->Add(0x04);								// 帧长
			sendFrame->Add(READ_MEMORY);				// 命令字
			sum += READ_MEMORY;

			// 将输入的 16 进制地址转换， 因为M51文件中的变量地址通常为 16进制
			m_szSampleAddr.MakeUpper();
			ushDataAddr = HexString2ush(m_szSampleAddr);

			uitemp.all = ushDataAddr;						// 数据地址
			sendFrame->Add(uitemp.b[0]);				// 低字节
			sum += uitemp.b[0];						
			sendFrame->Add(uitemp.b[1]);				// 高字节, 只取低 2 字节
			sum += uitemp.b[1];						

			sendFrame->Add(0x05);								// 读 5 字节
			sum += 0x05;

			sendFrame->Add(~sum);								// 校验和

			break;
		}
		default: break;
	}

	m_szCommandBack = "失败";
	this->UpdateData(false);
}

// 电机 PWM 控制处理

void CTalkToCarDlg::OnPWM_Out() 
{
	// TODO: Add your control notification handler code here
	this->UpdateData(true);				// 将对话框中的地址等数据取回

	if(open_CommPort())
	{
		m_uiPWM_Val[0] = m_ucMotorL_PWM;
		m_uiPWM_Val[1] = m_ucMotorR_PWM;

		switch(m_ctrlMotor_L.GetCurSel())
		{
			case FORWARD:
			{
				break;
			}

			case BACKWARD:
			{
				m_uiPWM_Val[0] +=0xFF00;
				break;
			}

			case FLOAT_C:
			{
				m_uiPWM_Val[0] =FLOAT_PWM;
				break;
			}

			case BRAKE_C:
			{
				m_uiPWM_Val[0] =BRAKE_PWM;
				break;
			}
		
			default: break;
		}

		switch(m_ctrlMotor_R.GetCurSel())
		{
			case FORWARD:
			{
				break;
			}

			case BACKWARD:
			{
				m_uiPWM_Val[1] +=0xFF00;
				break;
			}

			case FLOAT_C:
			{
				m_uiPWM_Val[1] =FLOAT_PWM;
				break;
			}

			case BRAKE_C:
			{
				m_uiPWM_Val[1] =BRAKE_PWM;
				break;
			}
		
			default: break;
		}

		makeSendFrame(MOTOR_PWM, &m_baSendFrame);									// 构建发送帧

		m_iRcvStat = NO_RCV;																			// 初始化接收状态
		m_ctrlCommToCar.SetOutput((COleVariant)m_baSendFrame);		// 启动发送
		m_baSendFrame.RemoveAll();
	}
	else
	{
		m_ctrlCommToCar.SetPortOpen(false);
	}	
	
}

// 电机刹车处理

void CTalkToCarDlg::OnBrake() 
{
	// TODO: Add your control notification handler code here
	if(open_CommPort())
	{
		m_uiPWM_Val[0] = BRAKE_PWM;
		m_uiPWM_Val[1] = BRAKE_PWM;

		makeSendFrame(MOTOR_PWM, &m_baSendFrame);									// 构建发送帧

		m_iRcvStat = NO_RCV;																			// 初始化接收状态
		m_ctrlCommToCar.SetOutput((COleVariant)m_baSendFrame);		// 启动发送
		m_baSendFrame.RemoveAll();
	}
	else
	{
		m_ctrlCommToCar.SetPortOpen(false);
	}	
}
	

// 电机惰行处理
void CTalkToCarDlg::OnFloat() 
{
	// TODO: Add your control notification handler code here
	if(open_CommPort())
	{
		m_uiPWM_Val[0] = FLOAT_PWM;
		m_uiPWM_Val[1] = FLOAT_PWM;

		makeSendFrame(MOTOR_PWM, &m_baSendFrame);									// 构建发送帧

		m_iRcvStat = NO_RCV;																			// 初始化接收状态
		m_ctrlCommToCar.SetOutput((COleVariant)m_baSendFrame);		// 启动发送
		m_baSendFrame.RemoveAll();
	}
	else
	{
		m_ctrlCommToCar.SetPortOpen(false);
	}		
}


// 走直线测试

void CTalkToCarDlg::OnRunStraight() 
{
	// TODO: Add your control notification handler code here

	this->UpdateData(true);				// 将对话框中的地址等数据取回

	if(open_CommPort())
	{
		makeSendFrame(RUN_STR, &m_baSendFrame);										// 构建发送帧

		m_iRcvStat = NO_RCV;																			// 初始化接收状态
		m_ctrlCommToCar.SetOutput((COleVariant)m_baSendFrame);		// 启动发送
		m_baSendFrame.RemoveAll();		
	}
	else
	{
		m_ctrlCommToCar.SetPortOpen(false);
	}		
}

// 因为读采样值同样需要将 HEX 码地址转换为 unsigned short 值，
// 所以将前面的处理变成函数

unsigned short CTalkToCarDlg::HexString2ush(CString szHexVal)
{
	BYTE ucVal,ucLen;
	unsigned short ushDataAddr;
	int i;

	ushDataAddr = 0;
	ucLen = szHexVal.GetLength();					// 得到输入的地址长度
	if(ucLen >4 )
	{
		ucLen = 4;															// 只处理 4 位
	}

	for(i =0; i<ucLen; i++)
	{
		ushDataAddr = ushDataAddr *16;

		ucVal = (BYTE) (szHexVal[i]);
		if(ucVal > 0x39)
		{
			ushDataAddr += ucVal - 0x41+10;			// 转换 A - F
		}
		else
		{
			ushDataAddr += ucVal - 0x30;					// 转换 0 - 9
		}
	}

	return (ushDataAddr) ;
}

/* 
	读取采样处理相关数据：4 路采样模拟量的 AD 结果（8位），以及由此而产生的逻辑值
	直接使用读内存功能，根据 M51 得到 AD 结果存放的地址。
	注意：为了可以使用一帧命令读出 AD 结果和逻辑值，在单片机程序设计时将逻辑值和
	AD 的结果放在一个数组中！！！
*/

void CTalkToCarDlg::OnReadSample() 
{
	// TODO: Add your control notification handler code here
	this->UpdateData(true);				// 将对话框中的地址等数据取回

	if(open_CommPort())
	{
		m_iReadMemContent = SAMPLE_DATA;
		makeSendFrame(READ_SAMPLE, &m_baSendFrame);									// 构建发送帧

		m_iRcvStat = NO_RCV;																			// 初始化接收状态
		m_ctrlCommToCar.SetOutput((COleVariant)m_baSendFrame);		// 启动发送
		m_baSendFrame.RemoveAll();
	}
	else
	{
		m_ctrlCommToCar.SetPortOpen(false);
	}		
}

// 因为读内存处理太多，为便于阅读，将各类数据变成函数
// 此为读通用数据的处理

void CTalkToCarDlg::gatGeneralData()
{
	union
	{
		int all;
		BYTE b[4];
	}itemp;

	union
	{
		short all;
		BYTE b[2];
	}shtemp;

	int iType;

	iType = m_ctrlDataType.GetCurSel();							// 获得数据类型
	switch(iType)																		// 根据数据类型处理返回的数据
	{
		case 0:
		{
			// sign char
			m_iReadData = (int)((signed char)m_ucBackBuf[m_ucStartPtr+4]);	
			break;
		}

		case 1:
		{
			// BYTE
			shtemp.all =0;
			shtemp.b[0] = m_ucBackBuf[m_ucStartPtr+4];
			m_iReadData = (int)shtemp.all;
			break;
		}

		case 2:
		{
			// int
			shtemp.b[1] = m_ucBackBuf[m_ucStartPtr+4];
			shtemp.b[0] = m_ucBackBuf[m_ucStartPtr+5];				// C51 的整形顺序相反
			m_iReadData = (int)shtemp.all;
			break;
		}

		case 3:
		{
			// UINT
			itemp.all =0;
			itemp.b[1] = m_ucBackBuf[m_ucStartPtr+4];
			itemp.b[0] = m_ucBackBuf[m_ucStartPtr+5];					// C51 的整形顺序相反
			m_iReadData = itemp.all;
			break;
		}

		case 4:
		{
			// long
			itemp.b[4] = m_ucBackBuf[m_ucStartPtr+4];
			itemp.b[3] = m_ucBackBuf[m_ucStartPtr+5];
			itemp.b[2] = m_ucBackBuf[m_ucStartPtr+6];
			itemp.b[1] = m_ucBackBuf[m_ucStartPtr+7];				 // C51 的长整形顺序相反
			m_iReadData = itemp.all;
			break;
		}

		default: break;
	}
}

// 读取轨迹采样数据处理

void CTalkToCarDlg::getSampleData()
{
	m_ucSampleVal1  = m_ucBackBuf[m_ucStartPtr+4];
	m_ucSampleVal2  = m_ucBackBuf[m_ucStartPtr+5];
	m_ucSampleVal3  = m_ucBackBuf[m_ucStartPtr+6];
	m_ucSampleVal4  = m_ucBackBuf[m_ucStartPtr+7];

	m_ucSensorStat  = m_ucBackBuf[m_ucStartPtr+8];
}
