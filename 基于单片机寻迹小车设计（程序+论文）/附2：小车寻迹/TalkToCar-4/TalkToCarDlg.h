// TalkToCarDlg.h : header file
//
//{{AFX_INCLUDES()
#include "mscomm.h"
//}}AFX_INCLUDES

#if !defined(AFX_TALKTOCARDLG_H__05B666A9_3B50_4609_B4C9_00E688633FDD__INCLUDED_)
#define AFX_TALKTOCARDLG_H__05B666A9_3B50_4609_B4C9_00E688633FDD__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

/////////////////////////////////////////////////////////////////////////////
// CTalkToCarDlg dialog

class CTalkToCarDlg : public CDialog
{
// Construction
public:
	CTalkToCarDlg(CWnd* pParent = NULL);	// standard constructor

// Dialog Data
	//{{AFX_DATA(CTalkToCarDlg)
	enum { IDD = IDD_TALKTOCAR_DIALOG };
	CComboBox	m_ctrlRunMode;
	CComboBox	m_ctrlDataType;
	CComboBox	m_ctrlRunDir;
	CComboBox	m_ctrlMotor_R;
	CComboBox	m_ctrlMotor_L;
	CComboBox	m_ctrlSelCommPort;
	CMSComm	m_ctrlCommToCar;
	BYTE	m_ucWriteData;
	BYTE	m_ucMotorL_PWM;
	BYTE	m_ucMotorR_PWM;
	BYTE	m_ucBase_PWM;
	UINT	m_uiLeftRunNum;
	int		m_iReadData;
	CString	m_szDataAddr;
	BYTE	m_ucSampleVal1;
	BYTE	m_ucSampleVal2;
	BYTE	m_ucSampleVal3;
	BYTE	m_ucSampleVal4;
	BYTE	m_ucSensorStat;
	CString	m_szSampleAddr;
	CString	m_szCommandBack;
	//}}AFX_DATA

	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CTalkToCarDlg)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:
	void getSampleData(void);
	void gatGeneralData(void);
	int m_iReadMemContent;
	UINT m_uiPWM_Val[2];							// 构成PWM命令帧的电机数据
	int m_iRcvStat;
	int DataFrame_OK(void);
	BYTE m_ucEndPtr;									// 数据帧结束位置指针
	BYTE m_ucStartPtr;								// 数据帧中有效数据起始位置
	BYTE m_ucGetBack;									// 从数据缓冲区中取数指针
	BYTE m_ucSaveBack;								// 将收到的数据存入缓冲区的指针
	BYTE m_ucBackBuf[256];						// 接收缓冲区，注意所设计的字节数以及所用指针的数据类型！！
	// 以上变量构成了文中所述的环形缓冲区接收处理模式！

	void makeSendFrame(int iCommand, CByteArray *sendFrame);
	CByteArray m_baSendFrame;					// 一个特殊的结构，用于串口发送，我也是抄来的 :D
	HICON m_hIcon;

	// Generated message map functions
	//{{AFX_MSG(CTalkToCarDlg)
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	afx_msg void OnSelCommPort();
	afx_msg void OnRead();
	afx_msg void OnWrite();
	afx_msg void OnData_T_R();
	afx_msg void OnPWM_Out();
	afx_msg void OnBrake();
	afx_msg void OnFloat();
	afx_msg void OnRunStraight();
	afx_msg void OnReadSample();
	DECLARE_EVENTSINK_MAP()
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
private:
	unsigned short HexString2ush(CString szHexVal);
	bool open_CommPort(void);
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_TALKTOCARDLG_H__05B666A9_3B50_4609_B4C9_00E688633FDD__INCLUDED_)
