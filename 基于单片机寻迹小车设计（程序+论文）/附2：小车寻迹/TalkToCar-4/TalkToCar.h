// TalkToCar.h : main header file for the TALKTOCAR application
//

#if !defined(AFX_TALKTOCAR_H__7176709F_B9BD_4C09_A695_F0264B642646__INCLUDED_)
#define AFX_TALKTOCAR_H__7176709F_B9BD_4C09_A695_F0264B642646__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#ifndef __AFXWIN_H__
	#error include 'stdafx.h' before including this file for PCH
#endif

#include "resource.h"		// main symbols

/////////////////////////////////////////////////////////////////////////////
// CTalkToCarApp:
// See TalkToCar.cpp for the implementation of this class
//

class CTalkToCarApp : public CWinApp
{
public:
	CTalkToCarApp();

// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CTalkToCarApp)
	public:
	virtual BOOL InitInstance();
	//}}AFX_VIRTUAL

// Implementation

	//{{AFX_MSG(CTalkToCarApp)
		// NOTE - the ClassWizard will add and remove member functions here.
		//    DO NOT EDIT what you see in these blocks of generated code !
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};


/////////////////////////////////////////////////////////////////////////////

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_TALKTOCAR_H__7176709F_B9BD_4C09_A695_F0264B642646__INCLUDED_)
