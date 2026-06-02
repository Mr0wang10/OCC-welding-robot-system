#include "pch.h"
#include "framework.h"
#include "CControlPane.h"
#include "resource.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

CControlPane::CControlPane()
{
}

CControlPane::~CControlPane()
{
}

BEGIN_MESSAGE_MAP(CControlPane, CDockablePane)
	ON_WM_CREATE()
	ON_WM_SIZE()
END_MESSAGE_MAP()

// 当面板被创建时执行
int CControlPane::OnCreate(LPCREATESTRUCT lpCreateStruct)
{
	if (CDockablePane::OnCreate(lpCreateStruct) == -1)
		return -1;

	// 1. 创建对话框资源
	// 父窗口设为 this (即当前的 CControlPane)
	if (!m_wndDlg.Create(IDD_ROBOT_CONTROL_PANEL, this))
	{
		TRACE0("未能创建机器人控制面板对话框\n");
		return -1;
	}

	// 2. 显示内部对话框
	m_wndDlg.ShowWindow(SW_SHOW);

	return 0;
}

// 当面板大小改变时执行（保证滑块界面铺满面板）
void CControlPane::OnSize(UINT nType, int cx, int cy)
{
	CDockablePane::OnSize(nType, cx, cy);

	// 3. 让对话框的尺寸始终等于停靠面板的客户区尺寸
	if (m_wndDlg.GetSafeHwnd())
	{
		m_wndDlg.SetWindowPos(NULL, 0, 0, cx, cy, SWP_NOZORDER | SWP_NOACTIVATE);
	}
}