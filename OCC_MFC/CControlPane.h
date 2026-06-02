#pragma once
#include <afxdockablepane.h> // 必须包含停靠面板头文件
#include "CRobotControlDlg.h" // 引用你的滑块对话框类

class CControlPane : public CDockablePane
{
public:
	CControlPane();
	virtual ~CControlPane();

	CRobotControlDlg m_wndDlg; // 嵌入对话框成员变量

protected:
	// 消息处理声明
	afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
	afx_msg void OnSize(UINT nType, int cx, int cy);

	DECLARE_MESSAGE_MAP()
};