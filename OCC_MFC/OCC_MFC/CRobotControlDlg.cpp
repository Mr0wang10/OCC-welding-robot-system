#include "pch.h"
#include "framework.h"
#include "OCC_MFC.h"
#include "CRobotControlDlg.h"
#include "afxdialogex.h"
#include "OCC_MFCDoc.h"
#include "MainFrm.h"
#include "OCC_MFCView.h"

IMPLEMENT_DYNAMIC(CRobotControlDlg, CDialogEx)

CRobotControlDlg::CRobotControlDlg(CWnd* pParent)
    : CDialogEx(IDD_ROBOT_CONTROL_PANEL, pParent), m_pDoc(nullptr)
{}

CRobotControlDlg::~CRobotControlDlg() {}

void CRobotControlDlg::DoDataExchange(CDataExchange* pDX)
{
    CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CRobotControlDlg, CDialogEx)
    ON_WM_SIZE()
END_MESSAGE_MAP()

BOOL CRobotControlDlg::OnInitDialog()
{
    CDialogEx::OnInitDialog();
    // 创建属性表，包含机器人控制和参数化焊接两个属性页
    m_sheet.AddPage(&m_pageRobot);
    m_sheet.AddPage(&m_pageWeld);
    m_sheet.Create(this, WS_CHILD | WS_VISIBLE | WS_TABSTOP, 0);
    // 移除属性表边框，融入父对话框背景
    m_sheet.ModifyStyleEx(WS_EX_CLIENTEDGE | WS_EX_STATICEDGE, 0, SWP_FRAMECHANGED);
    m_sheet.ModifyStyle(WS_BORDER | WS_THICKFRAME | WS_DLGFRAME, 0, SWP_FRAMECHANGED);
    // 让属性表填满整个对话框
    CRect rc;
    GetClientRect(&rc);
    m_sheet.MoveWindow(&rc);
    // 默认显示机器人控制页面
    m_sheet.SetActivePage(0);
    return TRUE;
}

// ========== 属性表随对话框大小变化自动调整 ==========
void CRobotControlDlg::OnSize(UINT nType, int cx, int cy)
{
    CDialogEx::OnSize(nType, cx, cy);
    // 确保属性表始终填满对话框客户区
    if (m_sheet.GetSafeHwnd())
    {
        m_sheet.SetWindowPos(NULL, 0, 0, cx, cy, SWP_NOZORDER | SWP_NOACTIVATE);
    }
}

void CRobotControlDlg::UpdateJointDisplay()
{
    m_pageRobot.UpdateJointDisplay();
}

void CRobotControlDlg::UpdateTCPDisplay(double x, double y, double z, double w, double p, double r)
{
    m_pageRobot.UpdateTCPDisplay(x, y, z, w, p, r);
}

void CRobotControlDlg::UpdateImportPosDisplay()
{
    m_pageRobot.UpdateImportPosDisplay();
}


