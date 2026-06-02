#include "pch.h"
#include "framework.h"
#include "OCC_MFC.h"
#include "CPageRobot.h"
#include "OCC_MFCDoc.h"
#include "MainFrm.h"
#include "OCC_MFCView.h"
#include "afxdialogex.h"

IMPLEMENT_DYNAMIC(CPageRobot, CPropertyPage)

CPageRobot::CPageRobot() : CPropertyPage(IDD_PAGE_ROBOT), m_bIsUpdating(false) { m_psp.pszTitle = _T("\u673a\u5668\u4eba\u63a7\u5236"); m_psp.dwFlags |= PSP_USETITLE; }
CPageRobot::~CPageRobot() {}

void CPageRobot::DoDataExchange(CDataExchange* pDX)
{
    CPropertyPage::DoDataExchange(pDX);
    DDX_Control(pDX, IDC_SLIDER_J1, m_sliderJ1); DDX_Control(pDX, IDC_SLIDER_J2, m_sliderJ2);
    DDX_Control(pDX, IDC_SLIDER_J3, m_sliderJ3); DDX_Control(pDX, IDC_SLIDER_J4, m_sliderJ4);
    DDX_Control(pDX, IDC_SLIDER_J5, m_sliderJ5); DDX_Control(pDX, IDC_SLIDER_J6, m_sliderJ6);
    DDX_Control(pDX, IDC_EDIT_J1, m_editJ1); DDX_Control(pDX, IDC_EDIT_J2, m_editJ2);
    DDX_Control(pDX, IDC_EDIT_J3, m_editJ3); DDX_Control(pDX, IDC_EDIT_J4, m_editJ4);
    DDX_Control(pDX, IDC_EDIT_J5, m_editJ5); DDX_Control(pDX, IDC_EDIT_J6, m_editJ6);
    DDX_Control(pDX, IDC_EDIT_IMPORT_X, m_editImportX);
    DDX_Control(pDX, IDC_EDIT_IMPORT_Y, m_editImportY);
    DDX_Control(pDX, IDC_EDIT_IMPORT_Z, m_editImportZ);
}

BEGIN_MESSAGE_MAP(CPageRobot, CPropertyPage)
    ON_WM_HSCROLL()
    ON_EN_CHANGE(IDC_EDIT_J1, &CPageRobot::OnEnChangeEditJoints)
    ON_EN_CHANGE(IDC_EDIT_J2, &CPageRobot::OnEnChangeEditJoints)
    ON_EN_CHANGE(IDC_EDIT_J3, &CPageRobot::OnEnChangeEditJoints)
    ON_EN_CHANGE(IDC_EDIT_J4, &CPageRobot::OnEnChangeEditJoints)
    ON_EN_CHANGE(IDC_EDIT_J5, &CPageRobot::OnEnChangeEditJoints)
    ON_EN_CHANGE(IDC_EDIT_J6, &CPageRobot::OnEnChangeEditJoints)
    ON_EN_CHANGE(IDC_EDIT_PX, &CPageRobot::OnEnChangeEditTcp)
    ON_EN_CHANGE(IDC_EDIT_PY, &CPageRobot::OnEnChangeEditTcp)
    ON_EN_CHANGE(IDC_EDIT_PZ, &CPageRobot::OnEnChangeEditTcp)
    ON_EN_CHANGE(IDC_EDIT_PW, &CPageRobot::OnEnChangeEditTcp)
    ON_EN_CHANGE(IDC_EDIT_PP, &CPageRobot::OnEnChangeEditTcp)
    ON_EN_CHANGE(IDC_EDIT_PR, &CPageRobot::OnEnChangeEditTcp)
    ON_BN_CLICKED(IDC_BUTTON_IMPORT_MODEL, &CPageRobot::OnBnClickedImportModel)
    ON_BN_CLICKED(IDC_BUTTON_DELETE_SELECTED, &CPageRobot::OnBnClickedDeleteSelected)
    ON_EN_CHANGE(IDC_EDIT_IMPORT_X, &CPageRobot::OnEnChangeImportPos)
    ON_EN_CHANGE(IDC_EDIT_IMPORT_Y, &CPageRobot::OnEnChangeImportPos)
    ON_EN_CHANGE(IDC_EDIT_IMPORT_Z, &CPageRobot::OnEnChangeImportPos)
    ON_WM_ERASEBKGND()
END_MESSAGE_MAP()

BOOL CPageRobot::OnInitDialog()
{
    CPropertyPage::OnInitDialog();
    // 初始化滑块范围为 -180° ~ 180°
    m_sliderJ1.SetRange(-180, 180); m_sliderJ2.SetRange(-180, 180);
    m_sliderJ3.SetRange(-180, 180); m_sliderJ4.SetRange(-180, 180);
    m_sliderJ5.SetRange(-180, 180); m_sliderJ6.SetRange(-180, 180);
    m_sliderJ1.SetPos(0); m_sliderJ2.SetPos(0); m_sliderJ3.SetPos(0);
    m_sliderJ4.SetPos(0); m_sliderJ5.SetPos(0); m_sliderJ6.SetPos(0);
    SetDlgItemInt(IDC_EDIT_J1, 0); SetDlgItemInt(IDC_EDIT_J2, 0);
    SetDlgItemInt(IDC_EDIT_J3, 0); SetDlgItemInt(IDC_EDIT_J4, 0);
    SetDlgItemInt(IDC_EDIT_J5, 0); SetDlgItemInt(IDC_EDIT_J6, 0);
    SetDlgItemText(IDC_EDIT_IMPORT_X, _T("0"));
    SetDlgItemText(IDC_EDIT_IMPORT_Y, _T("0"));
    SetDlgItemText(IDC_EDIT_IMPORT_Z, _T("0"));
    UpdateJointDisplay();
    return TRUE;
}

// ========== 每次页面切换到前台时重新初始化滑块 ==========
BOOL CPageRobot::OnSetActive()
{
    // 确保滑块范围始终有效（属性页切换时可能丢失范围设定）
    m_sliderJ1.SetRange(-180, 180); m_sliderJ2.SetRange(-180, 180);
    m_sliderJ3.SetRange(-180, 180); m_sliderJ4.SetRange(-180, 180);
    m_sliderJ5.SetRange(-180, 180); m_sliderJ6.SetRange(-180, 180);
    // 从文档同步关节显示
    UpdateJointDisplay();
    return CPropertyPage::OnSetActive();
}

double CPageRobot::GetDlgItemDouble(int nID)
{ CString str; GetDlgItemText(nID, str); return _ttof(str); }

CString CPageRobot::FormattedStr(double val)
{ CString s; s.Format(_T("%.3f"), val); return s; }

// ========== 滑块拖动处理 ==========
void CPageRobot::OnHScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar)
{
    if (m_bIsUpdating) return;
    CMainFrame* pMain = (CMainFrame*)AfxGetMainWnd(); if (!pMain) return;
    COCCMFCDoc* pDoc = (COCCMFCDoc*)pMain->GetActiveDocument(); if (!pDoc) return;
    // 通过控件ID识别是哪个滑块被拖动（避免指针比较的不可靠性）
    int nID = pScrollBar->GetDlgCtrlID();
    int idx = -1;
    int val = ((CSliderCtrl*)pScrollBar)->GetPos();
    if (nID == IDC_SLIDER_J1) { idx = 0; SetDlgItemInt(IDC_EDIT_J1, val); }
    else if (nID == IDC_SLIDER_J2) { idx = 1; SetDlgItemInt(IDC_EDIT_J2, val); }
    else if (nID == IDC_SLIDER_J3) { idx = 2; SetDlgItemInt(IDC_EDIT_J3, val); }
    else if (nID == IDC_SLIDER_J4) { idx = 3; SetDlgItemInt(IDC_EDIT_J4, val); }
    else if (nID == IDC_SLIDER_J5) { idx = 4; SetDlgItemInt(IDC_EDIT_J5, val); }
    else if (nID == IDC_SLIDER_J6) { idx = 5; SetDlgItemInt(IDC_EDIT_J6, val); }
    if (idx >= 0)
    {
        // 将角度值（度）直接存储到文档（UpdateRobotPose 内部会转换为弧度）
        pDoc->m_JointAngles[idx] = (double)val;
        pDoc->UpdateRobotPose();
    }
}

// ========== 关节编辑框输入 ==========
void CPageRobot::OnEnChangeEditJoints()
{
    if (m_bIsUpdating) return;
    CMainFrame* pMain = (CMainFrame*)AfxGetMainWnd(); if (!pMain) return;
    COCCMFCDoc* pDoc = (COCCMFCDoc*)pMain->GetActiveDocument(); if (!pDoc) return;
    m_bIsUpdating = true;
    double angles[6] = { GetDlgItemDouble(IDC_EDIT_J1), GetDlgItemDouble(IDC_EDIT_J2),
        GetDlgItemDouble(IDC_EDIT_J3), GetDlgItemDouble(IDC_EDIT_J4),
        GetDlgItemDouble(IDC_EDIT_J5), GetDlgItemDouble(IDC_EDIT_J6) };
    for (int i = 0; i < 6; i++) pDoc->m_JointAngles[i] = angles[i];
    m_sliderJ1.SetPos((int)angles[0]); m_sliderJ2.SetPos((int)angles[1]);
    m_sliderJ3.SetPos((int)angles[2]); m_sliderJ4.SetPos((int)angles[3]);
    m_sliderJ5.SetPos((int)angles[4]); m_sliderJ6.SetPos((int)angles[5]);
    pDoc->UpdateRobotPose();
    m_bIsUpdating = false;
}

// ========== TCP 编辑框输入 ==========
void CPageRobot::OnEnChangeEditTcp()
{
    if (m_bIsUpdating) return;
    CMainFrame* pMain = (CMainFrame*)AfxGetMainWnd(); if (!pMain) return;
    COCCMFCDoc* pDoc = (COCCMFCDoc*)pMain->GetActiveDocument(); if (!pDoc) return;
    m_bIsUpdating = true;
    double px = GetDlgItemDouble(IDC_EDIT_PX); double py = GetDlgItemDouble(IDC_EDIT_PY);
    double pz = GetDlgItemDouble(IDC_EDIT_PZ); double pw = GetDlgItemDouble(IDC_EDIT_PW);
    double pp = GetDlgItemDouble(IDC_EDIT_PP); double pr = GetDlgItemDouble(IDC_EDIT_PR);
    pDoc->OnHandleTCPInput(px, py, pz, pw, pp, pr);
    m_bIsUpdating = false;
}

// ========== 导入三维模型按钮 ==========
void CPageRobot::OnBnClickedImportModel()
{
    CFileDialog dlg(TRUE, NULL, NULL, OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST,
        _T("3D Models (*.step;*.stp;*.iges;*.igs;*.stl)|*.step;*.stp;*.iges;*.igs;*.stl||"), this);
    if (dlg.DoModal() != IDOK) return;
    CMainFrame* pMain = (CMainFrame*)AfxGetMainWnd(); if (!pMain) return;
    COCCMFCDoc* pDoc = (COCCMFCDoc*)pMain->GetActiveDocument(); if (!pDoc) return;
    pDoc->LoadModelFromFile(dlg.GetPathName());
}

// ========== 删除选中模型按钮 ==========
void CPageRobot::OnBnClickedDeleteSelected()
{
    CMainFrame* pMain = (CMainFrame*)AfxGetMainWnd(); if (!pMain) return;
    COCCMFCView* pView = (COCCMFCView*)pMain->GetActiveView(); if (!pView) return;
    pView->DeleteSelectedModel();
}

// ========== 导入模型位置编辑框 ==========
void CPageRobot::OnEnChangeImportPos()
{
    if (m_bIsUpdating) return;
    CMainFrame* pMain = (CMainFrame*)AfxGetMainWnd(); if (!pMain) return;
    COCCMFCDoc* pDoc = (COCCMFCDoc*)pMain->GetActiveDocument(); if (!pDoc) return;
    CWnd* pFocus = GetFocus(); if (!pFocus) return;
    int nID = pFocus->GetDlgCtrlID();
    if (nID != IDC_EDIT_IMPORT_X && nID != IDC_EDIT_IMPORT_Y && nID != IDC_EDIT_IMPORT_Z) return;
    pDoc->SetImportedModelPosition(GetDlgItemDouble(IDC_EDIT_IMPORT_X),
        GetDlgItemDouble(IDC_EDIT_IMPORT_Y), GetDlgItemDouble(IDC_EDIT_IMPORT_Z));
}

// ========== 绘制背景色与父对话框一致 ==========
BOOL CPageRobot::OnEraseBkgnd(CDC* pDC)
{
    CRect rc;
    GetClientRect(&rc);
    pDC->FillSolidRect(&rc, GetSysColor(COLOR_BTNFACE));
    return TRUE;
}

void CPageRobot::UpdateJointDisplay()
{
    CMainFrame* pMain = (CMainFrame*)AfxGetMainWnd(); if (!pMain) return;
    COCCMFCDoc* pDoc = (COCCMFCDoc*)pMain->GetActiveDocument(); if (!pDoc) return;
    m_bIsUpdating = true;
    int deg[6]; for (int i = 0; i < 6; i++) deg[i] = (int)pDoc->m_JointAngles[i];
    m_sliderJ1.SetPos(deg[0]); m_sliderJ2.SetPos(deg[1]);
    m_sliderJ3.SetPos(deg[2]); m_sliderJ4.SetPos(deg[3]);
    m_sliderJ5.SetPos(deg[4]); m_sliderJ6.SetPos(deg[5]);
    SetDlgItemInt(IDC_EDIT_J1, deg[0]); SetDlgItemInt(IDC_EDIT_J2, deg[1]);
    SetDlgItemInt(IDC_EDIT_J3, deg[2]); SetDlgItemInt(IDC_EDIT_J4, deg[3]);
    SetDlgItemInt(IDC_EDIT_J5, deg[4]); SetDlgItemInt(IDC_EDIT_J6, deg[5]);
    m_bIsUpdating = false;
}

void CPageRobot::UpdateTCPDisplay(double x, double y, double z, double w, double p, double r)
{
    m_bIsUpdating = true;
    SetDlgItemText(IDC_EDIT_PX, FormattedStr(x));
    SetDlgItemText(IDC_EDIT_PY, FormattedStr(y));
    SetDlgItemText(IDC_EDIT_PZ, FormattedStr(z));
    SetDlgItemText(IDC_EDIT_PW, FormattedStr(w));
    SetDlgItemText(IDC_EDIT_PP, FormattedStr(p));
    SetDlgItemText(IDC_EDIT_PR, FormattedStr(r));
    m_bIsUpdating = false;
}

void CPageRobot::UpdateImportPosDisplay()
{
    CMainFrame* pMain = (CMainFrame*)AfxGetMainWnd(); if (!pMain) return;
    COCCMFCDoc* pDoc = (COCCMFCDoc*)pMain->GetActiveDocument(); if (!pDoc) return;
    m_bIsUpdating = true;
    SetDlgItemText(IDC_EDIT_IMPORT_X, FormattedStr(pDoc->m_ImportPosition[0]));
    SetDlgItemText(IDC_EDIT_IMPORT_Y, FormattedStr(pDoc->m_ImportPosition[1]));
    SetDlgItemText(IDC_EDIT_IMPORT_Z, FormattedStr(pDoc->m_ImportPosition[2]));
    m_bIsUpdating = false;
}




