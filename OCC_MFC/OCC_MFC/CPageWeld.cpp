#include "pch.h"
#include "framework.h"
#include "OCC_MFC.h"
#include "CPageWeld.h"
#include "afxdialogex.h"
#include "MainFrm.h"
#include "OCC_MFCDoc.h"
#include "OCC_MFCView.h"

IMPLEMENT_DYNAMIC(CPageWeld, CPropertyPage)

CPageWeld::CPageWeld() : CPropertyPage(IDD_PAGE_WELD), m_bAllChecked(FALSE)
{ m_psp.pszTitle = _T("焊接参数"); m_psp.dwFlags |= PSP_USETITLE; }
CPageWeld::~CPageWeld() {}

void CPageWeld::DoDataExchange(CDataExchange* pDX)
{
    CPropertyPage::DoDataExchange(pDX);
    DDX_Control(pDX, IDC_BUTTON_IDENTIFY_WELD, m_btnIdentify);
    DDX_Control(pDX, IDC_TAB_WELD, m_tabWeld);
    DDX_Control(pDX, IDC_LIST_WELD, m_listWeld);
}

BEGIN_MESSAGE_MAP(CPageWeld, CPropertyPage)
    ON_WM_ERASEBKGND()
    ON_WM_SIZE()
    ON_BN_CLICKED(IDC_BUTTON_IDENTIFY_WELD, &CPageWeld::OnBnClickedIdentifyWeld)
    ON_NOTIFY(TCN_SELCHANGE, IDC_TAB_WELD, &CPageWeld::OnSelChangeTab)
    ON_NOTIFY(LVN_ITEMCHANGED, IDC_LIST_WELD, &CPageWeld::OnWeldListItemChanged)
END_MESSAGE_MAP()

BOOL CPageWeld::OnEraseBkgnd(CDC* pDC)
{
    CRect rc;
    GetClientRect(&rc);
    pDC->FillSolidRect(&rc, GetSysColor(COLOR_BTNFACE));
    return TRUE;
}

// ========== Add 20 sample data rows ==========
void CPageWeld::AddSampleData()
{
    CString starts[] = {
        _T("222.6457, -527.3547, 822.3368"),
        _T("200.1234, -30.5678, 10.1234"),
        _T("180.4567, -45.6789, 12.3456"),
    };
    CString ends[] = {
        _T("222.8961, 177.6429, 7.3115"),
        _T("210.9876, 150.4321, 6.7890"),
        _T("195.2345, 160.1234, 8.9012"),
    };

    for (int i = 0; i < 20; i++)
    {
        CString strId, strSeq;
        strId.Format(_T("%d"), i + 1);
        strSeq.Format(_T("%03d"), i + 1);

        int idx = m_listWeld.InsertItem(i, _T(""));
        m_listWeld.SetItemText(idx, 1, strId);
        m_listWeld.SetItemText(idx, 2, strSeq);
        m_listWeld.SetItemText(idx, 3, starts[i % 3]);
        m_listWeld.SetItemText(idx, 4, ends[i % 3]);
        m_listWeld.SetCheck(idx, FALSE);
    }
}

// ========== Header custom draw (select-all checkbox) ==========
LRESULT CPageWeld::HandleHeaderCustomDraw(NMHDR* pNMHDR)
{
    LPNMCUSTOMDRAW pCD = (LPNMCUSTOMDRAW)pNMHDR;
    if (pCD->dwDrawStage == CDDS_PREPAINT)
        return CDRF_NOTIFYITEMDRAW;
    if (pCD->dwDrawStage != CDDS_ITEMPREPAINT)
        return CDRF_DODEFAULT;
    if (pCD->dwItemSpec != 0)
        return CDRF_DODEFAULT;

    CDC* pDC = CDC::FromHandle(pCD->hdc);
    CRect rc = pCD->rc;

    pDC->FillSolidRect(&rc, GetSysColor(COLOR_BTNFACE));

    CRect cbRect = rc;
    cbRect.left += 4;
    cbRect.top += (rc.Height() - 13) / 2;
    cbRect.right = cbRect.left + 13;
    cbRect.bottom = cbRect.top + 13;
    pDC->DrawFrameControl(&cbRect, DFC_BUTTON,
        m_bAllChecked ? (DFCS_BUTTONCHECK | DFCS_CHECKED) : DFCS_BUTTONCHECK);

    CRect txtRect = rc;
    txtRect.left = cbRect.right + 4;
    pDC->SetBkMode(TRANSPARENT);
    pDC->SetTextColor(GetSysColor(COLOR_WINDOWTEXT));
    CFont* pOldFont = (CFont*)pDC->SelectObject(GetFont());
    pDC->DrawText(_T("全选"), &txtRect, DT_LEFT | DT_VCENTER | DT_SINGLELINE);
    pDC->SelectObject(pOldFont);

    return CDRF_SKIPDEFAULT;
}

// ========== OnNotify: header custom draw + select-all click ==========
BOOL CPageWeld::OnNotify(WPARAM wParam, LPARAM lParam, LRESULT* pResult)
{
    NMHDR* pNMHDR = (NMHDR*)lParam;

    if (pNMHDR->code == NM_CUSTOMDRAW && m_listWeld.GetSafeHwnd())
    {
        CHeaderCtrl* pHeader = m_listWeld.GetHeaderCtrl();
        if (pHeader && pNMHDR->hwndFrom == pHeader->GetSafeHwnd())
        {
            *pResult = HandleHeaderCustomDraw(pNMHDR);
            return TRUE;
        }
    }

    if ((pNMHDR->code == HDN_ITEMCLICKW || pNMHDR->code == HDN_ITEMCLICKA) && m_listWeld.GetSafeHwnd())
    {
        CHeaderCtrl* pHeader = m_listWeld.GetHeaderCtrl();
        if (pHeader && pNMHDR->hwndFrom == pHeader->GetSafeHwnd())
        {
            NMHEADER* pHD = (NMHEADER*)pNMHDR;
            if (pHD->iItem == 0)
            {
                m_bAllChecked = !m_bAllChecked;
                for (int i = 0; i < m_listWeld.GetItemCount(); i++)
                    m_listWeld.SetCheck(i, m_bAllChecked);
                pHeader->Invalidate();
                *pResult = 0;
                return TRUE;
            }
        }
    }

    return CPropertyPage::OnNotify(wParam, lParam, pResult);
}

// ========== OnSize: only adjust list column widths ==========
void CPageWeld::OnSize(UINT nType, int cx, int cy)
{
    CPropertyPage::OnSize(nType, cx, cy);
    if (!m_listWeld.GetSafeHwnd()) return;

    if (cx > 40)
    {
        int col0 = 50, col1 = 60, col2 = 60;
        int rem = cx - col0 - col1 - col2;
        int colW = max(240, rem / 2);
        m_listWeld.SetColumnWidth(0, col0);
        m_listWeld.SetColumnWidth(1, col1);
        m_listWeld.SetColumnWidth(2, col2);
        m_listWeld.SetColumnWidth(3, colW);
        m_listWeld.SetColumnWidth(4, colW);
    }
}

// ========== Identify weld button: compute + populate + display ==========
void CPageWeld::OnBnClickedIdentifyWeld()
{
    CMainFrame* pMain = (CMainFrame*)AfxGetMainWnd();
    if (!pMain) return;
    COCCMFCDoc* pDoc = (COCCMFCDoc*)pMain->GetActiveDocument();
    if (!pDoc) return;

    std::vector<WeldSeam> seams = pDoc->ComputeWeldSeams();

    if (seams.empty())
    {
        AfxMessageBox(_T("未检测到焊缝，请检查导入的模型是否正确"));
        return;
    }

    m_listWeld.DeleteAllItems();

    for (size_t i = 0; i < seams.size(); i++)
    {
        CString strId, strSeq, strStart, strEnd;
        strId.Format(_T("%d"), (int)(i + 1));
        strSeq.Format(_T("%03d"), (int)(i + 1));
        strStart.Format(_T("%.4f, %.4f, %.4f"),
            seams[i].StartPoint.X, seams[i].StartPoint.Y, seams[i].StartPoint.Z);
        strEnd.Format(_T("%.4f, %.4f, %.4f"),
            seams[i].EndPoint.X, seams[i].EndPoint.Y, seams[i].EndPoint.Z);

        int idx = m_listWeld.InsertItem((int)i, _T(""));
        m_listWeld.SetItemText(idx, 1, strId);
        m_listWeld.SetItemText(idx, 2, strSeq);
        m_listWeld.SetItemText(idx, 3, strStart);
        m_listWeld.SetItemText(idx, 4, strEnd);
        m_listWeld.SetCheck(idx, FALSE);
    }

    COCCMFCView* pView = (COCCMFCView*)pMain->GetActiveView();
    if (pView)
    {
        TopoDS_Shape weldShape = pDoc->GetWeldSeamShape();
        pView->DisplayWeldCurves(weldShape, seams);
    }

    AfxMessageBox(_T("识别完成"));
}

// ==============================
// OnWeldListItemChanged
// 功能：列表项勾选状态变化时，通知 3D 视图改变焊缝颜色
// ==============================
// ==============================
// SetWeldCheckbox
// 功能：由 3D 视图单击焊缝时调用，同步更新复选框状态
// ==============================
// ==============================
// ClearWeldList
// 功能：清除焊缝列表所有数据
// ==============================
void CPageWeld::ClearWeldList()
{
    if (!m_listWeld.GetSafeHwnd()) return;
    m_listWeld.DeleteAllItems();
}

void CPageWeld::SetWeldCheckbox(int index, bool checked)
{
    if (!m_listWeld.GetSafeHwnd()) return;
    if (index < 0 || index >= m_listWeld.GetItemCount()) return;
    m_listWeld.SetCheck(index, checked);
}

void CPageWeld::OnWeldListItemChanged(NMHDR* pNMHDR, LRESULT* pResult)
{
    LPNMLISTVIEW pLV = (LPNMLISTVIEW)pNMHDR;
    *pResult = 0;

    // 只处理复选框状态变化
    if ((pLV->uChanged & LVIF_STATE) == 0)
        return;
    if (((pLV->uOldState & LVIS_STATEIMAGEMASK) == (pLV->uNewState & LVIS_STATEIMAGEMASK)))
        return;

    int seamIndex = pLV->iItem;
    if (seamIndex < 0)
        return;

    bool selected = (pLV->uNewState & LVIS_STATEIMAGEMASK) == INDEXTOSTATEIMAGEMASK(2);

    CMainFrame* pMain = (CMainFrame*)AfxGetMainWnd();
    if (!pMain) return;

    COCCMFCView* pView = (COCCMFCView*)pMain->GetActiveView();
    if (!pView) return;

    pView->SetWeldSeamColor(seamIndex, selected);
}

// ========== Tab switch placeholder ==========
void CPageWeld::OnSelChangeTab(NMHDR* pNMHDR, LRESULT* pResult)
{
    int sel = m_tabWeld.GetCurSel();
    TRACE(_T("Tab switched to %d\n"), sel);
    *pResult = 0;
}

BOOL CPageWeld::OnInitDialog()
{
    CPropertyPage::OnInitDialog();

    m_tabWeld.InsertItem(0, _T("机器人末端坐标转换"));
    m_tabWeld.InsertItem(1, _T("用户坐标转换"));
    m_tabWeld.InsertItem(2, _T("图像焊缝信息"));
    m_tabWeld.SetCurSel(0);

    m_listWeld.SetExtendedStyle(m_listWeld.GetExtendedStyle()
        | LVS_EX_CHECKBOXES | LVS_EX_GRIDLINES | LVS_EX_FULLROWSELECT | LVS_EX_DOUBLEBUFFER);

    m_listWeld.InsertColumn(0, _T("全选"), LVCFMT_LEFT, 50);
    m_listWeld.InsertColumn(1, _T("焊缝号"), LVCFMT_LEFT, 60);
    m_listWeld.InsertColumn(2, _T("序列号"), LVCFMT_LEFT, 60);
    m_listWeld.InsertColumn(3, _T("起点(mm)"), LVCFMT_LEFT, 240);
    m_listWeld.InsertColumn(4, _T("终点(mm)"), LVCFMT_LEFT, 240);

    AddSampleData();
    return TRUE;
}
