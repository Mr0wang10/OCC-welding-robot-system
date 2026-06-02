#pragma once
#include "afxdialogex.h"
#include "resource.h"

class CPageWeld : public CPropertyPage
{
    DECLARE_DYNAMIC(CPageWeld)
public:
    CPageWeld();
    virtual ~CPageWeld();
    enum { IDD = IDD_PAGE_WELD };
    void SetWeldCheckbox(int index, bool checked);  // 供 3D 视图单击焊缝时同步勾选状态
    void ClearWeldList();  // 供删除全部模型时同步清除焊缝列表
protected:
    virtual void DoDataExchange(CDataExchange* pDX);
    virtual BOOL OnInitDialog();
    afx_msg BOOL OnEraseBkgnd(CDC* pDC);
    afx_msg void OnSize(UINT nType, int cx, int cy);
    afx_msg void OnBnClickedIdentifyWeld();
    afx_msg void OnSelChangeTab(NMHDR* pNMHDR, LRESULT* pResult);
    afx_msg void OnWeldListItemChanged(NMHDR* pNMHDR, LRESULT* pResult);
    virtual BOOL OnNotify(WPARAM wParam, LPARAM lParam, LRESULT* pResult);
    void AddSampleData();
    LRESULT HandleHeaderCustomDraw(NMHDR* pNMHDR);
    CButton m_btnIdentify;
    CTabCtrl m_tabWeld;
    CListCtrl m_listWeld;
    BOOL m_bAllChecked;
    DECLARE_MESSAGE_MAP()
};
