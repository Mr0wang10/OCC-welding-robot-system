#pragma once
#include "afxdialogex.h"
#include "resource.h"

class CPageRobot : public CPropertyPage
{
    DECLARE_DYNAMIC(CPageRobot)

public:
    CPageRobot();
    virtual ~CPageRobot();
    enum { IDD = IDD_PAGE_ROBOT };

    CSliderCtrl m_sliderJ1, m_sliderJ2, m_sliderJ3, m_sliderJ4, m_sliderJ5, m_sliderJ6;
    CEdit m_editJ1, m_editJ2, m_editJ3, m_editJ4, m_editJ5, m_editJ6;
    CEdit m_editImportX, m_editImportY, m_editImportZ;

    void UpdateJointDisplay();
    void UpdateTCPDisplay(double x, double y, double z, double w, double p, double r);
    void UpdateImportPosDisplay();
    double GetDlgItemDouble(int nID);
    CString FormattedStr(double val);

protected:
    virtual void DoDataExchange(CDataExchange* pDX);
    virtual BOOL OnInitDialog();
    virtual BOOL OnSetActive();
    afx_msg void OnHScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar);
    afx_msg void OnEnChangeEditJoints();
    afx_msg void OnEnChangeEditTcp();
    afx_msg void OnEnChangeImportPos();
    afx_msg void OnBnClickedImportModel();
    afx_msg void OnBnClickedDeleteSelected();

    afx_msg BOOL OnEraseBkgnd(CDC* pDC);
    bool m_bIsUpdating;
    DECLARE_MESSAGE_MAP()
};

