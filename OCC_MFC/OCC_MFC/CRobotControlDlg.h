#pragma once
#include "afxdialogex.h"
#include "resource.h"
#include "CPageRobot.h"
#include "CPageWeld.h"

class COCCMFCDoc;
class CRobotControlDlg : public CDialogEx
{
    DECLARE_DYNAMIC(CRobotControlDlg)
public:
    CRobotControlDlg(CWnd* pParent = nullptr);
    virtual ~CRobotControlDlg();
    COCCMFCDoc* m_pDoc;
#ifdef AFX_DESIGN_TIME
    enum { IDD = IDD_ROBOT_CONTROL_PANEL };
#endif
    void UpdateJointDisplay();
    void UpdateTCPDisplay(double x, double y, double z, double w, double p, double r);
    void UpdateImportPosDisplay();
public:
    CPropertySheet m_sheet;
    CPageRobot m_pageRobot;
    CPageWeld m_pageWeld;
protected:
    virtual void DoDataExchange(CDataExchange* pDX);
    virtual BOOL OnInitDialog();
    afx_msg void OnSize(UINT nType, int cx, int cy);
    DECLARE_MESSAGE_MAP()
};
