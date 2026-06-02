#include "pch.h"
#include "framework.h"
#include "resource.h"

#ifdef _DEBUG
#undef new
#endif

#include <AIS_InteractiveContext.hxx>
#include <AIS_Shape.hxx>
#include <V3d_View.hxx>
#include <V3d_Viewer.hxx>
#include <OpenGl_GraphicDriver.hxx>
#include <WNT_Window.hxx>
#include <Aspect_Window.hxx>
#include <Aspect_DisplayConnection.hxx>
#include <Aspect_TypeOfTriedronPosition.hxx>
#include "OCC_MFCDoc.h"
#include "OCC_MFCView.h"
#include "MainFrm.h"
#include "CControlPane.h"
#include "CRobotControlDlg.h"
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRep_Builder.hxx>
#include <TopoDS_Compound.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <AIS_TextLabel.hxx>
#include <TCollection_AsciiString.hxx>

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

IMPLEMENT_DYNCREATE(COCCMFCView, CView)

BEGIN_MESSAGE_MAP(COCCMFCView, CView)
    ON_WM_PAINT()
    ON_WM_SIZE()
    ON_WM_ERASEBKGND()
    ON_WM_LBUTTONDOWN()
    ON_WM_RBUTTONDOWN()
    ON_WM_LBUTTONUP()
    ON_WM_MOUSEMOVE()
    ON_WM_MOUSEWHEEL()
    ON_COMMAND(ID_FANUC_M10ID12, &COCCMFCView::OnFanucM10iD12)
    ON_COMMAND(ID_BUTTON_CLEAR_ALL, &COCCMFCView::OnButtonClearAll)
END_MESSAGE_MAP()

COCCMFCView::COCCMFCView() noexcept {
    m_bIsRotating = FALSE;
    myGraphicDriver.Nullify(); myViewer.Nullify(); myView.Nullify(); myContext.Nullify(); myWindow.Nullify();
}

COCCMFCView::~COCCMFCView() { if (!myView.IsNull()) myView->Remove(); }

void COCCMFCView::OnButtonClearAll()
{
    if (myContext.IsNull()) return;
    myContext->EraseAll(Standard_True);
    myContext->RemoveAll(Standard_True);
    if (!myView.IsNull()) {
        myView->Redraw();
    }
}

void COCCMFCView::OnInitialUpdate()
{
    CView::OnInitialUpdate();
    Handle(Aspect_DisplayConnection) aDisp = ::new Aspect_DisplayConnection();
    myGraphicDriver = ::new OpenGl_GraphicDriver(aDisp);
    myViewer = ::new V3d_Viewer(myGraphicDriver);
    myView = myViewer->CreateView();
    myContext = ::new AIS_InteractiveContext(myViewer);

    myWindow = ::new WNT_Window(GetSafeHwnd());
    myView->SetWindow(myWindow);
    if (!myWindow->IsMapped()) myWindow->Map();

    myView->SetBackgroundColor(Quantity_NOC_BLACK);
    myViewer->SetDefaultLights();
    myViewer->SetLightOn();

    myView->SetBgGradientColors(Quantity_NOC_GRAY10, Quantity_NOC_GRAY50, Aspect_GFM_VER);
    myContext->SetAutomaticHilight(Standard_False);  // 关闭鼠标悬停预高亮
    myView->SetShadingModel(V3d_PHONG);
    myView->TriedronDisplay(Aspect_TOTP_LEFT_LOWER, Quantity_NOC_WHITE, 0.1, V3d_WIREFRAME);
    ResizeOCCView();
}

void COCCMFCView::OnFanucM10iD12()
{
    COCCMFCDoc* pDoc = GetDocument();
    if (!pDoc || myContext.IsNull()) return;

    pDoc->LoadRobotModel();

    myContext->EraseAll(Standard_False);
    for (int i = 0; i < 7; i++) {
        if (!pDoc->m_RobotLinks[i].IsNull()) {
            myContext->Display(pDoc->m_RobotLinks[i], AIS_Shaded, 0, Standard_False);
        }
    }

    myContext->UpdateCurrentViewer();
    myView->FitAll();
    myView->Redraw();
}

void COCCMFCView::OnUpdate(CView* pSender, LPARAM lHint, CObject* pHint)
{
    CView::OnUpdate(pSender, lHint, pHint);
    COCCMFCDoc* pDoc = GetDocument();
    if (pDoc && !myContext.IsNull()) {
        for (int i = 0; i < 7; i++) {
            if (!pDoc->m_RobotLinks[i].IsNull()) {
                if (myContext->IsDisplayed(pDoc->m_RobotLinks[i])) {
                    myContext->Redisplay(pDoc->m_RobotLinks[i], Standard_False);
                }
                else {
                    myContext->Display(pDoc->m_RobotLinks[i], AIS_Shaded, 0, Standard_False);
                }
            }
        }
        if (!pDoc->m_ImportedModel.IsNull()) {
            if (myContext->IsDisplayed(pDoc->m_ImportedModel)) {
                myContext->Redisplay(pDoc->m_ImportedModel, Standard_False);
            }
            else {
                myContext->Display(pDoc->m_ImportedModel, AIS_Shaded, 0, Standard_False);
            }
        }
        myContext->UpdateCurrentViewer();
    }
    if (!myView.IsNull()) {
        myView->Redraw();
    }
}

void COCCMFCView::OnDraw(CDC* pDC)
{
    if (!myView.IsNull()) {
        myView->Redraw();
    }
}

// ========== PreCreateWindow：注册窗口类样式 ==========
BOOL COCCMFCView::PreCreateWindow(CREATESTRUCT& cs)
{
    if (!CView::PreCreateWindow(cs))
        return FALSE;
    cs.style |= WS_CLIPSIBLINGS | WS_CLIPCHILDREN;
    cs.lpszClass = AfxRegisterWndClass(
        CS_HREDRAW | CS_VREDRAW | CS_OWNDC,
        ::LoadCursor(nullptr, IDC_ARROW),
        (HBRUSH)GetStockObject(BLACK_BRUSH),
        nullptr);
    return TRUE;
}

// ========== ResizeOCCView：统一同步 WNT_Window + 视口 + 重绘 ==========
void COCCMFCView::ResizeOCCView()
{
    if (myView.IsNull() || myWindow.IsNull())
        return;
    if (!::IsWindow(m_hWnd))
        return;
    myWindow->DoResize();
    myView->MustBeResized();
    myView->Invalidate();
    myView->Redraw();
}

// ========== 在 3D 视图中显示焊缝曲线（红色粗线 + 圆点 + 编号标签） ==========
void COCCMFCView::DisplayWeldCurves(
    const TopoDS_Shape& weldShape,
    const std::vector<WeldSeam>& seams)
{
    if (myContext.IsNull())
        return;

    // 清空旧数据
    m_WeldItems.clear();
    m_WeldSelected.clear();

    //==================================================
    // 1. 显示焊缝曲线（红色）+ 圆点标记 + 编号标签
    //    每条焊缝单独创建 AIS_Shape，便于勾选后单独变色
    //==================================================
    {
        const double kDotRadius = 0.016; // 圆点半径（米），直径约 32mm

        for (size_t i = 0; i < seams.size(); ++i)
        {
            WeldDisplayItem item;

            // 将 mm 坐标转换为米（与 BRepBuilderAPI_MakeEdge 使用的坐标一致）
            gp_Pnt startPos(
                seams[i].StartPoint.X / 1000.0,
                seams[i].StartPoint.Y / 1000.0,
                seams[i].StartPoint.Z / 1000.0);
            gp_Pnt endPos(
                seams[i].EndPoint.X / 1000.0,
                seams[i].EndPoint.Y / 1000.0,
                seams[i].EndPoint.Z / 1000.0);

            // ===== 焊缝红线（单条边） =====
            {
                BRepBuilderAPI_MakeEdge mkEdge(startPos, endPos);
                if (mkEdge.IsDone())
                {
                    item.lineEdge = ::new AIS_Shape(mkEdge.Edge());
                    item.lineEdge->SetColor(Quantity_NOC_RED);
                    item.lineEdge->SetWidth(3.0);
                    myContext->Display(item.lineEdge, Standard_False);
                }
            }

            // ===== 起点红色圆点 =====
            {
                BRepPrimAPI_MakeSphere mkSphere(gp_Ax2(startPos, gp_Dir(0, 0, 1)), kDotRadius);
                if (mkSphere.IsDone())
                {
                    item.startDot = ::new AIS_Shape(mkSphere.Shape());
                    item.startDot->SetColor(Quantity_NOC_RED);
                    myContext->Display(item.startDot, Standard_False);
                }
            }

            // ===== 终点红色圆点 =====
            {
                BRepPrimAPI_MakeSphere mkSphere(gp_Ax2(endPos, gp_Dir(0, 0, 1)), kDotRadius);
                if (mkSphere.IsDone())
                {
                    item.endDot = ::new AIS_Shape(mkSphere.Shape());
                    item.endDot->SetColor(Quantity_NOC_RED);
                    myContext->Display(item.endDot, Standard_False);
                }
            }


            m_WeldItems.push_back(item);
        }
        m_WeldSelected.assign(m_WeldItems.size(), false);
    }

    //==================================================
    // 2. 刷新视图
    //==================================================
    myContext->UpdateCurrentViewer();

    if (!myView.IsNull())
        myView->Redraw();
}

// ==============================
// SetWeldSeamColor
// 功能：根据列表勾选状态改变焊缝颜色
// selected=true  -> 高亮（蓝色）
// selected=false -> 恢复默认（红色）
// ==============================
void COCCMFCView::SetWeldSeamColor(int index, bool selected)
{
    if (index < 0 || index >= (int)m_WeldItems.size())
        return;
    if (myContext.IsNull())
        return;

    const auto& item = m_WeldItems[index];

    if (!item.lineEdge.IsNull())
    {
        if (selected)
            item.lineEdge->SetColor(Quantity_NOC_CYAN1);  // 高亮为青色
        else
            item.lineEdge->SetColor(Quantity_NOC_RED);     // 恢复红色
        myContext->Redisplay(item.lineEdge, Standard_False);
    }

    if (!item.startDot.IsNull())
    {
        if (selected)
            item.startDot->SetColor(Quantity_NOC_CYAN1);
        else
            item.startDot->SetColor(Quantity_NOC_RED);
        myContext->Redisplay(item.startDot, Standard_False);
    }

    if (!item.endDot.IsNull())
    {
        if (selected)
            item.endDot->SetColor(Quantity_NOC_CYAN1);
        else
            item.endDot->SetColor(Quantity_NOC_RED);
        myContext->Redisplay(item.endDot, Standard_False);
    }

    if (!myView.IsNull())
        myView->Redraw();
}

void COCCMFCView::OnPaint()
{
    CPaintDC dc(this);
    if (myView.IsNull() || myWindow.IsNull())
        return;
    myWindow->DoResize();
    myView->MustBeResized();
    myView->Redraw();
}

void COCCMFCView::OnSize(UINT nType, int cx, int cy)
{
    CView::OnSize(nType, cx, cy);
    if (myView.IsNull() || myWindow.IsNull())
        return;
    myWindow->DoResize();
    myView->MustBeResized();
    Invalidate(FALSE);
}

BOOL COCCMFCView::OnEraseBkgnd(CDC* pDC) { return TRUE; }

void COCCMFCView::OnLButtonDown(UINT nFlags, CPoint point) {
    m_OldMousePoint = point;
    m_MouseDownPoint = point;
    m_bIsRotating = FALSE;
    if (!myContext.IsNull() && !myView.IsNull()) {
        myContext->MoveTo(point.x, point.y, myView, Standard_True);
    }
}

void COCCMFCView::OnRButtonDown(UINT nFlags, CPoint point) { m_OldMousePoint = point; }

void COCCMFCView::OnMouseMove(UINT nFlags, CPoint point) {
    if (!myContext.IsNull() && !myView.IsNull()) {
        myContext->MoveTo(point.x, point.y, myView, Standard_True);
    }

    if (nFlags & MK_LBUTTON) {
        int dx = point.x - m_MouseDownPoint.x;
        int dy = point.y - m_MouseDownPoint.y;
        if (!m_bIsRotating && (abs(dx) > 5 || abs(dy) > 5)) {
            if (!myView.IsNull()) {
                myView->StartRotation(m_MouseDownPoint.x, m_MouseDownPoint.y);
                m_bIsRotating = TRUE;
            }
        }
        if (m_bIsRotating && !myView.IsNull()) {
            myView->Rotation(point.x, point.y);
            myView->Redraw();
        }
    }
    else if (nFlags & MK_RBUTTON) {
        if (!myView.IsNull()) { myView->Pan(point.x - m_OldMousePoint.x, m_OldMousePoint.y - point.y); m_OldMousePoint = point; }
    }
}

BOOL COCCMFCView::OnMouseWheel(UINT nFlags, short zDelta, CPoint pt) {
    if (!myView.IsNull()) {
        CPoint clientPt = pt;
        ScreenToClient(&clientPt);
        myView->StartZoomAtPoint(clientPt.x, clientPt.y);
        int zoomStep = zDelta / 20;
        myView->ZoomAtPoint(clientPt.x, clientPt.y, clientPt.x + zoomStep, clientPt.y + zoomStep);
        myView->Redraw();
    }
    return TRUE;
}

COCCMFCDoc* COCCMFCView::GetDocument() const { return reinterpret_cast<COCCMFCDoc*>(m_pDocument); }

void COCCMFCView::DeleteSelectedModel()
{
    if (myContext.IsNull()) return;
    COCCMFCDoc* pDoc = GetDocument();
    if (!pDoc) return;

    // 移除导入的模型（保留机器人）
    if (!pDoc->m_ImportedModel.IsNull()) {
        myContext->Remove(pDoc->m_ImportedModel, Standard_False);
        pDoc->m_ImportedModel.Nullify();
    }

    // 清除所有焊缝显示元素
    for (const auto& item : m_WeldItems) {
        if (!item.lineEdge.IsNull()) myContext->Remove(item.lineEdge, Standard_False);
        if (!item.startDot.IsNull()) myContext->Remove(item.startDot, Standard_False);
        if (!item.endDot.IsNull()) myContext->Remove(item.endDot, Standard_False);
    }
    m_WeldItems.clear();
    m_WeldSelected.clear();

    // 同步清除左侧列表
    CMainFrame* pMain = (CMainFrame*)AfxGetMainWnd();
    if (pMain)
    {
        pMain->m_wndControlPane.m_wndDlg.m_pageWeld.ClearWeldList();
    }

    myContext->UpdateCurrentViewer();
    if (!myView.IsNull()) myView->Redraw();
}

void COCCMFCView::OnLButtonUp(UINT nFlags, CPoint point) {
    if (!myContext.IsNull() && !myView.IsNull()) {
        myContext->MoveTo(point.x, point.y, myView, Standard_True);
    }
    if (!m_bIsRotating && !myContext.IsNull()) {
        myContext->SelectDetected(AIS_SelectionScheme_Replace);

        // 检测是否点击了焊缝对象
        myContext->InitSelected();
        if (myContext->MoreSelected())
        {
            Handle(AIS_InteractiveObject) selObj = myContext->SelectedInteractive();
            for (size_t i = 0; i < m_WeldItems.size(); i++)
            {
                const auto& item = m_WeldItems[i];
                bool hit = false;
                if (!item.lineEdge.IsNull() && selObj == item.lineEdge) hit = true;
                if (!item.startDot.IsNull() && selObj == item.startDot) hit = true;
                if (!item.endDot.IsNull() && selObj == item.endDot) hit = true;

                if (hit)
                {
                    bool newState = !m_WeldSelected[i];
                    m_WeldSelected[i] = newState;
                    SetWeldSeamColor((int)i, newState);

                    // 同步复选框状态
                    CMainFrame* pMain = (CMainFrame*)AfxGetMainWnd();
                    if (pMain)
                    {
                        pMain->m_wndControlPane.m_wndDlg.m_pageWeld.SetWeldCheckbox((int)i, newState);
                    }

                    myContext->ClearSelected(Standard_False);
                    break;
                }
            }
        }
    }
}
