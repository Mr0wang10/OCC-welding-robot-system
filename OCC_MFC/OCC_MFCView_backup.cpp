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
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>

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

// ========== 在 3D 视图中显示焊缝曲线（红色粗线 + 顶点标记） ==========
void COCCMFCView::DisplayWeldCurves(const TopoDS_Shape& weldShape, const std::vector<WeldSeam>& seams)
{
    if (myContext.IsNull())
        return;

    // 显示红色粗焊缝边线
    if (!weldShape.IsNull()) {
        Handle(AIS_Shape) aisWeld = ::new AIS_Shape(weldShape);
        aisWeld->SetColor(Quantity_NOC_RED);
        aisWeld->SetWidth(3.0);
        myContext->Display(aisWeld, Standard_False);
    }

    // 在焊缝起点放置标记顶点（彩色）
    Quantity_NameOfColor colors[] = {
        Quantity_NOC_GREEN, Quantity_NOC_BLUE, Quantity_NOC_YELLOW,
        Quantity_NOC_ORANGE, Quantity_NOC_PINK, Quantity_NOC_CYAN1
    };

    int seamCount = (int)seams.size();
    for (int i = 0; i < seamCount; i++)
    {
        gp_Pnt pos(seams[i].StartPoint.X, seams[i].StartPoint.Y, seams[i].StartPoint.Z);
        BRepBuilderAPI_MakeVertex mkVert(pos);
        if (mkVert.IsDone()) {
            Handle(AIS_Shape) aisMarker = ::new AIS_Shape(mkVert.Shape());
            aisMarker->SetColor(colors[i % 6]);
            aisMarker->SetWidth(5.0);
            myContext->Display(aisMarker, Standard_False);
        }
    }

    myContext->UpdateCurrentViewer();
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
    if (!pDoc->m_ImportedModel.IsNull() && myContext->IsSelected(pDoc->m_ImportedModel)) {
        myContext->Remove(pDoc->m_ImportedModel, Standard_True);
        pDoc->m_ImportedModel.Nullify();
    }
}

void COCCMFCView::OnLButtonUp(UINT nFlags, CPoint point) {
    if (!myContext.IsNull() && !myView.IsNull()) {
        myContext->MoveTo(point.x, point.y, myView, Standard_True);
    }
    if (!m_bIsRotating && !myContext.IsNull()) {
        myContext->SelectDetected(AIS_SelectionScheme_Replace);
    }
}
