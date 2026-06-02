#pragma once

#include <Standard_Handle.hxx>
#include <AIS_TextLabel.hxx>
#include <AIS_Shape.hxx>
#include <vector>

class V3d_Viewer;
class V3d_View;
class AIS_InteractiveContext;
class OpenGl_GraphicDriver;
class WNT_Window;
class TopoDS_Shape;
struct WeldSeam;

// ==============================
// WeldDisplayItem
// 功能：焊缝显示元素存储，用于勾选后改变颜色
// ==============================
struct WeldDisplayItem {
    Handle(AIS_Shape) lineEdge;       // 焊缝红线（单条边）
    Handle(AIS_Shape) startDot;       // 起点红色圆点
    Handle(AIS_Shape) endDot;         // 终点红色圆点
};

class COCCMFCView : public CView
{
protected:
	COCCMFCView() noexcept;
	DECLARE_DYNCREATE(COCCMFCView)

public:
	COCCMFCDoc* GetDocument() const;
		void ResizeOCCView();

protected:
	Handle(V3d_Viewer)             myViewer;
	Handle(V3d_View)               myView;
	Handle(AIS_InteractiveContext) myContext;
	Handle(OpenGl_GraphicDriver)   myGraphicDriver;
	Handle(WNT_Window)             myWindow;
	CPoint                         m_OldMousePoint;
	CPoint                         m_MouseDownPoint;
	std::vector<WeldDisplayItem>   m_WeldItems;       // 焊缝显示元素列表，用于勾选变色
	std::vector<bool>              m_WeldSelected;    // 焊缝勾选状态跟踪
	BOOL                           m_bIsRotating;     // 是否处于旋转拖拽模式


public:
	virtual void OnDraw(CDC* pDC);
	virtual void OnInitialUpdate();
	virtual ~COCCMFCView();
	virtual void OnUpdate(CView* pSender, LPARAM lHint, CObject* pHint) override;
	void DeleteSelectedModel();
	void DisplayWeldCurves(const TopoDS_Shape& weldShape, const std::vector<WeldSeam>& seams);
	void SetWeldSeamColor(int index, bool selected);  // 勾选/取消勾选焊缝时改变颜色
	// 从 3D 视图中删除当前选中的导入模型（由 CRobotControlDlg 调用）
	// 如果选中的是导入模型，则从 AIS 上下文中移除并清空文档引用
protected:
	afx_msg void OnLButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnRButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnLButtonUp(UINT nFlags, CPoint point);
	// 左键松开：短点击选中模型，拖拽则旋转视图
	afx_msg void OnMouseMove(UINT nFlags, CPoint point);
	afx_msg BOOL OnMouseWheel(UINT nFlags, short zDelta, CPoint pt);
	afx_msg void OnPaint();
	virtual BOOL PreCreateWindow(CREATESTRUCT& cs);
	afx_msg void OnSize(UINT nType, int cx, int cy);
	afx_msg BOOL OnEraseBkgnd(CDC* pDC);
	afx_msg void OnButtonClearAll();
	afx_msg void OnFanucM10iD12();
	DECLARE_MESSAGE_MAP()
};
