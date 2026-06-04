#pragma once

#include "framework.h"

#include <AIS_Shape.hxx>
#include <gp_Trsf.hxx>
#include <gp_Ax1.hxx>
#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>
#include <gp_Vec.hxx>
#include <gp_Pln.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Solid.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <string>
#include <vector>
#include <map>

// ==============================
// WeldPose
// 功能：焊缝位姿结构体
// X,Y,Z: 空间位置坐标
// x,y,z: 姿态向量（法线组合方向）
// ==============================
struct WeldPose {
    double X = 0, Y = 0, Z = 0;
    double x = 0, y = 0, z = 0;
};

// ==============================
// WeldSeam
// 功能：焊缝信息结构体（专利 CN 120339259 A）
// 包含：几何参数、形成面、截断面、关联子模型等完整信息
// ==============================
struct WeldSeam {
    int Id = 0;                         // 焊缝唯一编号
    double Length = 0;                  // 焊缝长度
    std::string CurveType;              // 曲线类型（Line/Circle/...）
    WeldPose StartPoint;                // 起点坐标+姿态
    WeldPose EndPoint;                  // 终点坐标+姿态
    double CenterX = 0, CenterY = 0, CenterZ = 0;  // 圆心（圆形焊缝）
    double Radius = 0;                  // 半径（圆形焊缝）
    int ShapeIdx1 = -1, ShapeIdx2 = -1; // 关联的两个子模型索引
    TopoDS_Face FormingFace1;           // 形成面1（产生焊缝的面）
    TopoDS_Face FormingFace2;           // 形成面2（产生焊缝的面）
    TopoDS_Face TruncationFace1;        // 截断面1（切断焊缝的面）
    TopoDS_Face TruncationFace2;        // 截断面2（切断焊缝的面）
    int SectionCount = 0;               // 截断面数量(0/1/2)
    TopoDS_Edge EdgeGeom;               // 焊缝几何边（用于截断计算）
};

// ==============================
// WeldExtractor
// 功能：基于专利 CN 120339259 A 的焊缝提取算法
// 5步法：(1)模型读取映射 (2)初始焊缝提取 (3)截断检测 (4)内部截面 (5)去重输出
// 替换旧的 FindEdge2 类
// ==============================
class WeldExtractor {
public:
    TopoDS_Shape inputShape;                    // 输入模型
    std::vector<WeldSeam> WeldSeams;             // 输出焊缝

    WeldExtractor();
    ~WeldExtractor();

    // 主入口：执行完整5步焊缝提取
    void Compute();

private:
    // ----- 步骤1: 模型读取与映射 -----
    std::vector<TopoDS_Solid> m_Solids;         // 子模型列表

    void BuildTopologyMaps();

    // ----- 步骤2: 初始焊缝提取（基于零件对接触区域） -----
    void ExtractInitialWelds();

    // ----- 步骤3: 焊缝截断检测 -----
    void DetectTruncations();
    int CountIntersectionPoints(const TopoDS_Edge& edge, const TopoDS_Solid& solid,
                                 std::vector<gp_Pnt>& outPts);
    void HandleSingleSideTruncation(WeldSeam& seam, const TopoDS_Solid& cuttingSolid);
    bool HandleDoubleSideTruncation(WeldSeam& seam, const TopoDS_Solid& cuttingSolid, WeldSeam& outNewSeam);

    // ----- 步骤4: 内部截面检测 -----
    void DetectInternalSections();
    bool CheckNormalConsistency(const gp_Pnt& fromPnt, const gp_Pnt& toPnt,
                                 const TopoDS_Face& face);

    // ----- 步骤5: 去重输出 -----
    void RemoveDuplicateSeams();
    bool AreSeamsDuplicate(const WeldSeam& a, const WeldSeam& b, double tol);

    // ----- 步骤6: 后处理拓扑清洗 -----
    void MergeAndAlignBrokenSeams();                    // 碎段长直线熔接
    void ValidateAndClipSeamsByFaces(const TopoDS_Shape& globalShape); // 孔洞穿透过滤
    void FilterByWeldingProcess(const TopoDS_Shape& globalShape);      // 工艺特征洗白
    static bool CheckIfFlatContact(const TopoDS_Edge& edge, const TopoDS_Face& face1, const TopoDS_Face& face2);
    static bool IsConvexEdge(const TopoDS_Edge& edge, const TopoDS_Face& face1, const TopoDS_Face& face2);

    // ----- 辅助工具 -----
    double WeldEdgeLength(const TopoDS_Edge& edge);
    gp_Vec ComputeFaceNormal(const TopoDS_Face& face);
    bool IsFacePlanar(const TopoDS_Face& face);
};

class COCCMFCDoc : public CDocument
{
    DECLARE_DYNCREATE(COCCMFCDoc)

public:
    COCCMFCDoc() noexcept;
    virtual ~COCCMFCDoc();

    Handle(AIS_Shape) m_RobotLinks[7];
    Handle(AIS_InteractiveObject) m_RobotAssembly;
    Handle(AIS_Shape) m_ImportedModel;
    double m_ImportPosition[3];
    double m_JointAngles[6];

    void LoadRobotModel();
    void UpdateTCPNumericalDisplay();
    void OnHandleTCPInput(double x, double y, double z, double w, double p, double r);
    bool ComputeInverseKinematics(const gp_Trsf& T_Target);
    void UpdateRobotPose();
    void SetImportedModelPosition(double x, double y, double z);

    virtual BOOL OnNewDocument();
    virtual void Serialize(CArchive& ar) override;

    DECLARE_MESSAGE_MAP()

#ifdef _DEBUG
    virtual void AssertValid() const override;
    virtual void Dump(CDumpContext& dc) const override;
#endif

public:
    void LoadModelFromFile(const CString& filePath);
    std::vector<WeldSeam> ComputeWeldSeams();
    TopoDS_Shape GetWeldSeamShape() const { return m_WeldSeamShape; }

protected:
    TopoDS_Shape m_WeldSeamShape;
};
