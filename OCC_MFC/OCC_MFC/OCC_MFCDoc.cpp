#include "pch.h"
#include "framework.h"

// 彻底禁用 MFC 的 DEBUG_NEW 宏，防止与 OCCT 的 Handle/::new 冲突
#ifdef _DEBUG
#undef new
#endif

// 防止 windows.h 中的 max/min 宏污染 std::max / numeric_limits::max
#undef max
#undef min

#include <AIS_Shape.hxx>
#include <STEPControl_Reader.hxx>
#include <IGESControl_Reader.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <RWStl.hxx>
#include <Poly_Triangulation.hxx>
#include <OSD_Path.hxx>
#include <BRep_Builder.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <Quantity_Color.hxx>
#include <gp_Trsf.hxx>
#include <gp_Ax1.hxx>
#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>
#include <gp_Vec.hxx>
#include <gp_Quaternion.hxx>
#include <gp_EulerSequence.hxx>

#include <array>
#include <vector>
#include <limits>
#include <algorithm>
#include <cmath>
#include <string>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <TopExp_Explorer.hxx>
#include <BRep_Tool.hxx>
#include <Geom_Surface.hxx>
#include <Geom_Plane.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_ListOfShape.hxx>
#include <TopExp.hxx>
#include <BRepTools.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <TopoDS_Compound.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <Geom_Curve.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <GC_MakeLine.hxx>
#include <Geom_Line.hxx>
#include <GeomAPI_ProjectPointOnCurve.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <GProp_GProps.hxx>
#include <BRepGProp.hxx>
#include <Geom_Circle.hxx>
#include <GC_MakeCircle.hxx>
#include <Geom_ToroidalSurface.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <Precision.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Solid.hxx>
#include <TopoDS_Edge.hxx>
#include <TopAbs.hxx>
#include <gp_Dir.hxx>
#include <gp_Circ.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Torus.hxx>
#include <Standard_Type.hxx>

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Section.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <IntCurvesFace_ShapeIntersector.hxx>
#include <ShapeAnalysis_FreeBounds.hxx>
#include <GeomAdaptor_Curve.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepLib.hxx>
#include <TopoDS_Wire.hxx>
#include <TopTools_ListIteratorOfListOfShape.hxx>

#include "MainFrm.h"
#include "OCC_MFCDoc.h"

// ==============================
// 机器人几何参数（单位：mm）
// 这些参数用于正解/逆解计算
// ==============================
static const double d1 = 450.0;  // 基座到J2高度
static const double a1 = 75.0;   // J1到J2在X方向偏移
static const double a2 = 640.0;  // 大臂长度
static const double a3 = 195.0;  // 前臂补偿长度
static const double d4 = 700.0;  // 前臂主长度
static const double d6 = 75.0;   // 法兰到TCP参考点/腕部末端偏移

namespace
{
    // ==============================
    // 常量定义
    // ==============================
    constexpr double kDegToRad = M_PI / 180.0; // 角度 -> 弧度
    constexpr double kRadToDeg = 180.0 / M_PI; // 弧度 -> 角度
    constexpr double kEps = 1.0e-8;            // 数值计算容差
    // ==============================
    // PathExists
    // 功能：判断指定路径是否存在
    // 调用者：InitOcctResourcePaths
    // 关键逻辑：使用 Windows API 判断文件或目录，避免引入额外文件系统依赖
    // ==============================
    bool PathExists(const CString& path)
    {
        return !path.IsEmpty() && GetFileAttributes(path) != INVALID_FILE_ATTRIBUTES;
    }

    // ==============================
    // NormalizeDir
    // 功能：统一目录分隔符并去掉末尾反斜杠
    // 调用者：InitOcctResourcePaths
    // 关键逻辑：OCCT 环境变量使用普通目录路径，不需要末尾分隔符
    // ==============================
    CString NormalizeDir(const CString& path)
    {
        CString normalized = path;
        normalized.Replace(_T('/'), _T('\\'));
        while (!normalized.IsEmpty() && normalized.Right(1) == _T("\\"))
        {
            normalized = normalized.Left(normalized.GetLength() - 1);
        }
        return normalized;
    }

    // ==============================
    // GetParentDir
    // 功能：取得上一级目录
    // 调用者：InitOcctResourcePaths
    // 关键逻辑：只做字符串级路径拆分，避免改变当前工作目录
    // ==============================
    CString GetParentDir(const CString& path)
    {
        CString normalized = NormalizeDir(path);
        int pos = normalized.ReverseFind(_T('\\'));
        if (pos <= 0)
            return CString();
        return normalized.Left(pos);
    }

    // ==============================
    // AddOcctResourceCandidate
    // 功能：收集可能的 OCCT src 资源目录
    // 调用者：InitOcctResourcePaths
    // 关键逻辑：只加入包含 XSTEPResource/STEP 和 StdResource/Plugin 的有效目录
    // ==============================
    void AddOcctResourceCandidate(std::vector<CString>& candidates, const CString& srcDir)
    {
        CString normalized = NormalizeDir(srcDir);
        if (normalized.IsEmpty())
            return;

        CString stepDefaults = normalized + _T("\\XSTEPResource\\STEP");
        CString pluginDefaults = normalized + _T("\\StdResource\\Plugin");
        if (!PathExists(stepDefaults) || !PathExists(pluginDefaults))
            return;

        for (const CString& item : candidates)
        {
            if (item.CompareNoCase(normalized) == 0)
                return;
        }
        candidates.push_back(normalized);
    }

    // ==============================
    // SetOcctEnvPath
    // 功能：设置 OCCT 资源环境变量
    // 调用者：ApplyOcctResourceDir
    // 关键逻辑：OCCT 7.8 读取 CSF_STEPDefaults / CSF_IGESDefaults 等变量
    // ==============================
    void SetOcctEnvPath(LPCTSTR name, const CString& value)
    {
        if (!value.IsEmpty())
        {
            SetEnvironmentVariable(name, value);
        }
    }

    // ==============================
    // ApplyOcctResourceDir
    // 功能：把 OCCT src 目录展开为 STEP/IGES 所需的资源环境变量
    // 调用者：InitOcctResourcePaths
    // 关键逻辑：设置 OCCT 7.8 实际使用的变量，同时兼容旧代码中曾使用的变量名
    // ==============================
    void ApplyOcctResourceDir(const CString& srcDir)
    {
        CString xstepDir = srcDir + _T("\\XSTEPResource");       // STEP/IGES 默认参数目录
        CString stdDir = srcDir + _T("\\StdResource");           // 标准插件和默认参数目录
        CString xsMessageDir = srcDir + _T("\\XSMessage");       // 数据交换消息资源目录
        CString shMessageDir = srcDir + _T("\\SHMessage");       // Shape Healing 消息资源目录

        SetOcctEnvPath(_T("CSF_OCCTResourcePath"), srcDir);
        SetOcctEnvPath(_T("CSF_STEPDefaults"), xstepDir);
        SetOcctEnvPath(_T("CSF_IGESDefaults"), xstepDir);
        SetOcctEnvPath(_T("CSF_StandardDefaults"), stdDir);
        SetOcctEnvPath(_T("CSF_PluginDefaults"), stdDir);
        SetOcctEnvPath(_T("CSF_XCAFDefaults"), stdDir);
        SetOcctEnvPath(_T("CSF_TObjDefaults"), stdDir);
        SetOcctEnvPath(_T("CSF_StandardLiteDefaults"), stdDir);
        SetOcctEnvPath(_T("CSF_MIGRATION_TYPES"), stdDir + _T("\\MigrationSheet.txt"));
        SetOcctEnvPath(_T("CSF_XSMessage"), xsMessageDir);
        SetOcctEnvPath(_T("CSF_SHMessage"), shMessageDir);

        // 兼容旧版/历史修复里使用过的变量名，避免用户环境里有混合配置时失效
        SetOcctEnvPath(_T("CSF_StepResourcePath"), xstepDir);
        SetOcctEnvPath(_T("CSF_StepDefaultsPath"), xstepDir);
        SetOcctEnvPath(_T("CSF_IGESResourcePath"), xstepDir);
        SetOcctEnvPath(_T("CSF_XSProviderPath"), xstepDir);
    }

    // ==============================
    // InitOcctResourcePaths
    // 功能：自动初始化 STEP/IGES 导入所需的 OCCT 资源路径
    // 调用者：LoadModelFromFile 的 STEP/IGES 分支
    // 关键逻辑：优先使用环境变量，其次从已加载 DLL 路径反推 build/install/source 目录
    // ==============================
    bool InitOcctResourcePaths()
    {
        std::vector<CString> candidates; // 候选 OCCT src 资源目录

        TCHAR envPath[MAX_PATH * 2] = { 0 };
        if (GetEnvironmentVariable(_T("CSF_OCCTResourcePath"), envPath, _countof(envPath)) > 0)
        {
            AddOcctResourceCandidate(candidates, envPath);
        }
        if (GetEnvironmentVariable(_T("CASROOT"), envPath, _countof(envPath)) > 0)
        {
            AddOcctResourceCandidate(candidates, CString(envPath) + _T("\\src"));
        }

        HMODULE hMod = GetModuleHandle(_T("TKDESTEP.dll"));
        if (hMod == NULL)
        {
            hMod = GetModuleHandle(_T("TKXSBase.dll"));
        }
        if (hMod != NULL)
        {
            TCHAR dllPath[MAX_PATH * 2] = { 0 };
            if (GetModuleFileName(hMod, dllPath, _countof(dllPath)) > 0)
            {
                CString binDir = GetParentDir(dllPath);          // ...\win64\vc14\bind 或 bindd
                CString vcDir = GetParentDir(binDir);            // ...\win64\vc14
                CString archDir = GetParentDir(vcDir);           // ...\win64
                CString buildDir = GetParentDir(archDir);        // ...\OCCT-7_8_0bulid
                CString occtRoot = GetParentDir(buildDir);       // ...\D:\OCCT780

                AddOcctResourceCandidate(candidates, buildDir + _T("\\OCCT-7_8_0install\\src"));
                AddOcctResourceCandidate(candidates, occtRoot + _T("\\OCCT-7_8_0\\OCCT-7_8_0\\src"));
            }
        }

        AddOcctResourceCandidate(candidates, _T("D:\\OCCT780\\OCCT-7_8_0bulid\\OCCT-7_8_0install\\src"));
        AddOcctResourceCandidate(candidates, _T("D:\\OCCT780\\OCCT-7_8_0\\OCCT-7_8_0\\src"));

        if (candidates.empty())
            return false;

        ApplyOcctResourceDir(candidates.front());
        return true;
    }

    // ==============================
    // ToOcctFilePath
    // 功能：把 MFC CString 路径转为 OCCT 读取文件使用的 UTF-8 字节串
    // 调用者：LoadModelFromFile
    // 关键逻辑：OCCT 的 OSD 文件系统在 Windows 下可用 UTF-8 路径打开中文文件名
    // ==============================
    CStringA ToOcctFilePath(const CString& filePath)
    {
        return CStringA(CW2A(filePath, CP_UTF8));
    }

    // ==============================
    // StepStatusText
    // 功能：把 IFSelect 状态码转为可读说明
    // 调用者：LoadModelFromFile
    // 关键逻辑：避免把 RetError(2) 误判为读取成功
    // ==============================
    CString StepStatusText(IFSelect_ReturnStatus status)
    {
        switch (status)
        {
        case IFSelect_RetVoid:
            return _T("RetVoid：未创建可读取的数据");
        case IFSelect_RetDone:
            return _T("RetDone：读取成功");
        case IFSelect_RetError:
            return _T("RetError：文件无法打开或 STEP 语法解析失败");
        case IFSelect_RetFail:
            return _T("RetFail：读取过程中执行失败");
        case IFSelect_RetStop:
            return _T("RetStop：读取被中断");
        default:
            return _T("未知状态");
        }
    }

    // ==============================
    // MakeImportedModelTransform
    // 功能：生成导入模型的显示变换
    // 调用者：LoadModelFromFile、SetImportedModelPosition
    // 关键逻辑：机器人舞台按米显示，导入模型本体和位置都从 mm 统一缩放到 m
    // ==============================
    gp_Trsf MakeImportedModelTransform(const double importPosition[3])
    {
        const double s = 0.001; // 导入模型显示比例：mm -> m，保持与机器人连杆舞台比例一致
        gp_Trsf T;
        // SetScale：把导入模型本体按机器人舞台比例缩小，缩放中心为机器人基坐标原点
        T.SetScale(gp_Pnt(0.0, 0.0, 0.0), s);
        // SetTranslationPart：位置编辑框仍以 mm 输入，这里转换为 m 后叠加到缩放变换上
        T.SetTranslationPart(gp_Vec(importPosition[0] * s, importPosition[1] * s, importPosition[2] * s));
        return T;
    }
    // 三维向量 / 3x3矩阵简写
    using Vec3 = std::array<double, 3>;
    using Mat3 = std::array<std::array<double, 3>, 3>;

    // ==============================
    // 逆解候选解
    // qRad   : 6个关节角（弧度）
    // cost   : 与当前姿态的“距离代价”，越小越优
    // posErr : 正解后的位置误差
    // rotErr : 正解后的姿态误差
    // ==============================
    struct IkCandidate
    {
        std::array<double, 6> qRad{};
        double cost = (std::numeric_limits<double>::max)();
        double posErr = (std::numeric_limits<double>::max)();
        double rotErr = (std::numeric_limits<double>::max)();
    };

    // ==============================
    // Clamp
    // 功能：把数值限制在 [lo, hi] 范围内
    // 用途：避免 acos/atan2 前因浮点误差越界
    // ==============================
    double Clamp(double v, double lo, double hi)
    {
        return (v < lo) ? lo : (v > hi ? hi : v);
    }

    // ==============================
    // NormalizeRad
    // 功能：把弧度归一化到 (-pi, pi]
    // 用途：便于比较角度、选择最接近当前姿态的逆解
    // ==============================
    double NormalizeRad(double v)
    {
        while (v <= -M_PI) v += 2.0 * M_PI;
        while (v > M_PI) v -= 2.0 * M_PI;
        return v;
    }

    // ==============================
    // NormalizeDeg
    // 功能：把角度归一化到 (-180, 180]
    // 用途：最终写回界面显示和关节数组
    // ==============================
    double NormalizeDeg(double v)
    {
        while (v <= -180.0) v += 360.0;
        while (v > 180.0) v -= 360.0;
        return v;
    }

    // ==============================
    // MakeTranslation
    // 功能：构造平移变换矩阵
    // ==============================
    gp_Trsf MakeTranslation(double x, double y, double z)
    {
        gp_Trsf T;
        T.SetTranslation(gp_Vec(x, y, z));
        return T;
    }

    // ==============================
    // MakeRotationX / Y / Z
    // 功能：分别构造绕 X/Y/Z 轴的旋转变换
    // 输入：弧度
    // ==============================
    gp_Trsf MakeRotationX(double angRad)
    {
        gp_Trsf T;
        T.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(1, 0, 0)), angRad);
        return T;
    }

    gp_Trsf MakeRotationY(double angRad)
    {
        gp_Trsf T;
        T.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 1, 0)), angRad);
        return T;
    }

    gp_Trsf MakeRotationZ(double angRad)
    {
        gp_Trsf T;
        T.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), angRad);
        return T;
    }

    // ==============================
    // BuildLcsTransform
    // 功能：
    //   构造用户参考坐标系 LCS 的姿态修正矩阵
    //
    // 说明：
    //   这里按你前面确认过的 Fanuc 用户坐标系 4 来写
    //   该矩阵用于把“机器人法兰坐标系”映射到“用户显示坐标系”
    // ==============================
    gp_Trsf BuildLcsTransform()
    {
        gp_Trsf T;
        T.SetValues(
            0, 0, 1, 0,
            0, 1, 0, 0,
            -1, 0, 0, 0);
        return T;
    }

    // ==============================
    // BuildTcpCalibration
    // 功能：
    //   构造 TCP 标定矩阵
    //
    // 说明：
    //   这个矩阵通常来自实测标定结果，包含：
    //   1. TCP 相对法兰/工具基坐标系的姿态
    //   2. TCP 相对法兰/工具基坐标系的位置偏移
    //
    // 注意：
    //   这里直接使用你原程序中的标定数据
    // ==============================
    gp_Trsf BuildTcpCalibration()
    {
        gp_Trsf T;
        T.SetValues(
            0.9107, 0.1393, 0.3888, -79.2,
            0.0830, -0.9840, 0.1583, -0.7,
            0.4046, -0.1119, -0.9077, 393.6);
        return T;
    }

    // ==============================
    // BuildFlangeTransformFromRadians
    // 功能：
    //   根据 6 个关节角（弧度）计算法兰位姿
    //
    // 输出：
    //   返回从机器人基坐标系到法兰坐标系的变换矩阵
    //
    // 计算步骤：
    //   1. J1 绕 Z 轴旋转，再平移到 J2 位置
    //   2. J2 绕 Y 轴旋转，再沿 Z 平移大臂长度
    //   3. J3 绕 Y 轴旋转，再平移到腕部前端
    //   4. J4 绕 X 轴旋转
    //   5. J5 绕 Y 轴旋转
    //   6. 沿当前工具 X 轴平移 d6
    //   7. J6 再绕 X 轴旋转
    //
    // 说明：
    //   这里把 d6 放在 J5 之后、J6 之前，是为了让腕点推导和逆解一致
    // ==============================
    gp_Trsf BuildFlangeTransformFromRadians(const std::array<double, 6>& qRad)
    {
        gp_Trsf T;

        // 第1轴：底座回转
        T.Multiply(MakeRotationZ(qRad[0]));
        T.Multiply(MakeTranslation(a1, 0.0, d1));

        // 第2轴：大臂抬起
        T.Multiply(MakeRotationY(qRad[1]));
        T.Multiply(MakeTranslation(0.0, 0.0, a2));

        // 第3轴：小臂抬起
        T.Multiply(MakeRotationY(qRad[2]));
        T.Multiply(MakeTranslation(d4, 0.0, a3));

        // 第4~6轴：腕部
        T.Multiply(MakeRotationX(qRad[3]));
        T.Multiply(MakeRotationY(qRad[4]));
        T.Multiply(MakeTranslation(d6, 0.0, 0.0));
        T.Multiply(MakeRotationX(qRad[5]));

        return T;
    }

    // ==============================
    // BuildFlangeTransformFromDegrees
    // 功能：
    //   根据 6 个关节角（角度）计算法兰位姿
    //
    // 处理：
    //   先把角度转为弧度，再调用弧度版正解
    // ==============================
    gp_Trsf BuildFlangeTransformFromDegrees(const double jointAnglesDeg[6])
    {
        std::array<double, 6> qRad{};
        for (int i = 0; i < 6; ++i)
        {
            qRad[i] = jointAnglesDeg[i] * kDegToRad;
        }
        return BuildFlangeTransformFromRadians(qRad);
    }

    // ==============================
    // GetRotationMatrix
    // 功能：
    //   从 OCCT 的 gp_Trsf 中提取 3x3 旋转矩阵
    // ==============================
    Mat3 GetRotationMatrix(const gp_Trsf& T)
    {
        gp_Mat M = T.HVectorialPart();
        return { {
            {{ M(1, 1), M(1, 2), M(1, 3) }},
            {{ M(2, 1), M(2, 2), M(2, 3) }},
            {{ M(3, 1), M(3, 2), M(3, 3) }}
        } };
    }

    // ==============================
    // GetTranslationVector
    // 功能：
    //   从 gp_Trsf 中提取平移向量
    // ==============================
    Vec3 GetTranslationVector(const gp_Trsf& T)
    {
        gp_XYZ P = T.TranslationPart();
        return { { P.X(), P.Y(), P.Z() } };
    }

    // ==============================
    // MatTranspose
    // 功能：3x3矩阵转置
    // 用途：R03^-1 = R03^T（旋转矩阵正交）
    // ==============================
    Mat3 MatTranspose(const Mat3& A)
    {
        Mat3 R{};
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                R[i][j] = A[j][i];
        return R;
    }

    // ==============================
    // MatMul
    // 功能：3x3矩阵乘法
    // ==============================
    Mat3 MatMul(const Mat3& A, const Mat3& B)
    {
        Mat3 R{};
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                R[i][j] = 0.0;
                for (int k = 0; k < 3; ++k)
                {
                    R[i][j] += A[i][k] * B[k][j];
                }
            }
        }
        return R;
    }

    // ==============================
    // TranslationError
    // 功能：
    //   计算两个位姿之间的位置误差（欧氏距离）
    // 用途：
    //   判断某个逆解候选是否真的能到达目标位置
    // ==============================
    double TranslationError(const gp_Trsf& A, const gp_Trsf& B)
    {
        Vec3 a = GetTranslationVector(A);
        Vec3 b = GetTranslationVector(B);
        const double dx = a[0] - b[0];
        const double dy = a[1] - b[1];
        const double dz = a[2] - b[2];
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    // ==============================
    // RotationError
    // 功能：
    //   计算两个位姿之间的旋转误差
    //
    // 方法：
    //   逐项比较 3x3 旋转矩阵，取最大绝对误差
    //
    // 用途：
    //   判断逆解候选的姿态是否足够接近目标姿态
    // ==============================
    double RotationError(const gp_Trsf& A, const gp_Trsf& B)
    {
        Mat3 RA = GetRotationMatrix(A);
        Mat3 RB = GetRotationMatrix(B);
        double maxAbs = 0.0;

        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                const double e = std::fabs(RA[i][j] - RB[i][j]);
                if (e > maxAbs)
                    maxAbs = e;
            }
        }
        return maxAbs;
    }

    // ==============================
    // BuildR03
    // 功能：
    //   根据 q1 q2 q3 计算 R03
    //
    // 含义：
    //   R03 表示基座坐标系到第3轴末端坐标系的旋转矩阵
    //
    // 用途：
    //   在逆解中通过 R36 = R03^T * R06 分离腕部姿态
    // ==============================
    Mat3 BuildR03(double q1, double q2, double q3)
    {
        const double c1 = std::cos(q1);
        const double s1 = std::sin(q1);
        const double c23 = std::cos(q2 + q3);
        const double s23 = std::sin(q2 + q3);

        return { {
            {{  c1 * c23, -s1,  c1 * s23 }},
            {{  s1 * c23,  c1,  s1 * s23 }},
            {{     -s23,  0.0,      c23  }}
        } };
    }

    // ==============================
    // AddWristSolutions
    // 功能：
    //   根据 R36 计算腕部 J4 J5 J6 的解
    //
    // 输入：
    //   R36      : 第3轴到第6轴的旋转矩阵
    //   armPart  : 已经求好的前3轴解
    //   currentQ6: 当前J6角度，用于奇异位姿时保持连续
    //   out      : 输出所有腕部候选解
    //
    // 逻辑：
    //   1. 若非奇异，则正常分解出两组腕部解
    //   2. 若奇异（J5≈0 或 π），J4/J6 会耦合
    //      此时保留当前 J6，反推 J4
    // ==============================
    void AddWristSolutions(const Mat3& R36,
        const std::array<double, 6>& armPart,
        double currentQ6,
        std::vector<std::array<double, 6>>& out)
    {
        // s5 用于判断 J5 是否接近奇异位姿
        const double s5 = std::hypot(R36[0][1], R36[0][2]);

        // -------- 非奇异情况 --------
        if (s5 > kEps)
        {
            const double q5a = std::atan2(s5, R36[0][0]);
            const double q4a = std::atan2(R36[1][0], -R36[2][0]);
            const double q6a = std::atan2(R36[0][1], R36[0][2]);

            // 解A
            std::array<double, 6> solA = armPart;
            solA[3] = q4a;
            solA[4] = q5a;
            solA[5] = q6a;
            out.push_back(solA);

            // 解B：另一组对称解
            std::array<double, 6> solB = armPart;
            solB[3] = NormalizeRad(q4a + M_PI);
            solB[4] = -q5a;
            solB[5] = NormalizeRad(q6a + M_PI);
            out.push_back(solB);
            return;
        }

        // -------- 奇异情况 --------
        // X-Y-X 腕部奇异：q5 ≈ 0 或 π
        // 此时 q4/q6 不能独立唯一分解，只能保留一个并反推另一个
        const double combined = std::atan2(R36[2][1], R36[1][1]);

        std::array<double, 6> singular = armPart;

        if (R36[0][0] > 0.0)
        {
            // q5 ≈ 0，此时只剩 q4 + q6
            singular[4] = 0.0;
            singular[5] = currentQ6;
            singular[3] = combined - singular[5];
        }
        else
        {
            // q5 ≈ π，此时只剩 q4 - q6
            singular[4] = M_PI;
            singular[5] = currentQ6;
            singular[3] = combined + singular[5];
        }

        out.push_back(singular);
    }
}

// 现在安全恢复 MFC 的 DEBUG_NEW
#ifdef _DEBUG
#define new DEBUG_NEW
#endif

IMPLEMENT_DYNCREATE(COCCMFCDoc, CDocument)

BEGIN_MESSAGE_MAP(COCCMFCDoc, CDocument)
END_MESSAGE_MAP()

// ==============================
// 构造函数
// 功能：初始化文档对象
//
// 初始化内容：
//   1. 把各个机器人模型句柄置空
//   2. 把 6 个关节角初始化为 0
// ==============================
COCCMFCDoc::COCCMFCDoc() noexcept
{
    for (int i = 0; i < 7; i++)
        m_RobotLinks[i].Nullify();

    m_ImportedModel.Nullify();

    for (int i = 0; i < 6; i++)
        m_JointAngles[i] = 0.0;

    // 导入模型初始位置为机器人底座原点（0, 0, 0）
    for (int i = 0; i < 3; i++)
        m_ImportPosition[i] = 0.0;
}

// ==============================
// 析构函数
// 功能：释放文档对象
// 目前没有额外资源需要手工释放
// ==============================
COCCMFCDoc::~COCCMFCDoc()
{
}

// ==============================
// LoadRobotModel
// 功能：加载机器人各个连杆 STL 模型
//
// 步骤：
//   1. 获取当前程序路径
//   2. 拼出机器人模型目录
//   3. 逐个读取 LINK0 ~ LINK6 的 STL 文件
//   4. 把三角网格包装成 AIS_Shape
//   5. 设置材质和颜色
//
// 说明：
//   m_RobotLinks[i] 对应第 i 个机器人部件
// ==============================
void COCCMFCDoc::LoadRobotModel()
{
    TCHAR szPath[MAX_PATH];
    GetModuleFileName(NULL, szPath, MAX_PATH);
    CString strPath(szPath);

    // 回退 3 层目录，定位到工程相关路径
    for (int i = 0; i < 3; i++) {
        int pos = strPath.ReverseFind(_T('\\'));
        if (pos != -1) strPath = strPath.Left(pos);
    }

    CString modelFolder = strPath + _T("\\OCC_MFC\\Rbt3DModelLib\\Fanuc\\M10iD12\\");

    for (int i = 0; i <= 6; i++)
    {
        CString fileName;
        fileName.Format(_T("M10iD12_LINK%d.STL"), i);
        CString fullPath = modelFolder + fileName;

        // 文件不存在则跳过
        if (GetFileAttributes(fullPath) == INVALID_FILE_ATTRIBUTES)
            continue;

        Handle(Poly_Triangulation) aMesh = RWStl::ReadFile(Standard_CString(CT2A(fullPath)));

        if (!aMesh.IsNull())
        {
            // 把 STL 网格转换成可显示的面
            TopoDS_Face aFace;
            BRep_Builder B;
            B.MakeFace(aFace);
            B.UpdateFace(aFace, aMesh);
            BRepMesh_IncrementalMesh(aFace, 0.5);

            // 包装成 AIS_Shape，供 OCC 显示
            m_RobotLinks[i] = Handle(AIS_Shape)(::new AIS_Shape(aFace));
            m_RobotLinks[i]->SetMaterial(Graphic3d_NOM_SATIN);

            // 给末端工具单独设置灰色，其余关节黄色
            if (i == 6) {
                m_RobotLinks[i]->SetColor(Quantity_NOC_GRAY80);
            }
            else {
                m_RobotLinks[i]->SetColor(Quantity_NOC_YELLOW);
            }
        }
    }
}

void COCCMFCDoc::LoadModelFromFile(const CString& filePath)
{
    CString ext = filePath.Mid(filePath.ReverseFind(_T('.')) + 1);
    ext.MakeLower();

    TopoDS_Shape shape;

    if (ext == _T("stl"))
    {
        CStringA stlPath = ToOcctFilePath(filePath); // 转为 UTF-8，保证中文路径可被 OCCT 打开
        Handle(Poly_Triangulation) mesh = RWStl::ReadFile(Standard_CString(stlPath));
        if (mesh.IsNull())
        {
            AfxMessageBox(_T("STL 文件读取失败。"));
            return;
        }

        TopoDS_Face face;
        BRep_Builder builder;
        builder.MakeFace(face);
        builder.UpdateFace(face, mesh);
        BRepMesh_IncrementalMesh(face, 0.5);
        shape = face;
    }
    else if (ext == _T("step") || ext == _T("stp"))
    {
        // 初始化 OCCT STEP/IGES 资源变量，TransferRoot/TransferRoots 依赖这些默认参数文件
        if (!InitOcctResourcePaths())
        {
            AfxMessageBox(_T("未找到 OCCT STEP/IGES 资源目录，无法导入 STEP 文件。"));
            return;
        }

        STEPControl_Reader reader;
        CStringA stepPath = ToOcctFilePath(filePath); // 转为 UTF-8，保证中文路径可被 OCCT 打开
        IFSelect_ReturnStatus status = reader.ReadFile(Standard_CString(stepPath));
        if (status != IFSelect_RetDone)
        {
            CString msg;
            msg.Format(_T("STEP 文件读取失败(状态码: %d，%s)"), (int)status, StepStatusText(status).GetString());
            AfxMessageBox(msg);
            return;
        }
        // 使用 NbRootsForTransfer + TransferRoot(i) 逐个传输（比 TransferRoots() 更稳健）
        Standard_Integer nbRoots = reader.NbRootsForTransfer(); // STEP 顶层实体数量
        Standard_Integer transferredRoots = 0;                  // 成功传输的实体数量
        for (Standard_Integer ri = 1; ri <= nbRoots; ri++)
        {
            // TransferRoot：把 STEP 顶层实体转换为 OCCT TopoDS_Shape
            if (reader.TransferRoot(ri))
                transferredRoots++;
        }
        if (transferredRoots <= 0 && reader.TransferRoots() <= 0)
        {
            AfxMessageBox(_T("STEP 文件中未找到可传输的几何实体。"));
            return;
        }
        shape = reader.OneShape();
    }
    else if (ext == _T("iges") || ext == _T("igs"))
    {
        // 初始化 OCCT IGES 资源变量，保证 IGES 默认参数文件可被加载
        if (!InitOcctResourcePaths())
        {
            AfxMessageBox(_T("未找到 OCCT STEP/IGES 资源目录，无法导入 IGES 文件。"));
            return;
        }

        IGESControl_Reader reader;
        CStringA igesPath = ToOcctFilePath(filePath); // 转为 UTF-8，保证中文路径可被 OCCT 打开
        IFSelect_ReturnStatus status = reader.ReadFile(Standard_CString(igesPath));
        if (status != IFSelect_RetDone)
        {
            AfxMessageBox(_T("IGES 文件读取失败。"));
            return;
        }
        // IGESControl_Reader 不支持 TransferRoot(i)，使用 TransferRoots() 批量传输
        if (reader.TransferRoots() <= 0)
        {
            AfxMessageBox(_T("IGES 文件中未找到可传输的几何实体。"));
            return;
        }
        shape = reader.OneShape();
    }
    else
    {
        AfxMessageBox(_T("暂不支持该模型格式。"));
        return;
    }

    if (shape.IsNull())
    {
        AfxMessageBox(_T("模型内容为空，无法导入。"));
        return;
    }

    m_ImportedModel = Handle(AIS_Shape)(::new AIS_Shape(shape));
    m_ImportedModel->SetMaterial(Graphic3d_NOM_SATIN);
    m_ImportedModel->SetColor(Quantity_NOC_CYAN1);

    // ===== 应用导入模型比例和当前位置到模型 =====
    {
        // MakeImportedModelTransform：同时处理模型本体 mm->m 缩放和位置 mm->m 平移
        gp_Trsf T = MakeImportedModelTransform(m_ImportPosition);
        m_ImportedModel->SetLocalTransformation(T);
    }

    UpdateAllViews(NULL, 1);
}

// ==============================
// SetImportedModelPosition
// 功能：设置导入模型在 3D 场景中的摆放位置
//
// 输入参数：
//   x, y, z — 以机器人底座为原点的坐标值（单位：mm）
//
// 实现要点：
//   1. 保存坐标到 m_ImportPosition[3] 供后续使用
//   2. 将 mm 转为 m（OCCT 显示单位），构造 gp_Trsf 平移变换
//   3. 调用 AIS_Shape::SetLocalTransformation 应用到模型
void COCCMFCDoc::SetImportedModelPosition(double x, double y, double z)
{
    m_ImportPosition[0] = x;
    m_ImportPosition[1] = y;
    m_ImportPosition[2] = z;

    if (!m_ImportedModel.IsNull())
    {
        // MakeImportedModelTransform：保持导入模型本体缩放，同时更新其舞台位置
        gp_Trsf T = MakeImportedModelTransform(m_ImportPosition);
        m_ImportedModel->SetLocalTransformation(T);

        // 通知视图刷新
        UpdateAllViews(NULL);
        POSITION pos = GetFirstViewPosition();
        while (pos != NULL)
        {
            CView* pView = GetNextView(pos);
            if (pView != NULL)
            {
                pView->RedrawWindow(NULL, NULL, RDW_INVALIDATE | RDW_UPDATENOW | RDW_ALLCHILDREN);
            }
        }
    }
}

// ==============================
// OnNewDocument
// 功能：MFC 新建文档时调用
// 这里直接沿用基类逻辑
// ==============================
BOOL COCCMFCDoc::OnNewDocument()
{
    return CDocument::OnNewDocument();
}

// ==============================
// UpdateRobotPose
// 功能：根据当前 m_JointAngles 更新机器人三维模型姿态
//
// 主要步骤：
//   1. 定义各关节旋转中心
//   2. 从底座开始逐级累乘关节变换
//   3. 把累计变换设置到每个连杆模型
//   4. 更新 TCP 数值显示
//   5. 通知视图刷新重绘
//
// 说明：
//   这里主要服务于 3D 模型显示，单位使用米（和 OCC 模型一致）
// ==============================
void COCCMFCDoc::UpdateRobotPose()
{
    if (m_RobotLinks[0].IsNull()) return;

    // OCC 模型显示通常按米处理，这里把 mm 转 m
    const double s = 0.001;
    const double d0 = 450.0 * s;
    const double a1m = 75.0 * s;
    const double a2m = 640.0 * s;
    const double a3m = 195.0 * s;
    const double d3m = 700.0 * s;
    const double d5m = 75.0 * s;

    // 各关节旋转中心
    gp_Pnt p0(0, 0, 0);                            // J1
    gp_Pnt p1(a1m, 0, d0);                         // J2
    gp_Pnt p2(a1m, 0, d0 + a2m);                   // J3
    gp_Pnt p3(a1m, 0, d0 + a2m + a3m);             // J4
    gp_Pnt p4(a1m + d3m, 0, d0 + a2m + a3m);       // J5
    gp_Pnt p5(a1m + d3m + d5m, 0, d0 + a2m + a3m); // J6

    gp_Trsf T_accum;
    m_RobotLinks[0]->SetLocalTransformation(T_accum);

    // J1
    gp_Trsf T1, T2, T3;
    T1.SetRotation(gp_Ax1(p0, gp_Dir(0, 0, 1)), m_JointAngles[0] * kDegToRad);
    T_accum.Multiply(T1);
    if (!m_RobotLinks[1].IsNull()) m_RobotLinks[1]->SetLocalTransformation(T_accum);

    // J2
    T2.SetRotation(gp_Ax1(p1, gp_Dir(0, 1, 0)), m_JointAngles[1] * kDegToRad);
    T_accum.Multiply(T2);
    if (!m_RobotLinks[2].IsNull()) m_RobotLinks[2]->SetLocalTransformation(T_accum);

    // J3
    T3.SetRotation(gp_Ax1(p2, gp_Dir(0, 1, 0)), m_JointAngles[2] * kDegToRad);
    T_accum.Multiply(T3);
    if (!m_RobotLinks[3].IsNull()) m_RobotLinks[3]->SetLocalTransformation(T_accum);

    // J4
    gp_Trsf T4;
    T4.SetRotation(gp_Ax1(p3, gp_Dir(1, 0, 0)), m_JointAngles[3] * kDegToRad);
    T_accum.Multiply(T4);
    if (!m_RobotLinks[4].IsNull()) m_RobotLinks[4]->SetLocalTransformation(T_accum);

    // J5
    gp_Trsf T5;
    T5.SetRotation(gp_Ax1(p4, gp_Dir(0, 1, 0)), m_JointAngles[4] * kDegToRad);
    T_accum.Multiply(T5);
    if (!m_RobotLinks[5].IsNull()) m_RobotLinks[5]->SetLocalTransformation(T_accum);

    // J6
    gp_Trsf T6;
    T6.SetRotation(gp_Ax1(p5, gp_Dir(1, 0, 0)), m_JointAngles[5] * kDegToRad);
    T_accum.Multiply(T6);
    if (!m_RobotLinks[6].IsNull()) m_RobotLinks[6]->SetLocalTransformation(T_accum);

    // 根据当前关节角刷新 TCP 文本框显示
    UpdateTCPNumericalDisplay();

    // 通知所有视图刷新
    UpdateAllViews(NULL);
    POSITION pos = GetFirstViewPosition();
    while (pos != NULL)
    {
        CView* pView = GetNextView(pos);
        if (pView != NULL)
        {
            pView->RedrawWindow(NULL, NULL, RDW_INVALIDATE | RDW_UPDATENOW | RDW_ALLCHILDREN);
        }
    }
}

// ==============================
// UpdateTCPNumericalDisplay
// 功能：
//   根据当前关节角，计算末端 TCP 的 XYZWPR，回填到界面
//
// 步骤：
//   1. 用关节角做正解，得到法兰位姿
//   2. 乘上 LCS 修正矩阵
//   3. 乘上 TCP 标定矩阵
//   4. 提取平移 XYZ
//   5. 提取姿态四元数并转成 WPR
//   6. 回填到控制面板
// ==============================
void COCCMFCDoc::UpdateTCPNumericalDisplay()
{
    // 1. 正解：关节角 -> 法兰位姿
    gp_Trsf T_Flange = BuildFlangeTransformFromDegrees(m_JointAngles);

    // 2. 加上显示坐标系修正 + TCP 标定
    gp_Trsf T_Final = T_Flange;
    T_Final.Multiply(BuildLcsTransform());
    T_Final.Multiply(BuildTcpCalibration());

    // 3. 提取 XYZ
    gp_Pnt pos = T_Final.TranslationPart();

    // 4. 提取姿态并转成欧拉角
    gp_Quaternion quat = T_Final.GetRotation();
    Standard_Real angW = 0.0, angP = 0.0, angR = 0.0;
    quat.GetEulerAngles(gp_Extrinsic_ZYX, angR, angP, angW);

    // 5. 把 XYZWPR 更新到控制面板
    CMainFrame* pMainFrame = (CMainFrame*)AfxGetMainWnd();
    if (pMainFrame)
    {
        CRobotControlDlg& robotDlg = pMainFrame->m_wndControlPane.m_wndDlg;
        if (::IsWindow(robotDlg.GetSafeHwnd()))
        {
            robotDlg.UpdateTCPDisplay(
                pos.X(), pos.Y(), pos.Z(),
                angW * kRadToDeg, angP * kRadToDeg, angR * kRadToDeg
            );
        }
    }
}

// ==============================
// OnHandleTCPInput
// 功能：
//   当用户直接输入 TCP 的 XYZWPR 时，执行逆解
//
// 步骤：
//   1. 根据用户输入构造目标 TCP 位姿
//   2. 把显示链里的 LCS + TCP 标定逆掉
//      得到真正的法兰目标位姿
//   3. 调用六轴逆解
//   4. 若求解成功，则更新机器人姿态和界面关节显示
// ==============================
void COCCMFCDoc::OnHandleTCPInput(double x, double y, double z, double w, double p, double r)
{
    // 1. 构造用户输入的目标 TCP 位姿
    gp_Trsf T_TargetTcp;
    gp_Quaternion quat;
    quat.SetEulerAngles(gp_Extrinsic_ZYX, r * kDegToRad, p * kDegToRad, w * kDegToRad);
    T_TargetTcp.SetRotation(quat);
    T_TargetTcp.SetTranslationPart(gp_Vec(x, y, z));

    // 2. 逆掉显示链里的 LCS + TCP 标定，转成法兰目标位姿
    gp_Trsf T_LcsTcp = BuildLcsTransform();
    T_LcsTcp.Multiply(BuildTcpCalibration());

    gp_Trsf T_TargetFlange = T_TargetTcp;
    T_TargetFlange.Multiply(T_LcsTcp.Inverted());

    // 3. 做六轴逆解
    if (ComputeInverseKinematics(T_TargetFlange))
    {
        // 4. 更新机器人模型与TCP显示
        UpdateRobotPose();

        // 5. 更新关节编辑框/滑块显示
        CMainFrame* pMainFrame = (CMainFrame*)AfxGetMainWnd();
        if (pMainFrame)
        {
            pMainFrame->m_wndControlPane.m_wndDlg.UpdateJointDisplay();
        }
    }
}

// ==============================
// ComputeInverseKinematics
// 功能：
//   根据目标法兰位姿，求解 6 个关节角
//
// 输入：
//   T_TargetFlange = 目标法兰位姿
//
// 输出：
//   若求解成功，则把结果写入 m_JointAngles，并返回 true
//   若失败，则返回 false
//
// 总体思路：
//   1. 从目标法兰位姿中扣掉 d6，算出腕点位置
//   2. 用几何法求 J1/J2/J3
//   3. 用 R36 = R03^T * R06 求 J4/J5/J6
//   4. 验证每组候选解的正解误差
//   5. 选取离当前姿态最近的一组解
// ==============================
bool COCCMFCDoc::ComputeInverseKinematics(const gp_Trsf& T_TargetFlange)
{
    // 1. 提取目标法兰位置和姿态
    const Vec3 pTarget = GetTranslationVector(T_TargetFlange);
    const Mat3 R06 = GetRotationMatrix(T_TargetFlange);

    // 2. 根据工具X方向扣除 d6，得到腕点位置
    //    腕点 = 法兰位置 - d6 * 工具X方向
    Vec3 xTool = { { R06[0][0], R06[1][0], R06[2][0] } };
    Vec3 pWc = { {
        pTarget[0] - d6 * xTool[0],
        pTarget[1] - d6 * xTool[1],
        pTarget[2] - d6 * xTool[2]
    } };

    // 3. 当前姿态，用于后面选“最接近当前姿态”的解
    const double currentQ6 = m_JointAngles[5] * kDegToRad;
    const double currentRad[6] = {
        m_JointAngles[0] * kDegToRad,
        m_JointAngles[1] * kDegToRad,
        m_JointAngles[2] * kDegToRad,
        m_JointAngles[3] * kDegToRad,
        m_JointAngles[4] * kDegToRad,
        m_JointAngles[5] * kDegToRad
    };

    // 4. 把 J2/J3 所在平面问题转成 2R 几何问题
    const double l2 = a2;
    const double l3 = std::hypot(d4, a3);
    const double alpha = std::atan2(a3, d4);

    std::vector<std::array<double, 6>> rawSolutions;

    // 5. 求 J1：通常有两种基座方向可能
    const double q1Base = std::atan2(pWc[1], pWc[0]);
    const double q1Candidates[2] = { q1Base, NormalizeRad(q1Base + M_PI) };

    for (double q1 : q1Candidates)
    {
        // 6. 把腕点投影到 J2/J3 所在平面
        const double rhoSigned = std::cos(q1) * pWc[0] + std::sin(q1) * pWc[1];
        const double rPlanar = rhoSigned - a1;
        const double hPlanar = pWc[2] - d1;

        // 7. 用余弦定理求肘部角 psi
        double cPsi = (rPlanar * rPlanar + hPlanar * hPlanar - l2 * l2 - l3 * l3) / (2.0 * l2 * l3);
        if (cPsi < -1.01 || cPsi > 1.01)
            continue;

        cPsi = Clamp(cPsi, -1.0, 1.0);
        const double sPsiAbs = std::sqrt((std::max)(0.0, 1.0 - cPsi * cPsi));

        // 8. 肘上 / 肘下 两种解
        for (int elbowSign : { -1, 1 })
        {
            const double psi = std::atan2(elbowSign * sPsiAbs, cPsi);

            // 连杆方向角
            const double thetaLine = std::atan2(hPlanar, rPlanar)
                - std::atan2(l3 * std::sin(psi), l2 + l3 * std::cos(psi));

            // 9. 反推出 J2 / J3
            const double q2 = M_PI / 2.0 - thetaLine;
            const double q3 = alpha - M_PI / 2.0 - psi;

            // 10. 前三轴确定后，分离腕部姿态
            std::array<double, 6> armPart = { q1, q2, q3, 0.0, 0.0, 0.0 };
            const Mat3 R03 = BuildR03(q1, q2, q3);
            const Mat3 R36 = MatMul(MatTranspose(R03), R06);

            // 11. 根据 R36 求 J4/J5/J6
            AddWristSolutions(R36, armPart, currentQ6, rawSolutions);
        }
    }

    // 没有任何可行解
    if (rawSolutions.empty())
        return false;

    IkCandidate best;

    // 12. 遍历所有候选解，做正解验证并选最优
    for (const auto& qRaw : rawSolutions)
    {
        IkCandidate cand;
        cand.qRad = qRaw;

        // 把各轴角度归一化
        for (int i = 0; i < 6; ++i)
        {
            cand.qRad[i] = NormalizeRad(cand.qRad[i]);
        }

        // 做一次正解，看这个候选是不是和目标位姿一致
        gp_Trsf T_Test = BuildFlangeTransformFromRadians(cand.qRad);
        cand.posErr = TranslationError(T_Test, T_TargetFlange);
        cand.rotErr = RotationError(T_Test, T_TargetFlange);

        // 误差太大，认为这组解无效
        if (cand.posErr > 1.0e-3 || cand.rotErr > 1.0e-6)
            continue;

        // 计算这组解相对当前姿态的变化量
        // 变化越小，动作越平滑
        cand.cost = 0.0;
        for (int i = 0; i < 6; ++i)
        {
            const double dq = NormalizeRad(cand.qRad[i] - currentRad[i]);
            cand.cost += dq * dq;
        }

        // 保留最优解
        if (cand.cost < best.cost)
            best = cand;
    }

    // 如果所有候选都被误差筛掉了，则求解失败
    if (best.cost == (std::numeric_limits<double>::max)())
        return false;

    // 13. 把最优解写回关节角数组（角度制）
    for (int i = 0; i < 6; ++i)
    {
        m_JointAngles[i] = NormalizeDeg(best.qRad[i] * kRadToDeg);
    }

    return true;
}

// ==============================
// Serialize
// 功能：MFC 文档序列化
// 目前未做数据存取
// ==============================
void COCCMFCDoc::Serialize(CArchive& ar)
{
    if (ar.IsStoring())
    {
        // 如需保存数据，可在这里添加
    }
    else
    {
        // 如需加载数据，可在这里添加
    }
}

#ifdef _DEBUG

// ==============================
// AssertValid
// 功能：调试模式下检查对象有效性
// ==============================
void COCCMFCDoc::AssertValid() const
{
    CDocument::AssertValid();
}

// ==============================
// Dump
// 功能：调试模式下输出对象调试信息
// ==============================
void COCCMFCDoc::Dump(CDumpContext& dc) const
{
    CDocument::Dump(dc);
}
#endif

// ============================================================================
// 基于专利 CN 120339259 A 的焊缝提取算法 - WeldExtractor 完整实现
// 替换旧的 FindEdge2 实现
// 5步法:(1)拓扑映射 (2)初始焊缝提取 (3)截断检测 (4)内部截面 (5)去重输出
// ============================================================================

// ==============================
// 辅助工具函数
// ==============================

// 计算边的长度
static double WeldEdgeLen(const TopoDS_Edge& edge)
{
    try { return GCPnts_AbscissaPoint::Length(BRepAdaptor_Curve(edge)); }
    catch (...) { return 0.0; }
}

// 取边的起点、中点、终点
static bool GetEdgePts(const TopoDS_Edge& edge, gp_Pnt& p0, gp_Pnt& pm, gp_Pnt& p1)
{
    Standard_Real f = 0.0, l = 0.0;
    Handle(Geom_Curve) c = BRep_Tool::Curve(edge, f, l);
    if (c.IsNull()) return false;
    p0 = c->Value(f);
    pm = c->Value((f + l) * 0.5);
    p1 = c->Value(l);
    return true;
}

// 计算两端点距离
static double PointDist(const gp_Pnt& a, const gp_Pnt& b)
{
    return a.Distance(b);
}

// 焊缝长度排序(长到短)
static bool SortWeldByLength(const WeldSeam& a, const WeldSeam& b)
{
    return a.Length > b.Length;
}

// ==============================
// WeldExtractor 构造/析构
// ==============================
WeldExtractor::WeldExtractor() {}
WeldExtractor::~WeldExtractor() {}

// ==============================
// Compute - 主入口: 执行完整5步焊缝提取
// ==============================
void WeldExtractor::Compute()
{
    if (inputShape.IsNull()) return;
    WeldSeams.clear();
    m_Solids.clear();
    m_SolidFaceEdgeMaps.clear();

    // ===== 步骤1: 模型读取与映射 =====
    BuildTopologyMaps();
    if (m_Solids.size() < 2) return;  // 至少需要2个子模型

    // ===== 步骤2: 初始焊缝提取 =====
    ExtractInitialWelds();

    if (WeldSeams.empty()) return;

    // ===== 步骤3: 焊缝截断检测 =====
    DetectTruncations();

    // ===== 步骤4: 内部截面检测 =====
    DetectInternalSections();

    // ===== 步骤5: 去重输出 =====
    RemoveDuplicateSeams();
}

// ==============================
// BuildTopologyMaps - 步骤1: 提取子模型并建立面到边的映射
// ==============================
void WeldExtractor::BuildTopologyMaps()
{
    m_Solids.clear();
    m_SolidFaceEdgeMaps.clear();

    // 提取所有子模型(Solid)
    for (TopExp_Explorer x(inputShape, TopAbs_SOLID); x.More(); x.Next())
    {
        TopoDS_Solid solid = TopoDS::Solid(x.Current());
        m_Solids.push_back(solid);

        // 建立该子模型的面到边映射
        TopTools_IndexedDataMapOfShapeListOfShape faceEdgeMap;
        TopExp::MapShapesAndAncestors(solid, TopAbs_EDGE, TopAbs_FACE, faceEdgeMap);
        m_SolidFaceEdgeMaps.push_back(faceEdgeMap);
    }
}

// ==============================
// ComputeFaceNormal - 计算平面面的法向量
// ==============================
gp_Vec WeldExtractor::ComputeFaceNormal(const TopoDS_Face& face)
{
    Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
    if (surface.IsNull() || !surface->IsKind(STANDARD_TYPE(Geom_Plane)))
        return gp_Vec(0, 0, 0);

    gp_Dir dir = Handle(Geom_Plane)::DownCast(surface)->Pln().Axis().Direction();
    gp_Vec normal(dir.X(), dir.Y(), dir.Z());
    if (face.Orientation() == TopAbs_REVERSED)
        normal = -normal;
    return normal;
}

// ==============================
// IsFacePlanar - 判断是否为平面
// ==============================
bool WeldExtractor::IsFacePlanar(const TopoDS_Face& face)
{
    Handle(Geom_Surface) s = BRep_Tool::Surface(face);
    return (!s.IsNull() && s->IsKind(STANDARD_TYPE(Geom_Plane)));
}

// ==============================
// HasOppositeNormals - 判断两个面是否法线相反
// 专利步骤2: 通过OBB相交检测+法线方向筛选
// ==============================
bool WeldExtractor::HasOppositeNormals(const TopoDS_Face& face1, const TopoDS_Face& face2)
{
    // 只处理平面面
    if (!IsFacePlanar(face1) || !IsFacePlanar(face2))
        return false;

    // OBB包围盒相交性检测
    Bnd_Box box1, box2;
    BRepBndLib::Add(face1, box1);
    BRepBndLib::Add(face2, box2);
    if (box1.IsOut(box2))
        return false;

    // 计算法线
    gp_Vec n1 = ComputeFaceNormal(face1);
    gp_Vec n2 = ComputeFaceNormal(face2);
    if (n1.Magnitude() < Precision::Confusion() || n2.Magnitude() < Precision::Confusion())
        return false;

    n1.Normalize();
    n2.Normalize();

    // 法线方向相反(夹角接近180度)
    double dot = n1.Dot(n2);
    return (dot < -0.95);  // 夹角约162度到180度视为反向
}

// ==============================
// GetFaceParamRange - 获取面在投影平面上的参数范围
// 专利步骤2: 通过 OuterWire 提取外轮廓, 构造投影面, 计算参数范围
// ==============================
void WeldExtractor::GetFaceParamRange(const TopoDS_Face& face, double& uMin, double& uMax,
                                       double& vMin, double& vMax)
{
    uMin = uMax = vMin = vMax = 0.0;

    // 获取面的外轮廓
    TopoDS_Wire outerWire = BRepTools::OuterWire(face);
    if (outerWire.IsNull()) return;

    // 获取面的几何曲面
    Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
    if (surface.IsNull()) return;

    // 遍历外轮廓上的所有边, 投影到曲面上的参数空间
    bool first = true;
    for (TopExp_Explorer edgeExp(outerWire, TopAbs_EDGE); edgeExp.More(); edgeExp.Next())
    {
        TopoDS_Edge edge = TopoDS::Edge(edgeExp.Current());
        Standard_Real f = 0.0, l = 0.0;
        Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, f, l);
        if (curve.IsNull()) continue;

        // 采样边的多个点并投影到曲面参数空间
        int numSamples = 10;
        for (int i = 0; i <= numSamples; i++)
        {
            double param = f + (l - f) * i / numSamples;
            gp_Pnt pt = curve->Value(param);
            Standard_Real u = 0.0, v = 0.0;
            GeomAPI_ProjectPointOnSurf projector(pt, surface);
            if (projector.NbPoints() > 0)
            {
                projector.LowerDistanceParameters(u, v);
                if (first)
                {
                    uMin = uMax = u;
                    vMin = vMax = v;
                    first = false;
                }
                else
                {
                    uMin = std::min(uMin, u);
                    uMax = std::max(uMax, u);
                    vMin = std::min(vMin, v);
                    vMax = std::max(vMax, v);
                }
            }
        }
    }
}

// ==============================
// IsContainmentRelation - 判断两个面的包含关系
// 专利步骤2: 通过参数范围判断一个面是否被另一个面包含
// ==============================
bool WeldExtractor::IsContainmentRelation(const TopoDS_Face& face1, const TopoDS_Face& face2)
{
    double uMin1, uMax1, vMin1, vMax1;
    double uMin2, uMax2, vMin2, vMax2;

    GetFaceParamRange(face1, uMin1, uMax1, vMin1, vMax1);
    GetFaceParamRange(face2, uMin2, uMax2, vMin2, vMax2);

    // 判断 face1 是否被 face2 包含
    // 使用专利中的公式: uMin2-1 <= uMin1, uMax2+1 >= uMax1 ...
    const double tolerance = 1.0;
    bool contained1 = (uMin2 - tolerance <= uMin1) &&
                      (uMax2 + tolerance >= uMax1) &&
                      (vMin2 - tolerance <= vMin1) &&
                      (vMax2 + tolerance >= vMax1);

    // 判断 face2 是否被 face1 包含
    bool contained2 = (uMin1 - tolerance <= uMin2) &&
                      (uMax1 + tolerance >= uMax2) &&
                      (vMin1 - tolerance <= vMin2) &&
                      (vMax1 + tolerance >= vMax2);

    return contained1 || contained2;
}

// ==============================
// GetFaceArea - 计算面的面积
// ==============================
static double GetFaceArea(const TopoDS_Face& face)
{
    GProp_GProps props;
    BRepGProp::SurfaceProperties(face, props);
    return props.Mass();
}

// ==============================
// GetFaceOuterEdges - 获取面的外轮廓边
// ==============================
static std::vector<TopoDS_Edge> GetFaceOuterEdges(const TopoDS_Face& face)
{
    std::vector<TopoDS_Edge> edges;
    TopoDS_Wire outerWire = BRepTools::OuterWire(face);
    if (outerWire.IsNull()) return edges;
    for (TopExp_Explorer exp(outerWire, TopAbs_EDGE); exp.More(); exp.Next())
        edges.push_back(TopoDS::Edge(exp.Current()));
    return edges;
}

// ==============================
// FindAdjacentFormingFaces - 查找与接触面相接的面作为形成面
// 专利: 将相接且与焊缝存在共线的面作为焊缝的形成面
// ==============================
static void FindAdjacentFormingFaces(const TopoDS_Face& contactFace, const TopoDS_Solid& solid,
                                      TopoDS_Face& formingFace)
{
    // 获取接触面的边, 然后找该实体中共享这些边的其他面
    TopTools_IndexedDataMapOfShapeListOfShape edgeFaceMap;
    TopExp::MapShapesAndAncestors(solid, TopAbs_EDGE, TopAbs_FACE, edgeFaceMap);

    for (TopExp_Explorer edgeExp(contactFace, TopAbs_EDGE); edgeExp.More(); edgeExp.Next())
    {
        const TopoDS_Shape& edge = edgeExp.Current();
        const TopTools_ListOfShape& faces = edgeFaceMap.FindFromKey(edge);
        for (TopTools_ListIteratorOfListOfShape it(faces); it.More(); it.Next())
        {
            const TopoDS_Face& candidateFace = TopoDS::Face(it.Value());
            if (!candidateFace.IsSame(contactFace))
            {
                // 找到第一个与接触面共享边但不是接触面本身的面
                formingFace = candidateFace;
                return;
            }
        }
    }
}

// ==============================
// EdgeToWeldSeam - 将边转换为焊缝数据结构
// ==============================
static bool EdgeToWeldSeam(const TopoDS_Edge& edge, WeldSeam& seam)
{
    seam = WeldSeam();

    Standard_Real f = 0.0, l = 0.0;
    Handle(Geom_Curve) c = BRep_Tool::Curve(edge, f, l);
    if (c.IsNull()) return false;

    seam.Length = WeldEdgeLen(edge);
    if (seam.Length < 1.0) return false;

    seam.CurveType = c->DynamicType()->Name();

    // 起点/终点
    TopoDS_Vertex v1, v2;
    TopExp::Vertices(edge, v1, v2);
    if (!v1.IsNull())
    {
        gp_Pnt p = BRep_Tool::Pnt(v1);
        seam.StartPoint.X = p.X();
        seam.StartPoint.Y = p.Y();
        seam.StartPoint.Z = p.Z();
    }
    if (!v2.IsNull())
    {
        gp_Pnt p = BRep_Tool::Pnt(v2);
        seam.EndPoint.X = p.X();
        seam.EndPoint.Y = p.Y();
        seam.EndPoint.Z = p.Z();
    }

    // 圆形焊缝处理
    if (c->DynamicType() == STANDARD_TYPE(Geom_Circle))
    {
        Handle(Geom_Circle) ci = Handle(Geom_Circle)::DownCast(c);
        seam.CenterX = ci->Location().X();
        seam.CenterY = ci->Location().Y();
        seam.CenterZ = ci->Location().Z();
        seam.Radius = ci->Radius();
    }

    seam.EdgeGeom = edge;
    return true;
}

// ==============================
// ExtractWeldFromContainment - 包含关系: 取较小面的边作为焊缝
// 专利步骤2: 若两个面为包含关系, 取较小面作为交面, 将其边作为初始焊缝
// ==============================
// ==============================
// ExtractWeldFromContainment - 包含关系：取小面外轮廓边作为初始焊缝
// 适用场景：加强筋贴板、隔板嵌入等，一个面完全位于另一个面内
// 关键逻辑：取较小面的外轮廓边直接作为焊缝，绑定形成面
// ==============================
void WeldExtractor::ExtractWeldFromContainment(const TopoDS_Face& smallFace, const TopoDS_Face& bigFace,
                                                int shapeIdx1, int shapeIdx2)
{
    std::vector<TopoDS_Edge> edges = GetFaceOuterEdges(smallFace);

    for (const auto& edge : edges)
    {
        WeldSeam seam;
        if (!EdgeToWeldSeam(edge, seam))
            continue;

        seam.ShapeIdx1 = shapeIdx1;
        seam.ShapeIdx2 = shapeIdx2;
        seam.Id = (int)WeldSeams.size() + 1;

        // 绑定形成面(接触面两侧的面)
        BindFormingFaces(seam, smallFace, bigFace, shapeIdx1, shapeIdx2);

        WeldSeams.push_back(seam);
    }
}

// ==============================
// ExtractWeldFromIntersection - 相交关系: 将较小面投影至较大面生成轮廓
// 专利步骤2: 若两个面为相交关系, 将较小面投影至较大面, 生成轮廓作为初始焊缝
// ==============================
// ==============================
// ExtractWeldFromIntersection - 相交关系：将小面投影至大面取交线作为初始焊缝
// 适用场景：板-板搭接、管-板相贯等，两个面部分重叠
// 关键逻辑：使用 BRepAlgoAPI_Section 计算两面交线
// ==============================
void WeldExtractor::ExtractWeldFromIntersection(const TopoDS_Face& smallFace, const TopoDS_Face& bigFace,
                                                 int shapeIdx1, int shapeIdx2)
{
    // 使用 BRepAlgoAPI_Section 计算两个面的交线
    BRepAlgoAPI_Section section(smallFace, bigFace);
    section.ComputePCurveOn1(Standard_True);
    section.Approximation(Standard_True);
    section.Build();

    if (!section.IsDone())
        return;

    TopoDS_Shape result = section.Shape();

    // 提取交线中的边作为焊缝
    for (TopExp_Explorer exp(result, TopAbs_EDGE); exp.More(); exp.Next())
    {
        TopoDS_Edge edge = TopoDS::Edge(exp.Current());
        WeldSeam seam;
        if (!EdgeToWeldSeam(edge, seam))
            continue;

        seam.ShapeIdx1 = shapeIdx1;
        seam.ShapeIdx2 = shapeIdx2;
        seam.Id = (int)WeldSeams.size() + 1;

        // 绑定形成面
        BindFormingFaces(seam, smallFace, bigFace, shapeIdx1, shapeIdx2);

        WeldSeams.push_back(seam);
    }
}

// ==============================
// BindFormingFaces - 绑定焊缝的形成面
// 专利: 将接触面的相邻面(与焊缝共线的面)作为形成面
// ==============================
// ==============================
// BindFormingFaces - 绑定焊缝的形成面
// 形成面定义：与接触面共享边且非接触面本身的面，即焊缝两侧的实体面
// 关键逻辑：沿接触面的边在所属实体中查找相邻面
// ==============================
void WeldExtractor::BindFormingFaces(WeldSeam& seam, const TopoDS_Face& face1, const TopoDS_Face& face2,
                                      int shapeIdx1, int shapeIdx2)
{
    if (shapeIdx1 >= 0 && shapeIdx1 < (int)m_Solids.size())
    {
        TopoDS_Face forming1;
        FindAdjacentFormingFaces(face1, m_Solids[shapeIdx1], forming1);
        seam.FormingFace1 = forming1;
    }

    if (shapeIdx2 >= 0 && shapeIdx2 < (int)m_Solids.size())
    {
        TopoDS_Face forming2;
        FindAdjacentFormingFaces(face2, m_Solids[shapeIdx2], forming2);
        seam.FormingFace2 = forming2;
    }
}

// ==============================
// ExtractInitialWelds - 步骤2: 提取初始焊缝
// 遍历所有子模型对, 筛选相接且法线相反的面, 按包含/相交关系生成焊缝
// ==============================
// ==============================
// ExtractInitialWelds - 步骤2：提取初始焊缝（核心入口）
// 关键逻辑：遍历所有子模型对→OBB包围盒预筛→法线反向筛选→距离确认
//          →包含/相交关系分类→生成焊缝+绑定形成面
// ==============================
void WeldExtractor::ExtractInitialWelds()
{
    int n = (int)m_Solids.size();
    if (n < 2) return;

    // 预计算所有平面面列表
    struct FaceInfo {
        TopoDS_Face face;
        gp_Vec normal;
        int shapeIdx;
    };
    std::vector<std::vector<FaceInfo>> solidFaces(n);

    for (int si = 0; si < n; si++)
    {
        for (TopExp_Explorer fExp(m_Solids[si], TopAbs_FACE); fExp.More(); fExp.Next())
        {
            TopoDS_Face face = TopoDS::Face(fExp.Current());
            if (!IsFacePlanar(face)) continue;
            gp_Vec normal = ComputeFaceNormal(face);
            if (normal.Magnitude() < Precision::Confusion()) continue;
            normal.Normalize();
            solidFaces[si].push_back({ face, normal, si });
        }
    }

    // 遍历所有子模型对
    for (int i = 0; i < n; i++)
    {
        for (int j = i + 1; j < n; j++)
        {
            // OBB包围盒预筛 - 先检查两个实体的包围盒是否相交
            Bnd_Box boxI, boxJ;
            BRepBndLib::Add(m_Solids[i], boxI);
            BRepBndLib::Add(m_Solids[j], boxJ);
            if (boxI.IsOut(boxJ))
                continue;

            // 遍历两个实体的所有面, 找法线相反的对
            for (const auto& fi : solidFaces[i])
            {
                for (const auto& fj : solidFaces[j])
                {
                    // 法线方向相反
                    double dot = fi.normal.Dot(fj.normal);
                    if (dot >= -0.95) continue;

                    // OBB相交检测
                    Bnd_Box boxFi, boxFj;
                    BRepBndLib::Add(fi.face, boxFi);
                    BRepBndLib::Add(fj.face, boxFj);
                    if (boxFi.IsOut(boxFj))
                        continue;

                    // 距离检测: 确认两个面足够接近
                    BRepExtrema_DistShapeShape distTool(fi.face, fj.face);
                    distTool.Perform();
                    if (!distTool.IsDone() || distTool.Value() > 5.0)
                        continue;

                    // 判定包含/相交关系
                    double areaI = GetFaceArea(fi.face);
                    double areaJ = GetFaceArea(fj.face);

                    const TopoDS_Face& smallFace = (areaI <= areaJ) ? fi.face : fj.face;
                    const TopoDS_Face& bigFace = (areaI <= areaJ) ? fj.face : fi.face;
                    int smallIdx = (areaI <= areaJ) ? i : j;
                    int bigIdx = (areaI <= areaJ) ? j : i;

                    if (IsContainmentRelation(smallFace, bigFace))
                    {
                        // 包含关系: 取小面边
                        ExtractWeldFromContainment(smallFace, bigFace, smallIdx, bigIdx);
                    }
                    else
                    {
                        // 相交关系: 投影生成交线
                        ExtractWeldFromIntersection(smallFace, bigFace, smallIdx, bigIdx);
                    }
                }
            }
        }
    }
}

// ==============================
// CountIntersectionPoints - 计算边与实体的交点数量
// 专利步骤3: 使用 IntCurvesFace_ShapeIntersector 计算交点数目
// ==============================
// ==============================
// CountIntersectionPoints - 计算焊缝边与实体的交点数量
// 用于步骤3截断检测：0=无截断，1=单侧截断，2+=双侧截断
// 关键逻辑：使用 IntCurvesFace_ShapeIntersector 遍历实体所有面
// ==============================
int WeldExtractor::CountIntersectionPoints(const TopoDS_Edge& edge, const TopoDS_Solid& solid,
                                            std::vector<gp_Pnt>& outPts)
{
    outPts.clear();

    if (edge.IsNull() || solid.IsNull())
        return 0;

    // 获取边的曲线
    Standard_Real f = 0.0, l = 0.0;
    Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, f, l);
    if (curve.IsNull()) return 0;

    // 遍历实体的所有面, 计算边与面的交点
    for (TopExp_Explorer faceExp(solid, TopAbs_FACE); faceExp.More(); faceExp.Next())
    {
        TopoDS_Face face = TopoDS::Face(faceExp.Current());
        try
        {
            IntCurvesFace_ShapeIntersector intersector;
            intersector.Load(face, Precision::Confusion());
            Handle(GeomAdaptor_Curve) _gac = ::new GeomAdaptor_Curve(curve, f, l);
            intersector.Perform(_gac, f, l);

            for (int k = 1; k <= intersector.NbPnt(); k++)
            {
                gp_Pnt pt = intersector.Pnt(k);
                outPts.push_back(pt);
            }
        }
        catch (...)
        {
            continue;
        }
    }

    return (int)outPts.size();
}

// ==============================
// HandleSingleSideTruncation - 单侧截断处理
// 专利步骤3: 使用 BRepAlgoAPI_Cut 切除截断部分
// ==============================
// ==============================
// HandleSingleSideTruncation - 单侧截断：焊缝一端被另一实体截断
// 触发条件：CountIntersectionPoints 返回 1
// 关键逻辑：标记截断面计数为1，找到与焊缝相交的面作为截断面
// ==============================
void WeldExtractor::HandleSingleSideTruncation(WeldSeam& seam, const TopoDS_Solid& cuttingSolid)
{
    if (seam.SectionCount == 0)
    {
        seam.SectionCount = 1;

        // 找到与焊缝相交的面作为截断面
        Standard_Real f = 0.0, l = 0.0;
        Handle(Geom_Curve) curve = BRep_Tool::Curve(seam.EdgeGeom, f, l);
        if (curve.IsNull()) return;

        // 找最近的相交面
        for (TopExp_Explorer faceExp(cuttingSolid, TopAbs_FACE); faceExp.More(); faceExp.Next())
        {
            TopoDS_Face face = TopoDS::Face(faceExp.Current());
            try
            {
                IntCurvesFace_ShapeIntersector intersector;
                intersector.Load(face, Precision::Confusion());
                Handle(GeomAdaptor_Curve) _gac = ::new GeomAdaptor_Curve(curve, f, l);
                intersector.Perform(_gac, f, l);
                if (intersector.NbPnt() > 0)
                {
                    seam.TruncationFace1 = face;
                    break;
                }
            }
            catch (...)
            {
                continue;
            }
        }
    }
}

// ==============================
// HandleDoubleSideTruncation - 双侧截断处理
// 专利步骤3: 将焊缝切割为两段, 分别绑定最近的截断面
// ==============================
// ==============================
// HandleDoubleSideTruncation - 双侧截断：焊缝被另一实体从中间截断
// 触发条件：CountIntersectionPoints 返回 2+
// 关键逻辑：找到两个截断交点，分别靠近焊缝两端，各自绑定截断面
// ==============================
void WeldExtractor::HandleDoubleSideTruncation(WeldSeam& seam, const TopoDS_Solid& cuttingSolid)
{
    if (seam.SectionCount <= 1)
    {
        seam.SectionCount = 2;

        std::vector<gp_Pnt> allPts;
        CountIntersectionPoints(seam.EdgeGeom, cuttingSolid, allPts);

        gp_Pnt sp(seam.StartPoint.X, seam.StartPoint.Y, seam.StartPoint.Z);
        gp_Pnt ep(seam.EndPoint.X, seam.EndPoint.Y, seam.EndPoint.Z);

        double minDistS = 1e10;
        double minDistE = 1e10;

        Standard_Real f = 0.0, l = 0.0;
        Handle(Geom_Curve) curve = BRep_Tool::Curve(seam.EdgeGeom, f, l);
        if (curve.IsNull()) return;

        for (TopExp_Explorer faceExp(cuttingSolid, TopAbs_FACE); faceExp.More(); faceExp.Next())
        {
            TopoDS_Face face = TopoDS::Face(faceExp.Current());
            try
            {
                IntCurvesFace_ShapeIntersector intersector;
                intersector.Load(face, Precision::Confusion());
                Handle(GeomAdaptor_Curve) _gac = ::new GeomAdaptor_Curve(curve, f, l);
                intersector.Perform(_gac, f, l);

                for (int k = 1; k <= intersector.NbPnt(); k++)
                {
                    gp_Pnt pt = intersector.Pnt(k);
                    double dS = sp.Distance(pt);
                    double dE = ep.Distance(pt);

                    if (dS < minDistS) { minDistS = dS; seam.TruncationFace1 = face; }
                    if (dE < minDistE) { minDistE = dE; seam.TruncationFace2 = face; }
                }
            }
            catch (...)
            {
                continue;
            }
        }
    }
}

// ==============================
// DetectTruncations - 步骤3: 焊缝截断检测
// 遍历初始焊缝, 检测相交子模型, 根据截断面数量处理
// ==============================
// ==============================
// DetectTruncations - 步骤3：焊缝截断检测
// 关键逻辑：遍历每条初始焊缝，检测与其他子模型的相交情况
//          0交点=跳过，1交点=单侧截断，2+交点=双侧截断
// ==============================
void WeldExtractor::DetectTruncations()
{
    int n = (int)m_Solids.size();

    for (auto& seam : WeldSeams)
    {
        if (seam.EdgeGeom.IsNull())
            continue;

        // 遍历除形成面所属子模型外的其他子模型
        for (int si = 0; si < n; si++)
        {
            if (si == seam.ShapeIdx1 || si == seam.ShapeIdx2)
                continue;

            // OBB预筛
            Bnd_Box boxEdge, boxSolid;
            BRepBndLib::Add(seam.EdgeGeom, boxEdge);
            BRepBndLib::Add(m_Solids[si], boxSolid);
            if (boxEdge.IsOut(boxSolid))
                continue;

            // 计算交点
            std::vector<gp_Pnt> pts;
            int nPts = CountIntersectionPoints(seam.EdgeGeom, m_Solids[si], pts);

            if (nPts == 0)
                continue;
            else if (nPts == 1)
                HandleSingleSideTruncation(seam, m_Solids[si]);
            else if (nPts >= 2)
                HandleDoubleSideTruncation(seam, m_Solids[si]);
        }
    }
}

// ==============================
// CheckNormalConsistency - 检查法线方向一致性
// 专利步骤4: 计算焊缝端点与相邻面的法线方向一致性
// ==============================
bool WeldExtractor::CheckNormalConsistency(const gp_Pnt& fromPnt, const gp_Pnt& toPnt,
                                            const TopoDS_Face& face)
{
    // 构造从fromPnt到toPnt的向量
    gp_Vec dirVec(toPnt.X() - fromPnt.X(),
                  toPnt.Y() - fromPnt.Y(),
                  toPnt.Z() - fromPnt.Z());
    if (dirVec.Magnitude() < Precision::Confusion())
        return false;
    dirVec.Normalize();

    // 计算面的法向量
    gp_Vec normal = ComputeFaceNormal(face);
    if (normal.Magnitude() < Precision::Confusion())
        return false;
    normal.Normalize();

    // 判断方向是否相反(法线指向焊缝内部, 则向量应从起点到终点)
    double dot = dirVec.Dot(normal);
    return (dot < -0.5);
}

// ==============================
// DetectInternalSections - 步骤4: 内部截面检测
// 对截断完的焊缝进行自身实体内部的截面检测
// ==============================
// ==============================
// DetectInternalSections - 步骤4：内部截面检测
// 对不足2个截面的焊缝，在关联子模型内查找邻近平面
// 关键逻辑：焊缝端点指向焊缝内部的向量与面法线方向相反则为截断面
// ==============================
void WeldExtractor::DetectInternalSections()
{
    for (auto& seam : WeldSeams)
    {
        if (seam.SectionCount >= 2)
            continue;

        gp_Pnt sp(seam.StartPoint.X, seam.StartPoint.Y, seam.StartPoint.Z);
        gp_Pnt ep(seam.EndPoint.X, seam.EndPoint.Y, seam.EndPoint.Z);

        int shapeIdxs[2] = { seam.ShapeIdx1, seam.ShapeIdx2 };
        for (int si = 0; si < 2; si++)
        {
            int idx = shapeIdxs[si];
            if (idx < 0 || idx >= (int)m_Solids.size())
                continue;

            for (TopExp_Explorer faceExp(m_Solids[idx], TopAbs_FACE); faceExp.More(); faceExp.Next())
            {
                TopoDS_Face face = TopoDS::Face(faceExp.Current());

                if (!seam.FormingFace1.IsNull() && face.IsSame(seam.FormingFace1))
                    continue;
                if (!seam.FormingFace2.IsNull() && face.IsSame(seam.FormingFace2))
                    continue;

                if (!IsFacePlanar(face))
                    continue;

                GeomAPI_ProjectPointOnSurf projS(sp, BRep_Tool::Surface(face));
                GeomAPI_ProjectPointOnSurf projE(ep, BRep_Tool::Surface(face));

                double distS = projS.NbPoints() > 0 ? projS.LowerDistance() : 1e10;
                double distE = projE.NbPoints() > 0 ? projE.LowerDistance() : 1e10;

                if (distS > 5.0 && distE > 5.0)
                    continue;

                if (distS < distE && distS < 5.0)
                {
                    if (CheckNormalConsistency(ep, sp, face))
                    {
                        seam.TruncationFace1 = face;
                        seam.SectionCount = std::max(seam.SectionCount, 1);
                    }
                }
                else if (distE < 5.0)
                {
                    if (CheckNormalConsistency(sp, ep, face))
                    {
                        seam.TruncationFace2 = face;
                        seam.SectionCount = std::max(seam.SectionCount, 1);
                    }
                }
            }
        }
    }
}

// ==============================
// AreSeamsDuplicate - 判断两条焊缝是否重复
// ==============================
bool WeldExtractor::AreSeamsDuplicate(const WeldSeam& a, const WeldSeam& b, double tol)
{
    gp_Pnt a0(a.StartPoint.X, a.StartPoint.Y, a.StartPoint.Z);
    gp_Pnt a1(a.EndPoint.X, a.EndPoint.Y, a.EndPoint.Z);
    gp_Pnt b0(b.StartPoint.X, b.StartPoint.Y, b.StartPoint.Z);
    gp_Pnt b1(b.EndPoint.X, b.EndPoint.Y, b.EndPoint.Z);

    bool sameDir = (PointDist(a0, b0) <= tol && PointDist(a1, b1) <= tol);
    bool revDir = (PointDist(a0, b1) <= tol && PointDist(a1, b0) <= tol);

    return sameDir || revDir;
}

// ==============================
// RemoveDuplicateSeams - 步骤5: 去除重叠焊缝
// ==============================
// ==============================
// RemoveDuplicateSeams - 步骤5：去重并重新编号
// 关键逻辑：按长度降序→长焊缝优先保留→端点去重(tol=2.0)→重新分配Id
// ==============================
void WeldExtractor::RemoveDuplicateSeams()
{
    if (WeldSeams.empty()) return;

    const double tol = 2.0;

    // 按长度排序(长焊缝优先保留)
    std::sort(WeldSeams.begin(), WeldSeams.end(), SortWeldByLength);

    std::vector<WeldSeam> unique;
    unique.reserve(WeldSeams.size());

    for (const auto& seam : WeldSeams)
    {
        bool isDuplicate = false;
        for (const auto& existing : unique)
        {
            if (AreSeamsDuplicate(seam, existing, tol))
            {
                isDuplicate = true;
                break;
            }
        }
        if (!isDuplicate)
            unique.push_back(seam);
    }

    WeldSeams = std::move(unique);

    // 重新编号
    for (size_t i = 0; i < WeldSeams.size(); i++)
        WeldSeams[i].Id = (int)i + 1;
}

// ==============================
// WeldEdgeLength - 辅助函数
// ==============================
double WeldExtractor::WeldEdgeLength(const TopoDS_Edge& edge)
{
    return WeldEdgeLen(edge);
}

// ========== Weld seam computation entry ==========
std::vector<WeldSeam> COCCMFCDoc::ComputeWeldSeams()
{
    std::vector<WeldSeam> result;
    if (m_ImportedModel.IsNull()) return result;

    WeldExtractor extractor;
    extractor.inputShape = m_ImportedModel->Shape();
    extractor.Compute();
    result = extractor.WeldSeams;

    // Apply model transform (mm->m scaling + position offset)
    gp_Trsf tsf = m_ImportedModel->LocalTransformation();

    BRep_Builder b; TopoDS_Compound c; b.MakeCompound(c);
    for (auto& ws : result) {
        gp_Pnt sp(ws.StartPoint.X, ws.StartPoint.Y, ws.StartPoint.Z);
        gp_Pnt ep(ws.EndPoint.X, ws.EndPoint.Y, ws.EndPoint.Z);
        sp.Transform(tsf);
        ep.Transform(tsf);
        ws.StartPoint.X = sp.X() * 1000.0; ws.StartPoint.Y = sp.Y() * 1000.0; ws.StartPoint.Z = sp.Z() * 1000.0;
        ws.EndPoint.X = ep.X() * 1000.0; ws.EndPoint.Y = ep.Y() * 1000.0; ws.EndPoint.Z = ep.Z() * 1000.0;
        BRepBuilderAPI_MakeEdge me(sp, ep);
        if (me.IsDone()) b.Add(c, me.Edge());
    }
    m_WeldSeamShape = c;
    return result;
}
