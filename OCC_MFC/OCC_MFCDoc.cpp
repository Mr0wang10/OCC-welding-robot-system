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
#include <signal.h>
#include <setjmp.h>
#include <eh.h>
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
#include <Standard_ErrorHandler.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <IntCurvesFace_ShapeIntersector.hxx>
#include <ShapeAnalysis_FreeBounds.hxx>
#include <GeomAdaptor_Curve.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepLProp_SLProps.hxx>
#include <BRepTopAdaptor_FClass2d.hxx>
#include <ElCLib.hxx>
#include <Geom2d_Curve.hxx>
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
// ==============================
static jmp_buf g_AbortJmpBuf;
static void SigAbortHandler(int) { longjmp(g_AbortJmpBuf, 1); }
static void SEHTranslator(unsigned int, EXCEPTION_POINTERS*) { throw std::exception(); }
static void SetupAbortCatcher()
{
    _set_abort_behavior(0, _WRITE_ABORT_MSG | _CALL_REPORTFAULT);
    signal(SIGABRT, SigAbortHandler);
    _set_se_translator(SEHTranslator);
}
static void RestoreAbortCatcher()
{
    signal(SIGABRT, SIG_DFL);
}
// Compute - 主入口: 执行完整5步焊缝提取
// 整体用 setjmp 保护，任一环节 abort() 则安全退出
// ==============================
void WeldExtractor::Compute()
{
    SetupAbortCatcher();

    // 检查是否从 longjmp/setjmp 返回（即 abort() 已被拦截）
    if (setjmp(g_AbortJmpBuf) != 0)
    {
        RestoreAbortCatcher();
        return;
    }

    // __try/__except 捕获访问违例等 SEH 异常
    __try
    {
        if (inputShape.IsNull()) { RestoreAbortCatcher(); return; }
        WeldSeams.clear();
        m_Solids.clear();

        BuildTopologyMaps();
        if (m_Solids.size() < 2) { RestoreAbortCatcher(); return; }

        ExtractInitialWelds();

        if (WeldSeams.empty()) { RestoreAbortCatcher(); return; }

        DetectTruncations();
        DetectInternalSections();
        RemoveDuplicateSeams();

        // ===== 步骤6: 后处理拓扑清洗过滤器 =====
        if (!WeldSeams.empty() && !inputShape.IsNull())
        {
            MergeAndAlignBrokenSeams();
            ValidateAndClipSeamsByFaces(inputShape);
            FilterByWeldingProcess(inputShape);
        }
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        // 任何 SEH 异常（AV/除零等）均被捕获，安全退出
    }

    RestoreAbortCatcher();
}

// ==============================
// BuildTopologyMaps - 步骤1: 提取所有子模型（零件）
// ==============================
void WeldExtractor::BuildTopologyMaps()
{
    m_Solids.clear();

    for (TopExp_Explorer x(inputShape, TopAbs_SOLID); x.More(); x.Next())
        m_Solids.push_back(TopoDS::Solid(x.Current()));
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
// GetSolidThickness - 估算实体的板厚/壁厚（取包围盒最小尺寸）
// 关键逻辑：用于过滤长度≤板厚的伪焊缝（倒角边、折边等）
// ==============================
static double GetSolidThickness(const TopoDS_Solid& solid)
{
    try
    {
        Bnd_Box box;
        BRepBndLib::Add(solid, box);
        Standard_Real xmin, ymin, zmin, xmax, ymax, zmax;
        box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
        double dx = xmax - xmin;
        double dy = ymax - ymin;
        double dz = zmax - zmin;
        // 最小维度即为板厚/壁厚
        if (dx <= dy && dx <= dz) return dx;
        if (dy <= dx && dy <= dz) return dy;
        return dz;
    }
    catch (...)
    {
        return 1e10;  // 无法计算时返回大值，避免误过滤
    }
}

// ==============================
// FindEdgeOwnerFace - 在指定 Solid 中找到包含该 Edge 的面（形成面）
// 关键逻辑：遍历 Solid 的所有 Face，检查 Edge 是否属于该 Face
// ==============================
static TopoDS_Face FindEdgeOwnerFace(const TopoDS_Edge& edge, const TopoDS_Solid& solid)
{
    for (TopExp_Explorer fExp(solid, TopAbs_FACE); fExp.More(); fExp.Next())
    {
        TopoDS_Face face = TopoDS::Face(fExp.Current());
        for (TopExp_Explorer eExp(face, TopAbs_EDGE); eExp.More(); eExp.Next())
        {
            if (BRepTools::Compare(edge, TopoDS::Edge(eExp.Current())))
                return face;
        }
    }
    return TopoDS_Face();
}

// ==============================
// IsSolidValid - 检查 Solid 是否有效（包围盒非退化）
// Debug 模式下 OCCT abort() 不可捕获，需前置预防
// ==============================
static bool IsSolidValid(const TopoDS_Solid& solid)
{
    if (solid.IsNull()) return false;
    try {
        Bnd_Box box;
        BRepBndLib::Add(solid, box);
        if (box.IsVoid()) return false;
        Standard_Real xmin, ymin, zmin, xmax, ymax, zmax;
        box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
        double dx = xmax - xmin, dy = ymax - ymin, dz = zmax - zmin;
        // 任一边长过小（<0.001mm）视为退化
        if (dx < 0.001 || dy < 0.001 || dz < 0.001) return false;
        return true;
    }
    catch (...) { return false; }
}

// ==============================
// ==============================
// GetFaceArea - 计算面的面积（用于区分大面/小面）
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
// IsContainmentRelation - 判断两个面的包含关系（通过UV参数范围）
// ==============================
static bool IsContainmentRelation(const TopoDS_Face& face1, const TopoDS_Face& face2)
{
    auto getRange = [](const TopoDS_Face& f, double& uMin, double& uMax, double& vMin, double& vMax)
    {
        TopoDS_Wire outerWire = BRepTools::OuterWire(f);
        if (outerWire.IsNull()) return;
        Handle(Geom_Surface) surface = BRep_Tool::Surface(f);
        if (surface.IsNull()) return;
        bool first = true;
        for (TopExp_Explorer eExp(outerWire, TopAbs_EDGE); eExp.More(); eExp.Next())
        {
            TopoDS_Edge edge = TopoDS::Edge(eExp.Current());
            Standard_Real f0 = 0.0, l0 = 0.0;
            Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, f0, l0);
            if (curve.IsNull()) continue;
            for (int k = 0; k <= 10; k++)
            {
                double param = f0 + (l0 - f0) * k / 10.0;
                gp_Pnt pt = curve->Value(param);
                Standard_Real u = 0.0, v = 0.0;
                GeomAPI_ProjectPointOnSurf projector(pt, surface);
                if (projector.NbPoints() > 0)
                {
                    projector.LowerDistanceParameters(u, v);
                    if (first) { uMin=uMax=u; vMin=vMax=v; first=false; }
                    else { uMin=std::min(uMin,u); uMax=std::max(uMax,u); vMin=std::min(vMin,v); vMax=std::max(vMax,v); }
                }
            }
        }
    };

    double uMin1,uMax1,vMin1,vMax1, uMin2,uMax2,vMin2,vMax2;
    getRange(face1, uMin1,uMax1,vMin1,vMax1);
    getRange(face2, uMin2,uMax2,vMin2,vMax2);
    double tol = 1.0;
    return (uMin2-tol<=uMin1 && uMax2+tol>=uMax1 && vMin2-tol<=vMin1 && vMax2+tol>=vMax1)
        || (uMin1-tol<=uMin2 && uMax1+tol>=uMax2 && vMin1-tol<=vMin2 && vMax1+tol>=vMax2);
}

// ==============================
// EdgeToWeldSeam - 将边转为焊缝数据结构
// ==============================
static bool EdgeToWeldSeam(const TopoDS_Edge& edge, WeldSeam& seam, int shapeI, int shapeJ,
                            const TopoDS_Solid& solidI, const TopoDS_Solid& solidJ)
{
    seam = WeldSeam();
    Standard_Real f=0.0,l=0.0;
    Handle(Geom_Curve) c = BRep_Tool::Curve(edge,f,l);
    if (c.IsNull()) return false;

    seam.Length = WeldEdgeLen(edge);
    if (seam.Length < 3.0) return false;
    seam.CurveType = c->DynamicType()->Name();
    seam.ShapeIdx1 = shapeI;
    seam.ShapeIdx2 = shapeJ;
    seam.EdgeGeom = edge;

    TopoDS_Vertex v1,v2;
    TopExp::Vertices(edge,v1,v2);
    if (!v1.IsNull()) { gp_Pnt p=BRep_Tool::Pnt(v1);
        seam.StartPoint.X=p.X(); seam.StartPoint.Y=p.Y(); seam.StartPoint.Z=p.Z(); }
    if (!v2.IsNull()) { gp_Pnt p=BRep_Tool::Pnt(v2);
        seam.EndPoint.X=p.X(); seam.EndPoint.Y=p.Y(); seam.EndPoint.Z=p.Z(); }

    if (c->DynamicType()==STANDARD_TYPE(Geom_Circle))
    {
        Handle(Geom_Circle) ci = Handle(Geom_Circle)::DownCast(c);
        seam.CenterX=ci->Location().X(); seam.CenterY=ci->Location().Y();
        seam.CenterZ=ci->Location().Z(); seam.Radius=ci->Radius();
    }

    // 形成面绑定
    seam.FormingFace1 = FindEdgeOwnerFace(edge, solidI);
    seam.FormingFace2 = FindEdgeOwnerFace(edge, solidJ);

    return true;
}

// ==============================
// ExtractInitialWelds - 步骤2：基于面配对的初始焊缝提取（专利方法）
// 稳定版：OBB预筛 + 面法线反向 + 距离检测 + 包含/相交分类
// 新增过滤：板厚过滤 + 外轮廓边过滤 + 相邻面夹角过滤
// ==============================
void WeldExtractor::ExtractInitialWelds()
{
    int n = (int)m_Solids.size();
    if (n < 2) return;

    // 前置过滤：剔除退化Solid
    std::vector<int> validIdx;
    for (int k = 0; k < n; k++)
        if (IsSolidValid(m_Solids[k])) validIdx.push_back(k);
    if ((int)validIdx.size() < 2) return;

    // 预计算所有平面面法线
    struct FaceInfo { TopoDS_Face face; gp_Vec normal; int solidIdx; };
    std::vector<std::vector<FaceInfo>> allFaces(validIdx.size());

    for (size_t ii = 0; ii < validIdx.size(); ii++)
    {
        int si = validIdx[ii];
        for (TopExp_Explorer fExp(m_Solids[si], TopAbs_FACE); fExp.More(); fExp.Next())
        {
            TopoDS_Face face = TopoDS::Face(fExp.Current());
            if (!IsFacePlanar(face)) continue;
            gp_Vec normal = ComputeFaceNormal(face);
            if (normal.Magnitude() < Precision::Confusion()) continue;
            normal.Normalize();
            allFaces[ii].push_back({face, normal, si});
        }
    }

    for (size_t ii = 0; ii < validIdx.size(); ii++)
    {
        int i = validIdx[ii];
        for (size_t jj = ii + 1; jj < validIdx.size(); jj++)
        {
            int j = validIdx[jj];

            // OBB 预筛
            try {
                Bnd_Box bI, bJ;
                BRepBndLib::Add(m_Solids[i], bI);
                BRepBndLib::Add(m_Solids[j], bJ);
                if (bI.IsOut(bJ)) continue;
            } catch (...) { continue; }



            // 遍历面配对，找法线反向的接触面
            for (const auto& fi : allFaces[ii])
            {
                for (const auto& fj : allFaces[jj])
                {
                    try
                    {
                        // 法线反向检测（面对面贴合）
                        double dot = fi.normal.Dot(fj.normal);
                        if (dot >= -0.95) continue;

                        // 面级OBB
                        Bnd_Box bFi, bFj;
                        BRepBndLib::Add(fi.face, bFi);
                        BRepBndLib::Add(fj.face, bFj);
                        if (bFi.IsOut(bFj)) continue;



                        // 判定包含/相交关系
                        double areaI = GetFaceArea(fi.face);
                        double areaJ = GetFaceArea(fj.face);
                        const TopoDS_Face& smallFace = (areaI <= areaJ) ? fi.face : fj.face;
                        const TopoDS_Face& bigFace   = (areaI <= areaJ) ? fj.face : fi.face;
                        int sI = (areaI <= areaJ) ? i : j;
                        int sJ = (areaI <= areaJ) ? j : i;
                        const TopoDS_Solid& sSolid = (areaI <= areaJ) ? m_Solids[i] : m_Solids[j];
                        const TopoDS_Solid& bSolid = (areaI <= areaJ) ? m_Solids[j] : m_Solids[i];

                        if (IsContainmentRelation(smallFace, bigFace))
                        {
                            // 包含关系：取小面外轮廓边
                            std::vector<TopoDS_Edge> edges = GetFaceOuterEdges(smallFace);
                            for (const auto& edge : edges)
                            {
                                double eLen = WeldEdgeLen(edge);
                                if (eLen < 3.0) continue;

                                // 板厚过滤
                                double tI = GetSolidThickness(m_Solids[i]);
                                double tJ = GetSolidThickness(m_Solids[j]);
                                if (eLen <= ((tI>tJ)?tI:tJ) * 1.15) continue;
                                WeldSeam seam;
                                if (EdgeToWeldSeam(edge, seam, sI, sJ, sSolid, bSolid))
                                {
                                    seam.Id = (int)WeldSeams.size() + 1;
                                    WeldSeams.push_back(seam);
                                }
                            }
                        }
                        else
                        {
                            // 相交关系：面-面级 BRepAlgoAPI_Section（稳定，不会崩溃）
                            try
                            {
                                BRepAlgoAPI_Section section(smallFace, bigFace, Standard_False);
                                section.SetFuzzyValue(0.2);  // 开启模糊布尔，容忍 0.2mm 装配间隙
                                section.ComputePCurveOn1(Standard_True);
                                section.Approximation(Standard_True);
                                section.Build();
                                if (!section.IsDone()) continue;

                                TopoDS_Shape res = section.Shape();
                                for (TopExp_Explorer eExp(res, TopAbs_EDGE); eExp.More(); eExp.Next())
                                {
                                    TopoDS_Edge edge = TopoDS::Edge(eExp.Current());
                                    if (edge.IsNull()) continue;

                                    double eLen = WeldEdgeLen(edge);
                                    if (eLen < 3.0) continue;

                                    double tI = GetSolidThickness(m_Solids[i]);
                                    double tJ = GetSolidThickness(m_Solids[j]);
                                    if (eLen <= ((tI>tJ)?tI:tJ) * 1.15) continue;
                                    WeldSeam seam;
                                    if (EdgeToWeldSeam(edge, seam, sI, sJ, sSolid, bSolid))
                                    {
                                        seam.Id = (int)WeldSeams.size() + 1;
                                        WeldSeams.push_back(seam);
                                    }
                                }
                            }
                            catch (...) { continue; }
                        }
                    }
                    catch (...) { continue; }
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
// HandleSingleSideTruncation - 单侧截断：切除截断部分，保留剩余焊缝段
// 专利步骤3：BRepAlgoAPI_Cut 等效实现，通过 Geom_TrimmedCurve 裁剪曲线
// 关键逻辑：找到截断交点→投影到曲线上→确定哪端被截断→裁剪→更新EdgeGeom
// ==============================
void WeldExtractor::HandleSingleSideTruncation(WeldSeam& seam, const TopoDS_Solid& cuttingSolid)
{
    if (seam.SectionCount != 0) return;
    seam.SectionCount = 1;

    Standard_Real f = 0.0, l = 0.0;
    Handle(Geom_Curve) curve = BRep_Tool::Curve(seam.EdgeGeom, f, l);
    if (curve.IsNull()) return;

    gp_Pnt sp(seam.StartPoint.X, seam.StartPoint.Y, seam.StartPoint.Z);
    gp_Pnt ep(seam.EndPoint.X, seam.EndPoint.Y, seam.EndPoint.Z);

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
                gp_Pnt interPnt = intersector.Pnt(1);

                GeomAPI_ProjectPointOnCurve projector(interPnt, curve);
                if (projector.NbPoints() > 0)
                {
                    Standard_Real uInter = projector.Parameter(1);
                    double dS = sp.Distance(interPnt);
                    double dE = ep.Distance(interPnt);

                    // 保留未被截断的一端
                    Standard_Real uNewStart = (dS < dE) ? f : uInter;
                    Standard_Real uNewEnd   = (dS < dE) ? uInter : l;

                    Handle(Geom_TrimmedCurve) trimmed = ::new Geom_TrimmedCurve(curve, uNewStart, uNewEnd);
                    BRepBuilderAPI_MakeEdge mkNewEdge(trimmed);
                    if (mkNewEdge.IsDone())
                    {
                        seam.EdgeGeom = mkNewEdge.Edge();
                        seam.Length = WeldEdgeLen(seam.EdgeGeom);
                        TopoDS_Vertex v1, v2;
                        TopExp::Vertices(seam.EdgeGeom, v1, v2);
                        if (!v1.IsNull()) { gp_Pnt p = BRep_Tool::Pnt(v1);
                            seam.StartPoint.X=p.X(); seam.StartPoint.Y=p.Y(); seam.StartPoint.Z=p.Z(); }
                        if (!v2.IsNull()) { gp_Pnt p = BRep_Tool::Pnt(v2);
                            seam.EndPoint.X=p.X(); seam.EndPoint.Y=p.Y(); seam.EndPoint.Z=p.Z(); }
                    }
                }
                break;
            }
        }
        catch (...) { continue; }
    }
}

// ==============================
// HandleDoubleSideTruncation - 双侧截断：将焊缝分割为两条独立新焊缝
// 专利步骤3：将原焊缝切割为两段，分别绑定截断面
// 返回新创建的焊缝，由 DetectTruncations 统一加入列表
// ==============================
bool WeldExtractor::HandleDoubleSideTruncation(WeldSeam& seam, const TopoDS_Solid& cuttingSolid, WeldSeam& outNewSeam)
{
    if (seam.SectionCount >= 2) return false;
    seam.SectionCount = 2;

    Standard_Real f = 0.0, l = 0.0;
    Handle(Geom_Curve) curve = BRep_Tool::Curve(seam.EdgeGeom, f, l);
    if (curve.IsNull()) return false;

    gp_Pnt sp(seam.StartPoint.X, seam.StartPoint.Y, seam.StartPoint.Z);
    gp_Pnt ep(seam.EndPoint.X, seam.EndPoint.Y, seam.EndPoint.Z);

    // 收集所有交点参数
    std::vector<Standard_Real> params;
    TopoDS_Face truncFaceNearS, truncFaceNearE;

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
                GeomAPI_ProjectPointOnCurve projector(pt, curve);
                if (projector.NbPoints() > 0)
                {
                    Standard_Real u = projector.Parameter(1);
                    params.push_back(u);

                    double dS = sp.Distance(pt);
                    double dE = ep.Distance(pt);
                    if (dS < dE - Precision::Confusion()) { truncFaceNearS = face; }
                    else if (dE < dS - Precision::Confusion()) { truncFaceNearE = face; }
                    // 对称情况：距离相等时用法线方向一致性判定
                    else {
                        gp_Vec edgeDir = gp_Vec(ep.X()-sp.X(), ep.Y()-sp.Y(), ep.Z()-sp.Z());
                        gp_Vec faceNorm = ComputeFaceNormal(face);
                        faceNorm.Normalize();
                        if (edgeDir.Magnitude() > Precision::Confusion()) {
                            edgeDir.Normalize();
                            double normDot = edgeDir.Dot(faceNorm);
                            if (normDot < -0.5) truncFaceNearS = face;
                            else truncFaceNearE = face;
                        }
                    }
                }
            }
        }
        catch (...) { continue; }
    }

    if (params.size() < 2) return false;

    // 排序取两个最外侧交点
    std::sort(params.begin(), params.end());
    Standard_Real u1 = params.front();  // 靠近起点
    Standard_Real u2 = params.back();   // 靠近终点

    if (fabs(u1 - u2) < Precision::Confusion()) return false;

    // 更新原 seam 为第一段 [f, u1]
    Handle(Geom_TrimmedCurve) trimmed1 = ::new Geom_TrimmedCurve(curve, f, u1);
    BRepBuilderAPI_MakeEdge mkEdge1(trimmed1);
    if (!mkEdge1.IsDone()) return false;

    // 记录原 seam 信息用于创建新 seam
    int origShapeIdx1 = seam.ShapeIdx1;
    int origShapeIdx2 = seam.ShapeIdx2;
    TopoDS_Face origForming1 = seam.FormingFace1;
    TopoDS_Face origForming2 = seam.FormingFace2;

    // 更新原 seam
    seam.EdgeGeom = mkEdge1.Edge();
    seam.Length = WeldEdgeLen(seam.EdgeGeom);
    seam.TruncationFace1 = truncFaceNearS;
    seam.TruncationFace2 = TopoDS_Face();  // 第二段需要内部截面检测
    TopoDS_Vertex v1a, v2a;
    TopExp::Vertices(seam.EdgeGeom, v1a, v2a);
    if (!v1a.IsNull()) { gp_Pnt p = BRep_Tool::Pnt(v1a);
        seam.StartPoint.X=p.X(); seam.StartPoint.Y=p.Y(); seam.StartPoint.Z=p.Z(); }
    if (!v2a.IsNull()) { gp_Pnt p = BRep_Tool::Pnt(v2a);
        seam.EndPoint.X=p.X(); seam.EndPoint.Y=p.Y(); seam.EndPoint.Z=p.Z(); }

    // 创建新 seam 为第二段 [u2, l]
    Handle(Geom_TrimmedCurve) trimmed2 = ::new Geom_TrimmedCurve(curve, u2, l);
    BRepBuilderAPI_MakeEdge mkEdge2(trimmed2);
    if (!mkEdge2.IsDone()) return false;

    outNewSeam = WeldSeam();
    outNewSeam.EdgeGeom = mkEdge2.Edge();
    outNewSeam.Length = WeldEdgeLen(outNewSeam.EdgeGeom);
    outNewSeam.ShapeIdx1 = origShapeIdx1;
    outNewSeam.ShapeIdx2 = origShapeIdx2;
    outNewSeam.FormingFace1 = origForming1;
    outNewSeam.FormingFace2 = origForming2;
    outNewSeam.TruncationFace1 = truncFaceNearE;
    outNewSeam.TruncationFace2 = TopoDS_Face();
    outNewSeam.SectionCount = 1;
    TopoDS_Vertex v1b, v2b;
    TopExp::Vertices(outNewSeam.EdgeGeom, v1b, v2b);
    if (!v1b.IsNull()) { gp_Pnt p = BRep_Tool::Pnt(v1b);
        outNewSeam.StartPoint.X=p.X(); outNewSeam.StartPoint.Y=p.Y(); outNewSeam.StartPoint.Z=p.Z(); }
    if (!v2b.IsNull()) { gp_Pnt p = BRep_Tool::Pnt(v2b);
        outNewSeam.EndPoint.X=p.X(); outNewSeam.EndPoint.Y=p.Y(); outNewSeam.EndPoint.Z=p.Z(); }

    // 确定第二段的曲线类型
    Handle(Geom_Curve) c2 = BRep_Tool::Curve(outNewSeam.EdgeGeom, f, l);
    if (!c2.IsNull()) outNewSeam.CurveType = c2->DynamicType()->Name();

    return true;
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

        for (int si = 0; si < n; si++)
        {
            if (si == seam.ShapeIdx1 || si == seam.ShapeIdx2)
                continue;

            try
            {
                Bnd_Box boxEdge, boxSolid;
                BRepBndLib::Add(seam.EdgeGeom, boxEdge);
                BRepBndLib::Add(m_Solids[si], boxSolid);
                if (boxEdge.IsOut(boxSolid))
                    continue;

                std::vector<gp_Pnt> pts;
                int nPts = CountIntersectionPoints(seam.EdgeGeom, m_Solids[si], pts);

                if (nPts == 0) continue;
                else if (nPts == 1) HandleSingleSideTruncation(seam, m_Solids[si]);
                else if (nPts >= 2)
                {
                    WeldSeam newSeam;
                    if (HandleDoubleSideTruncation(seam, m_Solids[si], newSeam))
                    {
                        newSeam.Id = (int)WeldSeams.size() + 1;
                        WeldSeams.push_back(newSeam);
                    }
                }
            }
            catch (...) { continue; }
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
// GetAdjacentFaces - 获取与指定面共享边的相邻平面（专利要求）
// 关键逻辑：仅返回与 formingFace 共享边且非 formingFace 自身的面
// ==============================
static std::vector<TopoDS_Face> GetAdjacentFaces(const TopoDS_Face& formingFace, const TopoDS_Solid& solid)
{
    std::vector<TopoDS_Face> result;
    if (formingFace.IsNull()) return result;

    TopTools_IndexedDataMapOfShapeListOfShape edgeFaceMap;
    TopExp::MapShapesAndAncestors(solid, TopAbs_EDGE, TopAbs_FACE, edgeFaceMap);

    for (TopExp_Explorer edgeExp(formingFace, TopAbs_EDGE); edgeExp.More(); edgeExp.Next())
    {
        const TopoDS_Shape& edge = edgeExp.Current();
        const TopTools_ListOfShape& faces = edgeFaceMap.FindFromKey(edge);
        for (TopTools_ListIteratorOfListOfShape it(faces); it.More(); it.Next())
        {
            const TopoDS_Face& candidateFace = TopoDS::Face(it.Value());
            if (!candidateFace.IsSame(formingFace))
            {
                // 去重
                bool already = false;
                for (const auto& existing : result)
                    if (existing.IsSame(candidateFace)) { already = true; break; }
                if (!already) result.push_back(candidateFace);
            }
        }
    }
    return result;
}

// ==============================
// DetectInternalSections - 步骤4：内部截面检测（专利实现）
// 仅检测与形成面相接的面（stp4：遍历子模型计算与焊缝的形成面相接的面）
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
        TopoDS_Face formingFaces[2] = { seam.FormingFace1, seam.FormingFace2 };

        for (int si = 0; si < 2; si++)
        {
            int idx = shapeIdxs[si];
            if (idx < 0 || idx >= (int)m_Solids.size())
                continue;

            // 仅检查与形成面相接的面（专利要求）
            std::vector<TopoDS_Face> adjacentFaces;
            if (!formingFaces[si].IsNull())
                adjacentFaces = GetAdjacentFaces(formingFaces[si], m_Solids[idx]);
            else
            {
                // 如果没有形成面，回退遍历所有面
                for (TopExp_Explorer faceExp(m_Solids[idx], TopAbs_FACE); faceExp.More(); faceExp.Next())
                    adjacentFaces.push_back(TopoDS::Face(faceExp.Current()));
            }

            for (const auto& face : adjacentFaces)
            {
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

// ==============================
// MergeAndAlignBrokenSeams - 碎段长直线熔接器
// 工艺目的：将被立板切碎、宽度错位的共线碎段拉直融合成贯通长线
// 关键逻辑：空间距离 + 投影重叠度判定
// ==============================
void WeldExtractor::MergeAndAlignBrokenSeams()
{
    std::vector<WeldSeam> mergedList;
    const double DIST_THRESHOLD = 6.0;    // 轴间距容差(mm)
    const double ANGLE_THRESHOLD = 0.05;  // 平行度弧度容差(~3度)
    const double GAP_THRESHOLD = 8.0;     // 投影间隙容差(mm)

    for (const auto& current : WeldSeams)
    {
        bool isMerged = false;
        gp_Pnt s1(current.StartPoint.X, current.StartPoint.Y, current.StartPoint.Z);
        gp_Pnt e1(current.EndPoint.X, current.EndPoint.Y, current.EndPoint.Z);
        gp_Vec v1(s1, e1);
        if (v1.Magnitude() < 1e-3) continue;
        gp_Lin lin1(s1, gp_Dir(v1));

        for (auto& target : mergedList)
        {
            gp_Pnt s2(target.StartPoint.X, target.StartPoint.Y, target.StartPoint.Z);
            gp_Pnt e2(target.EndPoint.X, target.EndPoint.Y, target.EndPoint.Z);
            gp_Vec v2(s2, e2);
            if (v2.Magnitude() < 1e-3) continue;

            if (v1.IsParallel(v2, ANGLE_THRESHOLD))
            {
                double d = lin1.Distance(s2);
                if (d < DIST_THRESHOLD)
                {
                    double p_s1 = ElCLib::Parameter(lin1, s1);
                    double p_e1 = ElCLib::Parameter(lin1, e1);
                    double p_s2 = ElCLib::Parameter(lin1, s2);
                    double p_e2 = ElCLib::Parameter(lin1, e2);

                    double min1 = std::min(p_s1, p_e1), max1 = std::max(p_s1, p_e1);
                    double min2 = std::min(p_s2, p_e2), max2 = std::max(p_s2, p_e2);

                    if (!(max1 < min2 - GAP_THRESHOLD || max2 < min1 - GAP_THRESHOLD))
                    {
                        double finalMin = std::min(min1, min2);
                        double finalMax = std::max(max1, max2);

                        gp_Pnt newStart = ElCLib::Value(finalMin, lin1);
                        gp_Pnt newEnd = ElCLib::Value(finalMax, lin1);

                        target.StartPoint.X = newStart.X();
                        target.StartPoint.Y = newStart.Y();
                        target.StartPoint.Z = newStart.Z();
                        target.EndPoint.X = newEnd.X();
                        target.EndPoint.Y = newEnd.Y();
                        target.EndPoint.Z = newEnd.Z();

                        isMerged = true;
                        break;
                    }
                }
            }
        }
        if (!isMerged)
            mergedList.push_back(current);
    }
    WeldSeams = std::move(mergedList);
}

// ==============================
// ValidateAndClipSeamsByFaces - 3点动态探针孔洞检测
// 利用 BRepExtrema_DistShapeShape 定位最近面 + BRepTopAdaptor_FClass2d 二维拓扑分类
// 取 0.2/0.5/0.8 三处采样点，任意一点在孔内（TopAbs_OUT）则拦截
// ==============================
void WeldExtractor::ValidateAndClipSeamsByFaces(const TopoDS_Shape& globalShape)
{
    std::vector<WeldSeam> validatedSeams;

    for (const auto& seam : WeldSeams)
    {
        gp_Pnt s(seam.StartPoint.X, seam.StartPoint.Y, seam.StartPoint.Z);
        gp_Pnt e(seam.EndPoint.X, seam.EndPoint.Y, seam.EndPoint.Z);

        // 3 点动态步长探针：起点附近、中点、终点附近
        std::array<double, 3> t_samples = { 0.2, 0.5, 0.8 };
        bool isAnyPointInHole = false;

        for (double t : t_samples)
        {
            gp_Pnt samplePt(
                s.X() + (e.X() - s.X()) * t,
                s.Y() + (e.Y() - s.Y()) * t,
                s.Z() + (e.Z() - s.Z()) * t
            );

            // 寻找全局实体中离采样探针点最近的物理 Face
            TopoDS_Face closestFace;
            double minDistance = std::numeric_limits<double>::max();

            TopExp_Explorer expF(globalShape, TopAbs_FACE);
            for (; expF.More(); expF.Next())
            {
                TopoDS_Face face = TopoDS::Face(expF.Current());
                BRepExtrema_DistShapeShape extrema(samplePt, face);
                if (extrema.IsDone() && extrema.NbSolution() > 0)
                {
                    double dist = extrema.Value();
                    if (dist < minDistance)
                    {
                        minDistance = dist;
                        closestFace = face;
                    }
                }
            }

            // 探针点距离任何面都 > 2mm → 悬空在空气中
            if (closestFace.IsNull() || minDistance > 2.0)
            {
                isAnyPointInHole = true;
                break;
            }

            // 将采样点投影到最近面上，反求 UV 参数
            double uParam = 0.0, vParam = 0.0;
            BRepExtrema_DistShapeShape extremaProj(samplePt, closestFace);
            if (extremaProj.IsDone() && extremaProj.NbSolution() > 0)
            {
                extremaProj.ParOnFaceS1(1, uParam, vParam);
            }
            else
            {
                continue;
            }

            // 二维拓扑环分类器盘问 UV 是否在面拓扑外（圆孔空气区）
            gp_Pnt2d uvParam(uParam, vParam);
            BRepTopAdaptor_FClass2d classifier(closestFace, Precision::Confusion());
            TopAbs_State state = classifier.Perform(uvParam);

            if (state == TopAbs_OUT)
            {
                isAnyPointInHole = true;
                break;
            }
        }

        // 只有所有探针点都在真实金属面上才保留
        if (!isAnyPointInHole)
            validatedSeams.push_back(seam);
    }

    WeldSeams = std::move(validatedSeams);
}

// ==============================
// CheckIfFlatContact - 判断两面是否为平贴接触（过滤立板端头）
// 工艺原理：两面法线几乎平行（<10°或>170°）则为平贴安装面，不焊接
// ==============================
bool WeldExtractor::CheckIfFlatContact(const TopoDS_Edge& edge, const TopoDS_Face& face1, const TopoDS_Face& face2)
{
    double f = 0.0, l = 0.0;
    Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, f, l);
    if (curve.IsNull()) return true;

    double midParam = (f + l) * 0.5;

    BRepAdaptor_Surface surf1(face1);
    double u1 = 0.0, v1 = 0.0;
    Handle(Geom2d_Curve) c2d1 = BRep_Tool::CurveOnSurface(edge, face1, f, l);
    if (c2d1.IsNull()) return true;
    c2d1->Value(midParam).Coord(u1, v1);
    BRepLProp_SLProps props1(surf1, u1, v1, 1, Precision::Confusion());
    if (!props1.IsNormalDefined()) return true;
    gp_Dir norm1 = props1.Normal();
    if (face1.Orientation() == TopAbs_REVERSED) norm1.Reverse();

    BRepAdaptor_Surface surf2(face2);
    double u2 = 0.0, v2 = 0.0;
    Handle(Geom2d_Curve) c2d2 = BRep_Tool::CurveOnSurface(edge, face2, f, l);
    if (c2d2.IsNull()) return true;
    c2d2->Value(midParam).Coord(u2, v2);
    BRepLProp_SLProps props2(surf2, u2, v2, 1, Precision::Confusion());
    if (!props2.IsNormalDefined()) return true;
    gp_Dir norm2 = props2.Normal();
    if (face2.Orientation() == TopAbs_REVERSED) norm2.Reverse();

    double angleDeg = norm1.Angle(norm2) * 180.0 / M_PI;
    if (angleDeg < 10.0 || angleDeg > 170.0)
        return true;

    return false;
}

// ==============================
// IsConvexEdge - 凸凹性鉴别（过滤阳角保留阴角）
// 工艺原理：探针向两法线合成方向迈步，根据点积判定凸/凹
// ==============================
bool WeldExtractor::IsConvexEdge(const TopoDS_Edge& edge, const TopoDS_Face& face1, const TopoDS_Face& face2)
{
    double f = 0.0, l = 0.0;
    Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, f, l);
    if (curve.IsNull()) return false;

    double midParam = (f + l) * 0.5;

    BRepAdaptor_Surface surf1(face1);
    double u1 = 0.0, v1 = 0.0;
    Handle(Geom2d_Curve) c2d1 = BRep_Tool::CurveOnSurface(edge, face1, f, l);
    if (c2d1.IsNull()) return false;
    c2d1->Value(midParam).Coord(u1, v1);
    BRepLProp_SLProps props1(surf1, u1, v1, 1, Precision::Confusion());
    if (!props1.IsNormalDefined()) return false;
    gp_Dir norm1 = props1.Normal();
    if (face1.Orientation() == TopAbs_REVERSED) norm1.Reverse();

    BRepAdaptor_Surface surf2(face2);
    double u2 = 0.0, v2 = 0.0;
    Handle(Geom2d_Curve) c2d2 = BRep_Tool::CurveOnSurface(edge, face2, f, l);
    if (c2d2.IsNull()) return false;
    c2d2->Value(midParam).Coord(u2, v2);
    BRepLProp_SLProps props2(surf2, u2, v2, 1, Precision::Confusion());
    if (!props2.IsNormalDefined()) return false;
    gp_Dir norm2 = props2.Normal();
    if (face2.Orientation() == TopAbs_REVERSED) norm2.Reverse();

    // 两法线合成方向作为探针方向
    gp_Vec combineNorm = gp_Vec(norm1) + gp_Vec(norm2);
    if (combineNorm.Magnitude() < Precision::Confusion()) return false;
    combineNorm.Normalize();

    // 若合成方向与 face1 法线的点积为负 → 凸边（阳角）
    double check1 = combineNorm.Dot(gp_Vec(norm1));
    if (check1 < 0.0)
        return true; // 凸边，剔除

    return false; // 凹边（阴角焊缝根部），保留
}

// ==============================
// FilterByWeldingProcess - 工艺特征洗白过滤器
// 第1关：平贴接触过滤（干掉立板端头）
// 第2关：凸边过滤（干掉双胞胎阳角）
// ==============================
void WeldExtractor::FilterByWeldingProcess(const TopoDS_Shape& globalShape)
{
    std::vector<WeldSeam> filteredSeams;

    TopTools_IndexedDataMapOfShapeListOfShape edgeFaceMap;
    TopExp::MapShapesAndAncestors(globalShape, TopAbs_EDGE, TopAbs_FACE, edgeFaceMap);

    for (auto& seam : WeldSeams)
    {
        TopoDS_Edge currentEdge;
        gp_Pnt pStart(seam.StartPoint.X, seam.StartPoint.Y, seam.StartPoint.Z);
        gp_Pnt pEnd(seam.EndPoint.X, seam.EndPoint.Y, seam.EndPoint.Z);

        TopExp_Explorer expE(globalShape, TopAbs_EDGE);
        for (; expE.More(); expE.Next())
        {
            TopoDS_Edge e = TopoDS::Edge(expE.Current());
            double uFirst = 0.0, uLast = 0.0;
            Handle(Geom_Curve) c = BRep_Tool::Curve(e, uFirst, uLast);
            if (!c.IsNull())
            {
                gp_Pnt p1 = c->Value(uFirst);
                gp_Pnt p2 = c->Value(uLast);
                if ((p1.Distance(pStart) < 1.0 && p2.Distance(pEnd) < 1.0) ||
                    (p1.Distance(pEnd) < 1.0 && p2.Distance(pStart) < 1.0))
                {
                    currentEdge = e;
                    break;
                }
            }
        }

        if (currentEdge.IsNull() || !edgeFaceMap.Contains(currentEdge))
            continue;

        const TopTools_ListOfShape& faceList = edgeFaceMap.FindFromKey(currentEdge);
        if (faceList.Extent() < 2) continue;

        TopoDS_Face face1 = TopoDS::Face(faceList.First());
        TopoDS_Face face2 = TopoDS::Face(faceList.Last());

        // 第1关：平贴接触过滤（立板端头）
        if (CheckIfFlatContact(currentEdge, face1, face2))
            continue;

        // 第2关：凸边/阳角过滤（双胞胎平行阳角）
        if (IsConvexEdge(currentEdge, face1, face2))
            continue;

        filteredSeams.push_back(seam);
    }

    WeldSeams = std::move(filteredSeams);

    // 重新编号
    for (size_t i = 0; i < WeldSeams.size(); i++)
        WeldSeams[i].Id = (int)i + 1;
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
