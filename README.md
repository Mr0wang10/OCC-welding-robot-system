# OCC_MFC 项目说明

## 项目概览

这是一个基于 Visual Studio / MFC / C++ 的桌面应用项目，主目标是将 OpenCascade 三维显示能力嵌入 MFC 主界面，用于机器人三维模型显示、关节姿态控制、TCP 位姿显示与逆解输入。
项目采用 MFC 单文档 (SDI) 架构。
- 应用入口：`OCC_MFC/OCC_MFC.cpp`
- 主框架窗口：`OCC_MFC/MainFrm.cpp`
- 文档类：`OCC_MFC/OCC_MFCDoc.cpp`
- OpenCascade 视图类：`OCC_MFC/OCC_MFCView.cpp`
- 机器人控制面板：`OCC_MFC/CRobotControlDlg.cpp`
- 控制面板停靠容器：`OCC_MFC/CControlPane.cpp`

解决方案文件：`OCC_MFC.sln`，主工程文件：`OCC_MFC/OCC_MFC.vcxproj`。
## 技术栈与依赖
项目使用了：
- MFC 动态库
- C++17
- Visual Studio v143 工具集
- OpenCascade 7.8.0 相关库
- VTK、TBB、OpenVR、FreeImage、FreeType 等 OpenCascade 第三方依赖
- vcpkg 依赖：Assimp、FCL、OMPL、Boost、Octomap、TBB 等
- 外部运动学库：`Kine6Dof.lib` / `Kine6Dof.dll`

工程配置中存在本机绝对路径依赖，例如：
- `D:\OCCT780\...`
- `D:\dev\vcpkg\...`

如果在其他机器构建，优先检查 `OCC_MFC.vcxproj` 中的 `AdditionalIncludeDirectories`、`AdditionalLibraryDirectories` 和 `AdditionalDependencies`。
## 目录结构要点

- `OCC_MFC/`：主源码与资源目录
- `OCC_MFC/res/`：MFC 图标、位图、工具栏资源
- `OCC_MFC/Rbt3DModelLib/`：机器人三维模型库
- `OCC_MFC/Rbt3DModelLib/Fanuc/M10iD12/`：当前主要使用的 Fanuc M10iD12 模型
- `x64/` 和 `OCC_MFC/x64/`：构建产物目录，通常不应手动改动源码需求之外的产物

模型库中已有多个品牌或型号目录，例如 ABB、EFORT、Fanuc、GSK、Kawasaki、STEP 等。
## 资源与菜单
资源文件：`OCC_MFC/OCCMFC.rc`，资源 ID 定义在 `OCC_MFC/resource.h`。
资源文件是 Visual Studio 生成的本地编码文件，不一定是 UTF-8。修改 `.rc` 时要注意保持原编码，避免中文资源乱码。
`resource.h` 中已经存在多个机器人品牌或型号的菜单 ID，例如：

- `ID_FANUC_M10ID12`
- `ID_ABB_ABB1410`
- `ID_EFORT_ER20D`
- `ID_GSK_GSKRB08`
- `ID_GSK_GSKRB20`
- `ID_KAWASAKI_BA006N`
- `ID_STEP_STEPSA1400`
- `ID_STEP_STEPSA1800`

目前实际接入视图消息处理的主要是 `ID_FANUC_M10ID12`。
