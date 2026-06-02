# 核心模块

### 应用入口

`OCC_MFC/OCC_MFC.cpp` 定义 `COCCMFCApp`，负责 MFC 初始化、OLE 初始化、文档模板注册，以及创建 `COCCMFCDoc`、`CMainFrame`、`COCCMFCView` 之间的 SDI 关系。
### 主框架
`OCC_MFC/MainFrm.cpp` 定义 `CMainFrame`，负责：

- 菜单栏、工具栏、状态栏创建
- 文件视图、类视图、输出窗口、属性窗口等停靠面板创建
- 右侧"机器人控制面板"停靠面板创建
- MFC 外观主题切换

机器人控制面板通过 `m_wndControlPane` 挂载。
### OpenCascade 视图

`OCC_MFC/OCC_MFCView.cpp` 定义 `COCCMFCView`，负责：

- 创建 `Aspect_DisplayConnection`
- 创建 `OpenGl_GraphicDriver`
- 创建 `V3d_Viewer`
- 创建 `V3d_View`
- 创建 `AIS_InteractiveContext`
- 使用 `WNT_Window` 将 OCCT 渲染窗口绑定到 MFC `CView`
- 鼠标左键旋转、右键平移、滚轮缩放。
- 左键短点击（< 5px 移动阈值）选中模型，OCCT 自动高亮（预高亮为蓝色边框）
- 显示机器人模型与用户导入模型

窗口尺寸变化时，需要同步触发 OCCT viewport resize，否则 MFC 窗口最大化或停靠布局变化后可能出现渲染区域不随客户区扩展的问题。
### 文档类与机器人逻辑

`OCC_MFC/OCC_MFCDoc.cpp` 是当前业务逻辑最集中的文件，负责：
- 保存机器人 7 个连杆对象：`m_RobotLinks[7]`
- 保存当前 6 个关节角：`m_JointAngles[6]`
- 保存用户导入的独立模型：`m_ImportedModel`
- 加载 Fanuc M10iD12 STL 连杆模型


- 根据关节角更新机器人显示姿态
- 正向运动学计算 TCP 的 `XYZWPR`
- 根据 TCP 输入执行逆向运动学求解。
- 通知视图刷新

当前 Fanuc 几何参数在 `OCC_MFCDoc.cpp` 顶部以静态常量形式定义，单位主要为 mm；显示模型更新中也存在 mm 与 m 的缩放处理。
### 机器人控制面板
`OCC_MFC/CRobotControlDlg.cpp` 定义 `CRobotControlDlg`，负责：

- 6 个关节滑块
- 6 个关节角编辑框
- TCP 位置与姿态输入框
- 关节输入和滑块联动
- TCP 输入触发逆解
- 用户选择本地模型文件并导入。
为避免 UI 回填触发递归更新，使用 `m_bIsUpdating` 作为联动保护标志。
### 控制面板容器

`OCC_MFC/CControlPane.cpp` 定义 `CControlPane`，这是一个 `CDockablePane`，内部嵌入 `CRobotControlDlg`。面板大小变化时，会让内部对话框铺满面板客户区。
