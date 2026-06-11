# Changelog

## Version History

| Version | Commit ID | Name | Date | Description |
|----------|----------|----------|----------|----------|
| 1.0.0-MDtest | 1466229 | MDtest | 2026-06-04 | 测试提交流程，验证版本记录更�?|

## 最近修复记�?
- 2026-05-25: 完全替换焊缝识别算法。基于专�?CN 120339259 A 实现新的 WeldExtractor 类，替换旧的 FindEdge2 类。新算法5步法�?1)拓扑映射—提取子模型/�?边映射；(2)初始焊缝提取—OBB相交+法线反向筛选，包含/相交分类处理�?3)截断检测—IntCurvesFace检测截断；(4)内部截面检测—法线一致性检查；(5)去重输出。新增WeldSeam字段：形成面、截断面、截面计数、子模型索引�?
- 2026-05-18: 改进焊缝识别策略。`FindEdge2::S_contact()` 从仅依赖�?板接触边改为通用空间接触根部检测：遍历实体面边界，要求候选边的起�?中点/终点贴合另一个实体的有限支撑面，并通过平面法向夹角过滤平行面和外露凸边；新增候选边长度过滤、端点去重和根部姿态估计。`FindInternalCornerSeams()` 进一步补充同一实体内的平面-平面凹折边识别，用顶部外露边过滤和水平外包围盒边过滤压制上沿、底板外轮廓等非焊接凸边。该策略用于识别槽钢/方管/加强筋与底板、立板之间未共享拓扑 Edge 的真实内角焊缝�?
- 2026-05-16: 新增焊缝识别 + 3D 高亮显示。`FindEdge2` 类集成在 `OCC_MFCDoc.h` 中。`ComputeWeldSeams()` 从焊缝起�?终点构建 `BRepBuilderAPI_MakeEdge` 边线几何。`DisplayWeldCurves()` �?3D 视图中显示红色粗线（线宽 3），每个焊缝起点处放置彩色标记球�? 色轮换）+ `AIS_TextLabel` 白色编号标签。`CPageWeld` 传�?`WeldSeam` 数据到视图�?
- 2026-05-16: 最终修�?OCCT 视图黑色区域 bug（窗口类样式 + AdjustDockingLayout）。`COCCMFCView::PreCreateWindow` 注册窗口类为 `CS_HREDRAW | CS_VREDRAW | CS_OWNDC`，确保尺寸变化时整个客户区失效重绘。`OnPaint` 内联 `DoResize()+MustBeResized()+Redraw()`。`OnSize` 执行 `DoResize()+MustBeResized()+Invalidate(FALSE)`。`CMainFrame` 重写 `AdjustDockingLayout()` 通过 `DYNAMIC_DOWNCAST` 获取视图调用 `ResizeOCCView()`。`LoadFrame` 末尾 `PostMessage(WM_APP+100)` 触发 `OnAfterLoadLayout`�?
- 2026-05-16: 焊接参数页面 UI：`IDD_PAGE_WELD` �?`.rc` 中定义控件（PUSHBUTTON、LTEXT、SysTabControl32、SysListView32），`CPageWeld` 通过 `DDX_Control` 绑定。Tab 下移�?y=62 避免遮挡标题。列表固定显�?10 行高度（精确计算表头+行高×10），超出部分通过垂直滚动条访问。起�?终点列改为单�?3 值坐标（X,Y,Z）并加宽列宽�?240px 完整显示。填�?20 行示例数据（重复�?3 组模式）验证垂直滚动�?
- 2026-05-14: 添加机器人控制面板分页切换功能。在 `IDD_ROBOT_CONTROL_PANEL` 对话框中添加两个页面切换按钮（`IDC_BUTTON_PAGE_ROBOT`/`IDC_BUTTON_PAGE_WELD`），`SwitchToPage` 使用 `GetWindow(GW_CHILD)` 枚举方式覆盖所有子控件（包�?`IDC_STATIC` 静态文本）。修�?`.rc` 文件�?`VS_VERSION_INFO` 被注入对话框控制行的问题；移�?`resource.h` �?UTF-8 BOM；修�?`.rc` 中缺失的 `END` 语句�?

- 2026-05-13：修复控制面板导�?STEP/STP 文件失败问题。`COCCMFCDoc::LoadModelFromFile` �?STEP/IGES 导入前会自动初始�?OCCT 7.8 需要的 `CSF_STEPDefaults`、`CSF_IGESDefaults`、`CSF_PluginDefaults` 等资源环境变量，并使�?UTF-8 路径传给 OCCT Reader，避免中文路径或资源变量缺失导致 `IFSelect_RetError`�?
- 2026-05-13：修复导入三维模型后原舞台被重置的问题。`COCCMFCView::OnUpdate` 对导入刷新不再执�?`RemoveAll()` �?`FitAll()`，导入后保留已有机器人显示、已显示对象和当前视角�?
- 2026-05-13：修复导入模型与机器人比例不一致的问题。导入模型使�?`MakeImportedModelTransform` 统一执行本体 `0.001` 缩放和位�?mm->m 平移，不修改机器人连�?STL 加载比例�?DH 参数比例�?
