在动手修改代码前，必须先加载 skills/code_guidelines.md 中的 Code Guidelines 约束。

- # OCC_MFC 项目说明

  ## 项目简介

  本项目为基于 OpenCascade（OCCT）与 MFC 的工业机器人焊接离线编程与数字孪生仿真平台。

  项目主要涉及：

  - STEP 模型加载与显示
  - CAD 几何分析
  - 焊缝识别与提取
  - 工业机器人运动学计算
  - 手眼标定
  - 焊接轨迹生成
  - OCCT 三维可视化
  - 数字孪生仿真

  ------

  ## 文档导航

  项目文档按以下结构组织：

  ### 项目说明

  - README.md
    - 项目概览
    - 技术栈与依赖
    - 目录结构
    - 资源与菜单说明

  ------

  ### 项目文档

  - docs/architecture.md
    - 系统架构
    - 模块关系
    - 数据流
    - 核心类关系
  - docs/current_features.md
    - 当前已实现功能
  - docs/roadmap.md
    - 开发计划
    - 后续开发方向
  - docs/changelog.md
    - 修改记录
    - 版本记录
    - Git 提交历史
  - docs/development_rules.md
    - 开发规范
    - 文件编码规范
    - 注释规范
    - Git 提交规范
    - 开发注意事项

  ------

  ### 技能库

  - skills/

  用于存放项目经验、调试经验和可复用解决方案。

  例如：

  - OCCT 显示问题
  - 焊缝识别经验
  - 手眼标定经验
  - FCL 碰撞检测经验
  - 路径规划经验

  ------

  ## Agent 工作流程

  收到任务后：

  1. 阅读本文件（AGENTS.md）
  2. 根据任务类型定位对应文档
  3. 阅读 docs/development_rules.md
  4. 分析影响范围
  5. 实施修改
  6. 同步更新相关文档
  7. 输出修改报告

  ------

  ## 文档同步规则

  根据修改内容更新对应文档：

  - 功能变化
    → docs/current_features.md
  - 架构变化
    → docs/architecture.md
  - 开发计划变化
    → docs/roadmap.md
  - 修改记录或版本记录
    → docs/changelog.md
  - 开发规范变化
    → docs/development_rules.md
  - 项目经验沉淀
    → skills/

  ------

  ## AGENTS.md 更新限制

  AGENTS.md 仅用于：

  - 项目简介
  - 文档导航
  - 开发流程
  - 文档同步规则

  禁止将以下内容直接写入 AGENTS.md：

  - 功能说明
  - 架构细节
  - 开发记录
  - 版本记录
  - 项目经验
  - 调试经验

  相关内容应写入对应文档。
