# 第二阶段文件整理与路径迁移方案（待批准）

日期：2026-10-16。分析基线：`main`，提交 `9ffcbda`。

本文件只提出迁移方案。编写时没有移动源码、配置、数据或资源，没有修改业务代码，
没有提交、推送或操作 RoboDK。后续执行前应重新核对基线与工作区。

## 1. 当前判断

第一阶段把文档从根目录移到了 `docs/`，保留平级 Python 导入，风险较低。
当前根目录已有 28 个 Python 模块；`main.py` 共 2,751 行，承担界面初始化、工作站、
模型处理、路径、速度、标定和机器人接口编排。还新增了图标、多语言、UI 偏好与多机启动。

第二阶段建议采用“先统一路径，再迁移包，最后拆分主程序”的顺序。
仅创建功能文件夹并拖动文件，会同时改变 Python 导入名、资源查找位置、测试目标和启动顺序。

本轮静态检查还发现：

- `ui_panels.py` 与 `ui_theme.py` 有双向的函数内导入；不能机械改成顶部导入。
- `smoothing/` 当前仅有 10 个 `.pyc`，没有 `.py` 源码，且没有已跟踪文件。
  它不能作为当前分支已实现 smoothing 的依据，也不能直接作为可迁移模块。
- `artifacts/`、`tmp/`、`docs/reports/`、真实标定 JSON 和本机配置均被忽略。
  “本机存在”和“新克隆可获得”必须分开核对。
- 第一阶段聊天中的旧分支、测试数量和功能范围不能直接作为本次迁移基线。

## 2. 建议目录

先采用根目录下的 `scanning_app/` Python 包；暂不引入需要安装的 `src/` 布局。
这样可继续使用现有 Conda 环境和双击启动器，不增加依赖安装步骤。

```text
Software_optimizing/
├─ start_software.cmd                 # 保留现有用户入口
├─ run_integrated_app.ps1             # 保留本机配置入口
├─ app_bootstrap.py                   # 过渡期保留的轻量启动入口
├─ main.py                            # 过渡期兼容入口，不再持有第二份状态
├─ scanning_app/
│  ├─ __init__.py                     # 无 OCC、Qt、窗口创建副作用
│  ├─ __main__.py                     # 可选统一 python -m 入口
│  ├─ config.py
│  ├─ i18n.py
│  ├─ app/                           # 主程序、共享状态、任务调度
│  ├─ cad/                           # CAD 输入、分割、视点、碰撞
│  ├─ planning/                      # 路径排序、速度、可达性修复
│  ├─ robot/                         # RoboDK 与位姿坐标变换
│  ├─ persistence/                   # 工作站存档、CSV
│  ├─ ui/                            # 面板、主题、渲染、交互
│  └─ runtime/                       # 环境与统一路径服务
├─ assets/                           # 第二阶段保留根位置
├─ locales/                          # 第二阶段保留根位置
├─ calibration/                      # 示例跟踪，真实配置留本地
├─ config/                           # 本地环境配置与共享示例
├─ scripts/                          # 命令行验证与 GUI 自测
├─ tests/
├─ docs/
├─ artifacts/                        # 实验数据和证据，保持历史位置
├─ diagnostics/                      # 本机诊断
├─ archive/                          # 历史资料
└─ tmp/                              # 临时工作区
```

`config.py` 是算法默认值模块，`config/` 是机器设置目录，两者用途不同。
迁移后前者成为 `scanning_app.config`，不在 `config/` 中创建 Python 同名包。
`state.py` 包含 OCC 显示句柄，不能仅凭 dataclass 形式就归为纯算法数据层。

## 3. 现有源码迁移映射

以下路径均相对仓库根目录；这是目标清单，不表示已执行。

| 当前文件 | 建议目标 | 注意事项 |
| --- | --- | --- |
| `app_bootstrap.py` | 根目录保留 | 先保持运行环境准备顺序与参数兼容 |
| `main.py` | `scanning_app/app/main.py` | 最后迁移；根文件只保留兼容入口 |
| `state.py` | `scanning_app/app/state.py` | 全项目只加载一份状态类型 |
| `workers.py` | `scanning_app/app/workers.py` | 保持 Qt 信号与 OCC 线程策略 |
| `config.py` | `scanning_app/config.py` | CAD、规划、UI 共用 |
| `i18n.py` | `scanning_app/i18n.py` | 先改语言目录解析 |
| `runtime_environment.py` | `scanning_app/runtime/environment.py` | 必须早于 OCC/Qt 业务模块导入 |
| 新增路径服务 | `scanning_app/runtime/paths.py` | 标准库实现；不创建窗口、不连接外部服务 |
| `cad_io.py` | `scanning_app/cad/io.py` | 保持 STEP/IGES 读取契约 |
| `geometry.py` | `scanning_app/cad/geometry.py` | 依赖分割；包含显示相关工具，暂不强拆 |
| `surface_segmentation.py` | `scanning_app/cad/surface_segmentation.py` | 保持 OCC 版本兼容分支 |
| `viewpoints.py` | `scanning_app/cad/viewpoints.py` | 保持视点来源、姿态与显示行为 |
| `collision.py` | `scanning_app/cad/collision.py` | 保持现有碰撞接口 |
| `planning.py` | `scanning_app/planning/ordering.py` | 仅迁移，不更换求解器 |
| `speed_planning_core.py` | `scanning_app/planning/speed.py` | 保持单位、字段和区段语义 |
| `reachability_repair.py` | `scanning_app/planning/reachability.py` | 依赖 CAD 视点、共享状态与路径排序 |
| `pose_transform.py` | `scanning_app/robot/pose_transform.py` | 无窗口初始化副作用 |
| `robodk_bridge.py` | `scanning_app/robot/robodk_bridge.py` | 延迟加载 API，不在包导入时连接站点 |
| `robodk_discovery.py` | `scanning_app/robot/discovery.py` | 启动预检依赖；包初始化须足够轻 |
| `workstation_io.py` | `scanning_app/persistence/workstation.py` | 依赖 OCC，不是纯 JSON 工具 |
| `export_utils.py` | `scanning_app/persistence/csv_export.py` | 保持 Pose、Pose+Speed 两套输出 |
| `ui_panels.py` | `scanning_app/ui/panels.py` | 保留必要的延迟导入 |
| `ui_theme.py` | `scanning_app/ui/theme.py` | 先替换图标定位 |
| `ui_workspace.py` | `scanning_app/ui/workspace.py` | 保持 QSettings 身份和键名 |
| `speed_planning_ui.py` | `scanning_app/ui/speed_dialogs.py` | 对话框输出字段保持不变 |
| `model_drop.py` | `scanning_app/ui/model_drop.py` | 保持工作站/CAD 两种拖放分派 |
| `renderer.py` | `scanning_app/ui/renderer.py` | 保持统一渲染链与原生句柄生命周期 |
| `scene_style.py` | `scanning_app/ui/scene_style.py` | 仍视为依赖 OCC 的显示模块 |
| `local_frame_style.py` | `scanning_app/ui/local_frame_style.py` | 保持设置字段和归一化逻辑 |

上述按“职责”归类，不声称已经完成架构解耦。例如可达性修复仍依赖 CAD 数据，
AppState 仍持有显示对象；先确保迁移等价，再考虑拆接口。

## 4. 分批执行与每批验收

### 2A：迁移准备和路径统一

1. 记录实际分支、提交、文件清单和现有测试结果；保留工作区已有改动。
2. 新建轻量包与路径服务，源码先留原位。
3. 用统一入口替换分散的资源路径计算；保留资源、标定和本地配置的现有位置。
4. 对所有拟移动文件记录源路径、目标路径、哈希、Git 跟踪状态及引用。
5. 检查 scripts 的 `PYTHONOCC_PREFIX` 与启动器的 `SCANNING_APP_OCC_PREFIX` 差异；
   在本批明确兼容方式，不额外创建第三套环境注入流程。

验收：从项目根、其他目录、带空格/中文路径副本启动预检；图标、语言和默认标定路径一致；
本地配置仍命中忽略规则。对用户输入的相对路径，应以调用时目录解析后再切换工作目录。

### 2B：算法、类型和基础模块迁移

按依赖顺序迁移 config、状态类型、速度、坐标变换、环境与发现模块，再迁移 CAD、
路径排序、修复和持久化模块。每个提交必须同步改完所有相关导入、测试和命令脚本。

包内统一使用 `scanning_app.*` 的明确导入路径；不靠全局设置 `PYTHONPATH` 或反复插入
源码子目录维持运行。旧模块若确需过渡兼容，应指向同一规范模块，不能复制实现。
普通 `from new_module import *` 不能保证私有符号、可变模块状态或 mock 目标兼容。

验收：导入对象身份一致；纯算法测试不需要导入 GUI；工作站往返、CSV、标定和 mock
RoboDK 测试通过。迁移不得改变状态失效规则或把模拟验证标志提升为物理验证。

### 2C：UI 和入口迁移

先迁移 UI 辅助模块，最后移动主程序；脚本和源码定位测试一起更新。
保留 `start_software.cmd`、PowerShell 参数和根 `app_bootstrap.py` 的用户用法。
兼容期根 `main.py` 与新主模块共用同一状态，不能触发两次 viewer 创建。

验收：Qt5/Qt6 各自的入口检查、语言切换、图标、Ribbon、布局恢复、模型拖放、
导入→分割→中心/法向→视点→路径、工作站保存恢复通过；实际图形环境验证 OCC 画布。
无图形环境时明确保留人工验收项，不能以 offscreen 单测代替选面/显示验收。

### 2D：主程序职责拆分（单独审批、单独提交）

包迁移稳定后，再把窗口创建从模块顶层移到显式的应用工厂或启动函数。
先抽工作站和模型操作，再抽路径、速度、标定及机器人命令编排。
采用显式传入 state、display、task runner 的应用上下文，逐步减少跨模块全局变量。
不要同时重写算法、存档格式和 UI。

验收：导入主模块不自动创建窗口；每次启动只创建一个 QApplication/viewer/状态实例；
后台结果仍由主线程提交；关闭、取消和失败恢复行为不退化。

2D 属于结构重构，其风险和工作量高于目录移动。建议先批准并完成 2A–2C，
观察稳定后再确认 2D 的具体拆分边界。

## 5. 统一路径约定

| 路径类别 | 第二阶段约定 |
| --- | --- |
| 内置只读资源 | 用统一资源入口定位根 `assets/`、`locales/`；不相对任意业务模块 |
| 本机运行环境 | 继续读取根 `config/local_environment.ps1`；示例跟踪，本机文件忽略 |
| 标定 | 默认根 `calibration/`；示例与真实文件区分，不因迁移改 validated 等标志 |
| 用户 CAD/工作站/导出 | 使用用户选择的完整路径；命令参数先解析为绝对路径 |
| UI 偏好 | 继续使用既有 QSettings UserScope、组织名、应用名和键 |
| 新实验产物 | 统一 `artifacts/<任务>/<run_id>/`，由任务显式指定，避免落到源码目录 |
| 新临时文件 | 系统临时目录或显式的 `tmp/<任务>/`，不与正式导出混用 |
| 历史证据 | 保持原路径和内容；索引记录引用，不批量重写历史绝对路径或校验值 |

资源解析器应支持显式根目录注入，便于复制目录和测试；源码布局可提供确定的默认根。
找不到必需资源时给出可定位的错误，不通过遍历任意父目录“猜”项目根。
未来若采用安装包，再设计 package data / importlib.resources，不在本轮强制改变分发方式。

## 6. 路径移动风险清单

“已确认”表示本次可由源码和路径计算直接验证；“条件风险”表示在特定移动/运行方式下发生。

| 风险 | 当前证据 | 可能后果 | 控制与验收 |
| --- | --- | --- | --- |
| 平级导入失效（高，已确认依赖） | 根模块与 tests 广泛使用 `from config/state/... import ...` | ModuleNotFoundError，或误导入其他同名模块 | 所有消费端同步迁移；使用明确包名；从其他目录启动验证 |
| 模块被加载两次（高，条件风险） | i18n、main 有共享状态；workstation 使用类型判断 | 两份语言管理器/状态；类身份不一致；mock 打错模块 | 规范导入唯一；兼容层不复制实现；比较对象身份 |
| 语言包失效（高，已确认路径） | `i18n.py:35`、`ui_workspace.py:29` | 前者无法初始化语言服务；后者异常被吞后搜索别名悄然缺失 | 统一 locale 路径；两种语言及命令搜索验收 |
| 图标悄然消失（中，已确认路径） | `ui_theme.py:6`、`:132`–`:133` | 找不到文件时返回空 QIcon，UI 仍可打开 | 图标目录一致；核对关键动作图标，不只看启动成功 |
| 标定读写落错目录（高，已确认路径） | `main.py:1719` 用相对目录；`:1763` 用 main 的父目录 | 默认选择目录错误，保存到包子目录或创建重复配置 | 统一 calibration 根；原文件哈希与状态不变；实测保存路径 |
| 环境注入顺序错误（高，条件风险） | `configure_runtime()` 在 OCC 已导入时拒绝继续；包内模块依赖 OCC | DLL 加载失败、Qt 插件冲突、原生异常 | __init__ 保持轻量；先环境后 OCC；保持 DLL 注册句柄生命周期 |
| 启动器定位失效（高，已确认路径） | PS 用 `$PSScriptRoot`；CMD 用 `%~dp0`；bootstrap 用自身目录 | 双击失败、读取不到本地配置或 chdir 到包目录 | 启动入口留根；调整内部转发；双击与绝对路径调用都验收 |
| UI 循环导入显化（中，条件风险） | panels:320 与 theme:203 的函数内互调 | 顶部导入出现 partially initialized module | 保持延迟导入；解环放到后续职责重构 |
| 测试假失败或假通过（高，已确认依赖） | 源码 AST 测试固定读取根 main；mock 固定 `workstation_io._write_shape` | 测试读取到兼容壳；mock 失效；正确行为没被测到 | 更新源文件定位和 patch 目标；拆分测试 helper；检查 skip 原因 |
| 复制项目后旧路径仍在（中，条件风险） | 工作站保存 source/calibration 路径，UI 最近列表保存绝对路径 | 重开最近文件失败；重存时无法再嵌入外部原始 CAD | 不篡改历史来源；验证存档无原 CAD 仍能读取；明确重定位流程 |
| 忽略规则失效（高，条件风险） | `.gitignore` 对 calibration/config/artifacts/docs/reports 有路径规则 | 移到新目录后真实标定、本机设置或大文件意外入库 | 每批检查 git check-ignore 与 status；只按清单暂存 |
| 实验复现链断开（中，条件风险） | artifacts 中存在 CSV/JSON、脚本、RDK 与历史绝对引用 | 结果与原配置关联丢失 | 历史产物先留原位；需要迁移时制作映射和哈希清单 |
| 外部脚本依赖遗漏（中，已确认） | `planning.py:66` 内导入 ABC；CSV 测试寻找仓库外 importer | 新克隆算法缺失；外部测试跳过被误认全部覆盖 | 盘点可选依赖；本地契约测试与外部集成测试分别报告 |
| 缓存被误归为源码（中，已确认） | smoothing 只有 pyc，0 个 py | 假装恢复功能，复制不匹配版本缓存 | 从 Git/原始源码找证据；此轮不删除、不反编译、不当作可交付源码 |

本次只做路径计算探针：若按目标目录移动但保留原公式，语言包、UI 搜索语言包、应用图标、
标定目录四个结果均指向当前不存在的位置。没有实际移动文件，也未据此声称已复现运行崩溃。

### 工作站兼容性的特殊判断

`.swstation` 使用 JSON、BREP 和显式记录，不使用 pickle，因此 Python 模块改名本身
不要求升级存档格式。应保持内部成员名、字段、单位和版本不变。
但内存类型判断仍受重复导入影响；需要用迁移前保存的实际工作站验证新代码读取，
再验证新代码保存后可被旧基线读取。存档可恢复模型不代表其中的原始外部路径仍有效。

### 第一阶段文档移动遗留风险

源码不依赖这些文档的原路径，所以通常不会影响程序执行。
可能受影响的是书签、聊天/工单链接、相对图片路径，以及说明里的命令执行目录。
后续只修活动文档链接，并注明从仓库根执行命令；历史日志保留原文。
将报告移入当前被忽略的 `docs/reports/` 会使新克隆看不到它们；若选定审查结论需要共享，
应另建可跟踪的摘要并核对敏感路径，不依赖本机报告目录作为唯一说明。

## 7. 验证和回滚

每批采用独立提交，提交应同时包含移动、导入修正、资源引用修正和必要测试更新。
不把整个项目拆到一半再统一修复。以 Git 按批回退，已产生用户数据不随源码回退删除。
当前 main 相对 origin/main 领先 7 个提交；执行时应重新核对并保护这些已有提交。

至少保留以下可比较证据：

1. 迁移前后的测试通过数、失败数、跳过数与环境摘要；本轮没有重跑完整测试。
2. 纯算法无 GUI 导入；正常启动和启动预检；Qt5/Qt6 支持边界。
3. 用户实际 STEP/IGES 的同参数分割、面积/诊断、视点姿态、路径与速度输出对比。
4. 用同一种显示模式对比；已知 GUI 网格缓存可使分区数与无界面不同，不能跨模式要求数量相等。
5. 迁移前工作站加载与重存；标定文件哈希；CSV header 和字段语义。
6. 图标、语言、命令搜索、UI 偏好恢复、工作站/CAD 拖放。
7. mock RoboDK 指令与失败恢复；若要复验真实站点，先做读取核对，再按单独授权的验证流程执行。
8. 根目录/其他目录/空格与中文路径/复制到另一目录的运行矩阵。
9. Git 差异检查、忽略规则、无意外大文件/本机配置，所有活动 Markdown 链接可解析。

发现行为变化应暂停该批，区分“导入/资源迁移问题”与“原有业务缺陷”。
本机既有 [代码审查报告](../reports/code_review_20261016/代码审查报告.md)
列出的速度区段、姿态角加速度、碰撞与存档问题应另立修复提交；本方案不把它们当作已修复。
本机 [运行环境验证记录](../reports/segmentation_merge_audit_20261016/修复与验证.md)
也是历史证据，不替代迁移后的实测。

## 8. 建议批准范围

建议批准 **2A–2C**：路径统一、现有模块迁入包、兼容入口及消费端同步更新，按批验证。
暂时保留根 assets/locales/config/calibration/scripts/tests 和历史实验数据位置。
2D 主程序职责拆分在包迁移稳定后单独确认。

本次方案未执行上述任何迁移。实际实施不增加 Python 依赖、不删除历史资料，
不修改算法契约、存档格式或机器人验证标志。
