# UI 改善与路径平滑模块 — 详细实施计划

> 创建日期：2026-05-20
> 状态：待评审

---

## 一、UI 改善计划

### 1.1 当前 UI 结构

```
┌─────────────────────────────────────────────────┐
│  菜单栏 (File | Model Processing | Collision | View | Path Planning | Export | Help)
├─────────────────────────────────────────────────┤
│                                                 │
│           OCC 3D Viewer (中央区域)               │
│                                                 │
│              ┌──────────┐  ┌──────────┐         │
│              │ Workflow  │  │  Layers  │         │
│              │  (右侧)   │  │  (右侧)  │         │
│              └──────────┘  └──────────┘         │
│                                                 │
│                              状态栏 (底部)       │
└─────────────────────────────────────────────────┘
```

**现状问题**：
- Workflow 面板只显示文本状态，无交互按钮
- 菜单项始终可用，用户可能在不恰当的步骤执行操作
- 无日志面板，所有信息通过 `print()` 输出到控制台或弹窗
- 路径规划有 4 种算法，但没有统一的参数配置界面

### 1.2 目标 UI 结构

```
┌─────────────────────────────────────────────────────────────┐
│  菜单栏                                                      │
├──────────────┬───────────────────────────────┬───────────────┤
│              │                               │               │
│  Workflow    │       OCC 3D Viewer           │   Layers      │
│  Panel       │                               │   Panel       │
│  (左侧)      │                               │   (右侧)      │
│  ┌────────┐  │                               │  ┌─────────┐  │
│  │[Import]│  │                               │  │☐ Model  │  │
│  │[Select]│  │                               │  │☐ Centers│  │
│  │[Segment│  │                               │  │☐ Normals│  │
│  │[Centers│  │                               │  │☐ Views  │  │
│  │[Views] │  │                               │  │☐ Path   │  │
│  │[Filter]│  │                               │  │☐ OBB    │  │
│  │[Sensor]│  │                               │  │☐ Sensor │  │
│  │[OBB]   │  │                               │  └─────────┘  │
│  │[Collis]│  │                               │               │
│  │[Path]  │  │                               │               │
│  │[Export]│  │                               │               │
│  └────────┘  │                               │               │
│              │                               │               │
├──────────────┴───────────────────────────────┴───────────────┤
│  日志面板 (可折叠，显示操作历史和错误信息)                       │
├─────────────────────────────────────────────────────────────┤
│  状态栏                                                       │
└─────────────────────────────────────────────────────────────┘
```

### 1.3 实施任务清单

#### 任务 1：Workflow 面板增加步骤按钮

**文件**：`ui_panels.py`

**设计**：
- 将当前的纯文本 `QLabel` 替换为 `QWidget + QVBoxLayout`
- 每个步骤是一个 `QPushButton`，显示格式：`[状态图标] 步骤名称`
- 状态图标：`⬜ Not ready` / `🟢 Ready` / `🔄 Running` / `✅ Done` / `❌ Error`
- 按钮根据 `AppState` 自动启用/禁用
- 点击按钮触发对应的操作回调

**实现步骤**：
1. 修改 `create_workflow_panel()` 创建按钮布局
2. 每个按钮连接到对应的回调函数（通过参数传入）
3. 修改 `update_workflow_status()` 更新按钮状态和启用/禁用
4. 添加 `WorkflowButtonConfig` 数据类，定义每个按钮的 key、label、callback

**代码结构**：
```python
def create_workflow_panel(button_callbacks):
    """button_callbacks: dict mapping step_key -> callable"""
    panel = QWidget()
    layout = QVBoxLayout(panel)
    for step in DEFAULT_WORKFLOW_STEPS:
        btn = QPushButton(step.label)
        btn.setEnabled(False)  # 初始禁用
        btn.clicked.connect(lambda checked, k=step.key: button_callbacks[k]())
        layout.addWidget(btn)
        _workflow_buttons[step.key] = btn
    layout.addStretch()
    ...
```

**启用/禁用逻辑**：
| 步骤 | 启用条件 |
|------|----------|
| Import Model | 始终可用 |
| Select Face | model_loaded |
| Segment | model_loaded |
| Compute Centers | segmented |
| Generate Viewpoints | centers_computed |
| Filter Viewpoints | viewpoints_generated |
| Create Sensor Volumes | viewpoints_generated |
| Generate OBB | sensor_created |
| Collision Check | obb_generated |
| Path Planning | optimal_viewpoints 非空 |
| Export CSV | optimal_path 非空 |

#### 任务 2：菜单自动启用/禁用

**文件**：`main.py`

**设计**：
- 菜单项根据 `AppState` 自动启用/禁用
- 在每个操作完成后调用 `_update_menu_state()`
- `_update_menu_state()` 遍历所有菜单项，根据状态设置 `setEnabled()`

**实现步骤**：
1. 在 `run()` 中保存菜单项引用到字典
2. 定义 `_update_menu_state()` 函数
3. 在每个操作回调末尾调用 `_update_menu_state()`

#### 任务 3：底部日志面板

**文件**：`ui_panels.py`（新增 `create_log_panel()`），`main.py`

**设计**：
- `QDockWidget` 放在底部，包含 `QTextEdit`（只读）
- 显示时间戳 + 操作信息
- 保留最近 500 行，超出自动截断
- 可折叠（默认展开）

**实现步骤**：
1. 在 `ui_panels.py` 中添加 `create_log_panel()` 和 `append_log(message)` 函数
2. 创建全局 `_log_text_edit` 变量
3. 在 `main.py` 的 `run()` 中调用 `create_log_panel()`
4. 将关键的 `print()` 替换为 `append_log()`

**日志级别**：
```python
LOG_INFO = "INFO"
LOG_WARN = "WARN"
LOG_ERROR = "ERROR"

def append_log(message, level=LOG_INFO):
    timestamp = time.strftime("%H:%M:%S")
    _log_text_edit.append("[{}] {}: {}".format(timestamp, level, message))
```

#### 任务 4：路径规划统一对话框

**文件**：`ui_panels.py`（新增 `get_path_planning_dialog()`）

**设计**：
- 统一对话框，包含算法选择和参数配置
- 算法选择：`QComboBox`（Sequential / Greedy / ABC / MSCGA）
- 根据选择的算法动态显示/隐藏参数区域
- ABC 参数：food_number, limit, maximum_evaluation
- MSCGA 参数：pop_size, generations, crossover_rate, mutation_rate

**UI 布局**：
```
┌─ Path Planning ──────────────────────────┐
│                                          │
│  Algorithm: [Sequential ▼]               │
│                                          │
│  ┌─ Advanced Parameters ──────────────┐  │
│  │ (根据算法动态变化)                   │  │
│  │ Population Size: [200    ]          │  │
│  │ Generations:     [1500   ]          │  │
│  │ Crossover Rate:  [0.9    ]          │  │
│  │ Mutation Rate:   [0.08   ]          │  │
│  └────────────────────────────────────┘  │
│                                          │
│  ☐ Smooth path after planning            │
│                                          │
│         [ OK ]    [ Cancel ]             │
└──────────────────────────────────────────┘
```

**返回值**：
```python
@dataclass
class PathPlanningResult:
    algorithm: str  # "sequential" / "greedy" / "abc" / "mscga"
    smooth: bool = False
    abc_config: ABCConfig = None
    mscga_config: MSCGAConfig = None
```

#### 任务 5：UI 视觉优化（可选）

- 统一按钮样式和间距
- 添加工具栏图标（使用 Qt 内置图标或简单 Unicode 字符）
- 状态栏显示当前步骤和进度

---

## 二、路径平滑模块

### 2.1 背景

当前路径规划算法（Sequential / Greedy / ABC / MSCGA）生成的路径是**折线段**——直接连接相邻视点的直线。在实际机器人运动中，这种路径存在以下问题：

1. **尖锐转角**：路径在视点处可能产生急转弯，机器人难以跟踪
2. **不连续的速度/加速度**：折线路径在转角处速度不连续
3. **不自然的运动**：对于喷涂、检测等应用，平滑路径更高效

### 2.2 平滑算法选择

| 算法 | 特点 | 适用场景 | 复杂度 |
|------|------|----------|--------|
| **B-spline 曲线** | 局部控制，不通过控制点 | 路径逼近（允许偏离原始点） | O(n) |
| **Catmull-Rom 样条** | 通过所有控制点，C1 连续 | 路径插值（必须通过所有视点） | O(n) |
| **Bezier 曲线拼接** | 通过端点，C1 连续 | 路径插值 | O(n) |
| **Chaikin 细分** | 简单迭代，渐进平滑 | 快速平滑 | O(n·k) |
| **Douglas-Peucker 简化** | 减少路径点数 | 路径压缩 | O(n·log n) |

**推荐方案**：**Catmull-Rom 样条** — 因为路径必须通过所有视点（这些是检测/作业的关键位置），同时提供 C1 连续性（位置和切线连续）。

### 2.3 技术设计

#### 2.3.1 新建模块 `path_smoothing.py`

```python
"""Path smoothing utilities.

Provides spline-based path smoothing for 3D viewpoint paths.
All functions accept and return lists/tuples of 3D points.
"""

def catmull_rom_smooth(points, tension=0.5, num_intermediate=10):
    """Generate a smooth Catmull-Rom spline through the given 3D points.

    Parameters
    ----------
    points : list of (x, y, z) tuples or gp_Pnt objects
        The waypoints to interpolate.
    tension : float
        Spline tension parameter (0.0 = uniform, 0.5 = centripetal, 1.0 = chordal).
    num_intermediate : int
        Number of intermediate points to generate between each pair of waypoints.

    Returns
    -------
    list of (x, y, z) tuples
        The smoothed path, including all original waypoints plus interpolated points.
    list of int
        Indices of the original waypoints in the smoothed path.
    """

def chaikin_smooth(points, iterations=3):
    """Chaikin's corner-cutting algorithm for quick path smoothing.

    Parameters
    ----------
    points : list of (x, y, z) tuples
    iterations : int
        Number of subdivision iterations.

    Returns
    -------
    list of (x, y, z) tuples
    """

def smooth_path_adaptive(points, max_angle_deg=30.0, num_intermediate=10):
    """Adaptive smoothing: only smooth segments with sharp turns.

    Parameters
    ----------
    points : list of (x, y, z) tuples
    max_angle_deg : float
        Segments with turn angle > this threshold will be smoothed.
    num_intermediate : int
        Number of intermediate points for smoothed segments.

    Returns
    -------
    list of (x, y, z) tuples
    """
```

#### 2.3.2 Catmull-Rom 样条算法实现

```python
def _catmull_rom_segment(p0, p1, p2, p3, tension, t):
    """Compute a point on a Catmull-Rom spline segment.

    t ranges from 0 (at p1) to 1 (at p2).
    """
    # Convert tension to alpha (centripetal parameterization)
    alpha = tension
    t2 = t * t
    t3 = t2 * t

    # Catmull-Rom matrix coefficients
    # Using the standard formulation with tension parameter
    v0 = p0
    v1 = p1
    v2 = p2
    v3 = p3

    # Hermite basis functions for Catmull-Rom
    # m1 = tension * (p2 - p0)
    # m2 = tension * (p3 - p1)
    m1 = tuple(tension * (c2 - c0) for c0, c2 in zip(v0, v2))
    m2 = tuple(tension * (c3 - c1) for c1, c3 in zip(v1, v3))

    # Hermite basis
    h00 = 2*t3 - 3*t2 + 1
    h10 = t3 - 2*t2 + t
    h00_val = h00
    h10_val = h10
    h01 = -2*t3 + 3*t2
    h11 = t3 - t2

    result = tuple(
        h00 * c1 + h10 * m1_i + h01 * c2 + h11 * m2_i
        for c1, m1_i, c2, m2_i in zip(v1, m1, v2, m2)
    )
    return result
```

#### 2.3.3 与现有模块的集成

**修改文件**：

| 文件 | 修改内容 |
|------|----------|
| `path_smoothing.py`（新建） | 路径平滑算法实现 |
| `planning.py` | 在路径规划完成后可选调用平滑 |
| `config.py` | 添加 `SmoothingConfig` 数据类 |
| `main.py` | 添加平滑菜单项 / 集成到路径规划对话框 |
| `renderer.py` | 添加平滑路径的渲染支持（曲线 vs 折线） |
| `export_utils.py` | 导出时可选包含平滑路径点 |

**`config.py` 新增**：
```python
@dataclass
class SmoothingConfig:
    enabled: bool = False
    method: str = "catmull_rom"  # "catmull_rom" / "chaikin" / "adaptive"
    tension: float = 0.5         # Catmull-Rom tension (0-1)
    num_intermediate: int = 10   # 中间点数量
    chaikin_iterations: int = 3  # Chaikin 迭代次数
    adaptive_angle_deg: float = 30.0  # 自适应平滑的角度阈值
```

### 2.4 用户交互流程

```
用户操作流程：
  Path Planning → 选择算法 → 勾选 "Smooth path" → 配置平滑参数 → OK

执行流程：
  1. 执行选定的路径规划算法 → 得到 optimal_path
  2. 如果 Smooth enabled:
     a. 提取路径点坐标
     b. 调用平滑算法
     c. 生成平滑路径点
     d. 渲染平滑曲线（覆盖原始折线）
  3. 更新 Workflow 状态和路径长度
```

### 2.5 渲染设计

**当前渲染**：`draw_path_edges()` 画直线段

**平滑路径渲染**：
- 使用 `AIS_Shape` 显示 B-spline 或折线逼近的曲线
- 或者用更密集的折线段近似曲线（简单方案）
- 颜色区分：原始路径用虚线，平滑路径用实线

**实现方案（简单版）**：
```python
def draw_smoothed_path(display, smoothed_points, path_objects):
    """Draw the smoothed path as a polyline with more segments."""
    # smoothed_points 已经是密集的点序列
    # 直接用 draw_path_edges 绘制即可
    indices = list(range(len(smoothed_points)))
    draw_path_edges(display, indices, smoothed_points, path_objects)
```

### 2.6 路径长度计算

平滑路径的长度 = 相邻平滑点之间的欧氏距离之和

```python
def calculate_smoothed_path_length(smoothed_points):
    total = 0.0
    for i in range(len(smoothed_points) - 1):
        total += point_distance(smoothed_points[i], smoothed_points[i+1])
    return total
```

### 2.7 CSV 导出

平滑路径的导出格式与原始路径相同，但行数更多（包含插值点）。

导出时添加一列 `is_original` 标记该点是否为原始视点。

```csv
x,y,z,qw,qx,qy,qz,is_original
1.0,2.0,3.0,1.0,0.0,0.0,0.0,true
1.1,2.1,3.1,0.0,0.0,0.0,0.0,false
...
```

---

## 三、实施优先级和依赖关系

```
Phase 1: 基础 UI 改善 (可独立实施)
├── T1: Workflow 面板按钮化
├── T2: 菜单启用/禁用
└── T3: 日志面板

Phase 2: 路径平滑模块 (依赖 Phase 1 中的 T4)
├── T4: 路径规划统一对话框 (包含平滑选项)
├── T5: path_smoothing.py 模块实现
├── T6: 集成到规划流程
└── T7: 渲染和导出支持

Phase 3: 打磨 (可选)
├── T8: UI 视觉优化
├── T9: 平滑参数预览 (实时预览平滑效果)
└── T10: 路径长度对比显示 (原始 vs 平滑)
```

**依赖关系**：
- T4（统一对话框）可以独立于 T1-T3 实施
- T5（平滑算法）可以独立实现和测试
- T6 依赖 T4 和 T5
- T7 依赖 T6

---

## 四、新增/修改文件清单

| 文件 | 类型 | 说明 |
|------|------|------|
| `path_smoothing.py` | 新建 | 路径平滑算法（Catmull-Rom, Chaikin, 自适应） |
| `ui_panels.py` | 修改 | Workflow 按钮化、日志面板、路径规划对话框 |
| `main.py` | 修改 | 菜单启用/禁用、平滑集成、日志替换 print |
| `config.py` | 修改 | 添加 SmoothingConfig |
| `planning.py` | 修改 | 路径规划后可选平滑 |
| `renderer.py` | 修改 | 平滑路径渲染 |
| `export_utils.py` | 修改 | 平滑路径导出 |

---

## 五、风险和注意事项

1. **gp_Pnt 兼容性**：平滑后的点是 `(x, y, z)` 元组而非 `gp_Pnt`，需要在渲染和距离计算时做类型转换
2. **路径长度变化**：平滑路径长度 > 原始折线长度（三角不等式），需要在 UI 中同时显示两个长度
3. **性能**：大量插值点可能影响渲染性能，建议 `num_intermediate` 默认值不超过 20
4. **Python 3.7 兼容**：避免使用 f-string、walrus operator 等 3.8+ 特性
5. **OCC 线程安全**：路径平滑算法不涉及 OCC 操作，可在任意线程执行
