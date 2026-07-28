# 综合软件设计与分阶段实施计划

## 1. 已落地架构

```text
CAD STEP/IGES
  -> 曲面剖分
  -> 面心/法线/候选视点
  -> 几何与碰撞筛选
  -> 路径排序
  -> 受约束速度规划
  -> Pose+Speed CSV
  -> RoboDK Set Speed -> Move
```

模块边界：

- `geometry.py`：当前等参数剖分；后续策略扩展入口。
- `viewpoints.py`：扫描视点位置和四元数姿态。
- `planning.py`：顺序、Greedy、ABC、MSCGA 路径排序。
- `speed_planning_core.py`：确定性前后向速度规划 + Double Q 研究模式。
- `robodk_bridge.py`：每 Pose 四参数与运动指令配对导入。
- `speed_planning_ui.py`：集成 Qt 参数对话框。
- `pose_transform.py`：版本化`T_tool_scanner`校验和scanner→tool位姿转换。
- `main.py`：工作流编排；CAD 导入和剖分使用后台线程。
- `workers.py`：统一后台任务和进度窗口生命周期；主程序不再维护重复runner。

## 2. 论文方法与实现映射

论文第5章把速度规划放在最终扫描路径之后：状态是“路径点+该点离散速度”，
动作是下一点速度档，段时间为代价，并用关节力矩、关节角加速度作约束；采用双Q表、
epsilon-greedy衰减和随机表更新。

本版本实现：

- **确定性模式**：基于弧长、曲率、姿态变化、线加速度与角加速度限制构造速度包络，
  再做前向/后向时间参数化。该模式作为可复现、安全基线。
- **Double-Q 模式**：按照“路径点×速度档”的分层状态转移训练，采用代价最小化、
  双表随机更新、epsilon衰减和固定随机种子。
- 两种模式输出同一 schema，并由同一约束检查器验证。

没有直接复用旧 `speed_planning*.py`：这些脚本包含假关节角、近似动力学、恒零奖励项
和错误 PPO 概率比，只适合作为历史原型。

## 3. Pose+Speed schema

每点输出：

- `index,X,Y,Z,w,x,y,z`
- `path_s_mm,segment_length_mm,curvature_1_mm,orientation_delta_deg`
- `linear_speed,linear_accel,joint_speed,joint_accel`（RoboDK 四参数命令列）
- `tcp_angular_speed_deg_s,tcp_angular_accel_deg_s2`（TCP 姿态变化诊断列）
- `dt_to_next_s,max_linear_speed_mm_s,feasible,violation_codes`

必须另外保存的生产元数据：机器人、参考坐标系、工具、扫描仪到工具的固定外参、
长度/角度单位、约束配置版本和规划器版本。

注意：RoboDK `setSpeed` 的第 2/4 参数是关节速度/关节加速度，不是 TCP 姿态角速度/
角加速度。当前 `joint_speed/joint_accel` 是显式配置的命令上限；只有完成 IK 连续性和
机器人动力学验证后，才能称为逐点规划出的关节约束结果。
导出的线/角加速度命令裕量由`accel_margin_factor`显式配置，默认1.10，不再硬编码。

坐标链采用`T_A_B`表示B在A中的位姿：
`T_world_tool = T_world_scanner @ inverse(T_tool_scanner)`。CSV保留source scanner pose和
command tool pose，sidecar记录外参矩阵、状态、配置ID和SHA256。缺少`validated`外参时，
研究规划/导出允许，但RoboDK导入在UI与API两层被阻止。

## 4. UI 卡死修复

- Close event filter 只拦截主窗口；主窗口未知时不拦截任何子窗口。
- STEP/IGES 解析移至现有 QThread worker。
- 成功读取后才原子替换 `state`，取消或失败不会清空旧模型。
- 曲面剖分也移至后台 worker；所有 `display.*` 仍只在主线程执行。
- 移除 PyQt5 硬编码，自动选择 PyQt5/PySide6；Signal API 做兼容。

## 5. 曲面剖分演进

当前仍保留等参数剖分，但已增加逐split和全局面积守恒、periodic面、split异常及回退诊断。
不守恒的split会保守回退原face；这能阻止重叠/丢失patch进入下游，但不等同于完整的
trim-aware自适应剖分。论文方法应分阶段加入：

1. 把 `segment_model` 改造成策略接口：`equal_param | adaptive_complexity | thesis_geodesic`。
2. P1：参数域递归四分块，使用法线变化/曲率作为复杂度近似，按相对复杂度分配采样数。
3. P2：在三角网格上用 Dijkstra 图测地线作可验证近似。
4. P3：实现论文热方法（离散热方程、归一化热梯度、泊松方程）和块内
   Poisson/Hammersley/NRook 混合采样。
5. 用面积守恒、BRepCheck、ME/RMSE 验证；不能用 `patch_count == u*v` 作为复杂曲面的
   唯一成功判据。

## 6. 尚未完成的生产约束

- scanner pose 到 robot flange/tool pose 的固定外参。
- 每点稳定 IK 分支和奇异性指标。
- 基于真实 UR10 参数的关节速度、关节加速度和力矩验证。
- 控制器/后处理器对加速度字段的真实执行语义验证。
- 大模型、大路径的性能与多随机种子 Double-Q 对照。

这些约束未完成前，软件会在结果中保留
`dynamic_constraints_validated=False`，不得将输出称为生产动态可行轨迹。
