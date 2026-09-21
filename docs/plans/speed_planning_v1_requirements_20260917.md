# 连续扫描速度规划 V1：需求与现状

## 已确认范围
- 软件内规划、RoboDK 离线仿真、程序导出；不包含 UR10 实机验收。
- 扫描内部点连续通过，允许低速；整条轨迹首尾允许起停。
- 效率提升是验收目标，必须在相同路径任务、约束、容差和计时区间下验证。
- 速度范围、位置及姿态容差尚未确定，需要文献及参数敏感性研究。

## 当前证据
- 用户截图：32 点，deterministic，26.690 s，feasible=Yes；映射缺失或未验证导致 RoboDK 导入被阻止。此时间是规划估计，不能直接与另一截图的仿真时间比较。
- GUI 已有读取当前 RoboDK 工作站坐标关系入口。代码接受 station_verified 供仿真使用，但部分中英文错误提示仍只写 validated，存在提示语义滞后。
- 当前速度算法是 TCP 离散曲率/姿态变化速度上限、前后向加减速和角加速度迭代降速；没有连续关节轨迹约束。
- 规划段字段是 outgoing，RoboDK Set Speed 后 Move 的指令作用于 incoming，需建立独立命令转换层。
- 当前导入桥未检出 setRounding。仅设置各点速度不能保证不停车，必须核查实际程序和后处理输出。
- Computer Use 已识别两个窗口；首次取状态 app approval timed out；刷新窗口后重试出现 SetIsBorderRequired failed: 不支持此接口 (0x80004002)。未执行 GUI 点击，GUI 验收未完成。

## 平滑参考仓库
只读参考 D:\Pycharm files\Path Smoothing，HEAD 44de804，前一提交 b3fe25e。
- Yang 2020 C3 与 Peng 2021 解耦局部平滑：作为优先审计候选；需验证位置/姿态误差、段连接连续性和退化情况，不能把脚本说明当复现验收。
- Huo2024 脚本原论文针对 Ackerman 移动机器人，轴距/转向角曲率约束不能直接作为 UR10 关节约束。
- 不直接将 PyQt5/OCC GUI 脚本整体并入当前应用；先明确原始方法边界，再提取经过核验的纯计算接口。

## 实施顺序与验收
1. 映射诊断：区分缺失、示例、站点不符、工具不符；捕获当前站点，重算后导入。保持现有检查，不伪造验证标志。
2. 几何路径：选择一种容差受控平滑方法，保留位姿及段来源；位置和 SO(3) 姿态误差单独测量。
3. 时间参数化：连续 IK、关节分支检查、关节速度/加速度约束；TOPP-RA 为候选基线。完整保存 t/q/qd/qdd 与 TCP 状态。jerk 先作为独立诊断，未实现约束不得宣称受限。
4. 执行转换：独立 incoming_segment 命令表，首段接近/末端退出单列；配置过渡并核查 URScript 的 v/a/r/t 及连续性。参考轨迹与控制器重规划结果分别报告。
5. 三组对照：A 原始路径逐点停止；B 容差内平滑＋固定速度＋连续过渡；C 相同平滑路径＋受约束变速＋相同过渡策略。A-C 测整体收益，B-C 分离速度规划贡献。
6. 验收指标：扫描段仿真耗时、内部非预期停车次数、TCP 最低速度、关节速度/加速度违反量、jerk 诊断、位置/姿态偏差、规划耗时、导出与指令读回一致性。没有实际扫描数据前不宣称扫描质量改善。

## 文献和官方依据
- RoboDK Set Rounding：无 rounding 的连续移动通常在非相切连接处减速至零；rounding 带来误差/速度权衡。https://robodk.com/doc/en/Robot-Programs.html
- Yang et al. 2020，An analytical C3 continuous tool path corner smoothing algorithm for 6R robot manipulator：在位置与姿态误差约束下构造同步平滑路径。https://doi.org/10.1016/j.rcim.2020.101947
- TOPP-RA 官方文档：https://hungpham2511.github.io/toppra/

## V1 完成状态（2026-09-19）

- 已读取并锁定当前真实站点映射；映射缺失、工具／参考系不符仍会阻止导入。
- 已完成 Yang2020 纯计算内核来源审计、欧拉角分支处理、共享姿态基函数工程适配及密集 SO(3) 误差审计。
- 已完成9组位置／姿态容差敏感性试验。当前案例由姿态预算主导；V1选择1.5 mm／2°和最大0.3 mm rounding，最终FK误差0.7354 mm／1.2546°。
- 已建立参考节点与 incoming-segment 命令的独立数据契约，接入连续IK、关节重定时、原生MoveL导入和高精度URScript导出。
- 已完成三组同条件RoboDK 1倍播放：A 252.9568 s、B 209.7092 s、C 204.0856 s；最终报告见 `docs/reports/speed_planning_v1_final.md`。
- GUI速度设置对话框已完成离屏构造、默认值、双语文案和截图检查。Computer Use能够定位已打开的目标站点窗口，但本机截图组件返回`SetIsBorderRequired 0x80004002`，因此没有把Computer Use截图作为验收证据；RoboDK API和用户当前窗口截图共同确认站点存在。
