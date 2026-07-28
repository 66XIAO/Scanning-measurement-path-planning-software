# Trae审查独立复核与处理结果

## 结论概要

Trae报告中的14项正向确认基本成立。7个问题中：4项需要处理，2项本来就是正确行为，
1项属于尚未证实的periodic风险。复核没有照搬报告，而是增加了实际OCC模型测试。

## 逐项裁决

| # | Trae结论 | 独立复核 | 处理 |
|---|---|---|---|
| 1 | trimmed面可能异常 | 确认风险，但实际表现是patch重叠，不是静默丢失 | 带圆孔薄板复现面积比1.060085；增加逐split面积守恒，不守恒即回退 |
| 2 | 缺少面积守恒 | 确认 | 增加原面积、patch面积、比值、容差和UI警告 |
| 3 | periodic面可能退化 | 尚未证明几何退化；periodic面客观存在 | 增加periodic计数和seam检查警告；不守恒由同一回退保护 |
| 4 | 1.10裕量硬编码 | 确认 | 新增`accel_margin_factor`配置、校验、UI和测试 |
| 5 | 单位四元数 | 当前实现正确 | 不修改 |
| 6 | 后台runner重复 | 确认 | `main.py`统一使用`workers.TaskRunner` |
| 7 | 两套CSV header | 分别服务纯Pose和Pose+Speed，行为正确 | 不修改 |

## 实测证据

- 普通box，2×2：24 patches，面积比1.000000。
- 带圆孔薄板，修复前2×2：28 patches，面积比1.060085，测试失败。
- 同一带孔模型，修复后：22 patches，面积比1.000000；异常split被拒绝并保留原face。
- 纯算法/RoboDK/CSV：10项通过。
- OCC专项：2项通过。

patch数量减少不等于面积丢失：保守回退会保留较大的原face，并通过diagnostics明确说明。
完整trim-aware策略、自适应剖分与热方法仍属于下一阶段S1-S3。

## 环境结论修正

- Pytorch：Python3.9，SciPy 1.13.1，NumPy 2.0.2，可做独立算法研究。
- Pythonocc：Python3.12，NumPy 2.4.2，无SciPy。
- test：Python3.12，本身无NumPy/SciPy，通过bootstrap复用Pythonocc环境。
- 不能把Pytorch的SciPy直接注入GUI环境，主要原因不仅是NumPy版本，还包括Python3.9/
  Python3.12扩展ABI不同。热方法应先采用进程隔离或明确的序列化边界。
