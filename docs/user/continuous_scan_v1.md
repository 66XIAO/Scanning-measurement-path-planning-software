# 连续扫描速度规划 V1 操作说明

V1 已用于当前 32 点大曲面路径和 RoboDK 站点 `UR10_大曲面_MSCGA_corrected` 的离线验证。该版本保留 RoboDK 原生 `MoveL`，内部点使用非零 rounding 连续通过，首尾正常起停。参数均为仿真实验设置，不是 UR10 实机参数。

## 软件内使用

1. 关闭旧窗口后运行 `start_software.cmd`，确保加载当前代码。
2. 在 RoboDK 打开目标站点，确认机器人 `UR10`、参考系 `Frame 2`、工具 `Creaform MetraSCAN`。
3. 在软件中选择“标定 → 读取当前 RoboDK 工作站坐标关系”，保存并选用 `station_verified` 映射。映射缺失或与活动站点不符时，导入继续被阻止。
4. 生成当前 32 点有序路径。打开“速度规划”，选择“连续扫描：Yang 平滑＋确定性规划（仿真）”。
5. 当前验收参数：位置容差 1.5 mm、姿态容差 2°、最大 rounding 0.3 mm、内部最低扫描速度 0.1 mm/s；首尾速度由连续模式固定为 0。
6. 点击规划。系统依次执行容差受控平滑、位姿误差审计、冗余点压缩、TCP 前后向速度规划、RoboDK 连续 IK/FK 及关节参考轨迹重定时。
7. 导入时使用独立程序名。程序结构为首次接近 `MoveJ`，随后对每个扫描段输出“Set Speed → Set Rounding → MoveL”；最后一个点 rounding 为 0。
8. 使用“速度规划 → 导出连续扫描 URScript（离线）”。主 `.script` 保留高精度速度、加速度和 rounding；`stock_post_output` 仅作为原后处理输出留档。

## 当前验收结果

最终公平对比使用同一站点、同一起点、RoboDK 1 倍模拟和同一计时方法。三组分别为原始 32 点逐点停止、平滑固定速度、平滑变速。

| 组别 | 位姿数 | 实际播放耗时 / s | 相对逐点停止 | 最大位置误差 / mm | 最大姿态误差 / ° |
| --- | ---: | ---: | ---: | ---: | ---: |
| A 逐点停止 | 32 | 252.9568 | — | 约 0 | 0 |
| B 平滑固定速度 | 330 | 209.7092 | 缩短 17.10% | 0.7354 | 1.2546 |
| C 平滑变速 | 330 | 204.0856 | 缩短 19.32% | 0.7354 | 1.2546 |

C 比 B 继续缩短 2.68%。B、C 的内部 rounding 最小值为 0.049 mm，终点 rounding 为 0；程序无 Pause 指令，RoboDK `Update` 有效率为 1.0，官方轨迹接口返回 `Success` 且错误码为 0。六轴原生轨迹速度与加速度均低于本轮 60°/s、180°/s²仿真实验限值。

最终机器可读报告：`artifacts/speed_v1/fair_playback_20260919_093222/final_report.json`。

最终变速 URScript：`artifacts/speed_v1/fair_playback_20260919_093222/C_variable/C_variable_fair_playback_20260919_093222.script`。

速度与耗时图：`artifacts/speed_v1/fair_playback_20260919_093222/final_figure_20260919/`。

## 结果边界

- “连续”在 V1 中指程序内部点均有非零 rounding、无显式停顿指令；旧版 RoboDK 的轨迹采样接口不能直接证明目标点处瞬时速度严格大于零。
- 误差来自 RoboDK 站点模型 FK 与原始路径的位置／SO(3)姿态比较，不是扫描仪测量误差。
- 尚未完成碰撞、扭矩、控制器插补、实物标定、真实扫描质量或 UR10 实机执行验证。
- `.urp` 是原始后处理器输出，可能把小于 0.5 mm 的过渡半径舍入为 0；本版本交付以高精度主 `.script` 为准。

## 复现与保留

运行全部自动化测试：

```powershell
& 'D:\Env\conda\2024\envs\test\python.exe' -m unittest discover -s tests -v
```

当前站点的清理、备份和对象保留规则见 `docs/user/robodk_experiment_retention.md`。不要在活动站点中无限累计带时间戳的测试程序；完成实验后先保存证据和站点备份，再按精确名称删除已退役对象。
