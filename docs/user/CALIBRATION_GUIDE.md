# 扫描仪到工具外参配置指南

## 为什么需要它

CAD生成的是扫描仪在世界/参考坐标系中的期望位姿，机器人执行的是工具或法兰位姿。
两者不能直接当作同一个Pose。软件采用以下统一约定：

```text
T_A_B：B坐标系在A坐标系中的位姿，也把B坐标转换到A坐标
T_world_scanner = T_world_tool × T_tool_scanner
T_world_tool    = T_world_scanner × inverse(T_tool_scanner)
```

平移单位为mm，四元数顺序为`w,x,y,z`。

## 配置文件

格式示例：[`calibration/scanner_to_tool.example.json`](../../calibration/scanner_to_tool.example.json)。
该文件是单位阵示例，不是真实标定，
其`calibration_status`为`example_only`，不会解锁RoboDK导入。

关键字段：

- `schema_version`：当前写出`1.2`；读取时兼容`1.0`和`1.1`。
- `config_id`：标定记录的唯一编号。
- `calibration_status`：`example_only | unvalidated | station_verified | validated`。
- `T_tool_scanner`：4×4刚体齐次矩阵。
- `length_unit=mm`、`quaternion_order=wxyz`。

当配置来自当前打开的RoboDK工作站时，还会保存：

- `T_station_robot_base`：UR10 Base在RoboDK站点全局坐标系中的位姿；
- `T_station_reference_frame`：所选参考系（例如`Frame 2`）在站点全局坐标系中的位姿；
- `T_base_workpiece`：工件/参考系在UR10 Base坐标系中的位姿；
- `T_flange_scanner`：扫描仪TCP在机器人法兰坐标系中的位姿。

三条参考系关系必须满足：

```text
T_base_workpiece = inverse(T_station_robot_base) × T_station_reference_frame
```

`T_station_reference_frame`和`T_base_workpiece`只有在机器人Base与站点全局原点
完全重合时才相同，不能因为名称中包含“Frame”而互换使用。

程序会拒绝非有限值、非正交旋转、行列式不是+1、错误末行、错误单位或未知版本。

## 使用步骤

1. 若RoboDK工作站已经配置好机器人、参考系和扫描仪TCP，使用
   `Calibration -> 读取当前 RoboDK 工作站坐标关系...`，填写精确对象名并选择
   保存位置。保存对话框可以覆盖旧JSON，也可以新建JSON。
2. 软件只读取现有对象，不创建目标点/程序，也不移动机器人。保存后的状态为
   `station_verified`，只代表RoboDK仿真站点关系已核对。
3. 若使用独立命令工具而不是“RoboDK TCP即扫描仪”的模式，复制示例文件并填入
   实测`T_tool_scanner`，先保持`unvalidated`。
4. 也可以通过`Calibration -> 加载扫描仪到工具的外参`重新加载已有JSON。
5. 运行速度规划并导出CSV；检查其中source pose与command pose是否符合安装方向。
6. 在多个位置和姿态下，用已知基准件验证扫描仪实际视线/测点与理论结果。
7. 保存标定方法、日期、操作者、扫描仪/支架序列号和残差报告。
8. 只有验证通过后才把状态改为`validated`，重新加载并重新规划速度。

## 输出与安全门

- CSV中的`X,Y,Z,w,x,y,z`是转换后的tool command pose。
- `source_X...source_z`保存原始scanner pose。
- 每行保存frame、config id、是否应用、是否验证。
- 同名`.metadata.json`保存矩阵、配置SHA256、诊断、算法和警告。
- 缺失或未验证外参时，可做研究规划和导出，但UI及`robodk_bridge.py`都会拒绝RoboDK导入。

## 本阶段仍缺少的真实输入

- 实测的扫描仪相对工具/法兰安装矩阵；
- 采用的机器人reference frame与CAD world frame之间的关系；
- 标定残差和允许阈值；
- 扫描仪、支架、工具的版本/序列号。

这些数据由实际标定提供，软件不会用示例单位阵代替。
