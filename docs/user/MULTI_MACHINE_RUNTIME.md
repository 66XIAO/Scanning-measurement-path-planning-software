# 两台机器共用启动器与本地环境配置

共用启动器和兼容代码随 Git 同步；解释器、OCC、Qt、RoboDK 的机器路径仅放在
`config/local_environment.ps1`。该文件已经在 `.gitignore` 中，正常提交和拉取不会覆盖另一台机器的配置。

## 另一台机器的配置步骤

1. 同步本次共用代码，包括新增的 `runtime_environment.py`，保留已有的旧版 OCC 参数、线程和显示兼容代码。
2. 如果尚无 `config/local_environment.ps1`，复制 `config/local_environment.example.ps1` 并重命名；已有配置则编辑已有文件。
3. 若该机器有完整可用的 Conda 环境，通常只需填写：

   ```powershell
   $env:SCANNING_APP_CONDA_ENV = '该机器实际的环境名'
   $env:SCANNING_APP_QT_BACKEND = 'pyqt5' # 按实际环境选择；也可用 pyside6 或留空自动选择
   ```

4. 如果需要直接指定解释器，则使用 `SCANNING_APP_PYTHON_EXE`，它优先于环境名。只有确实需要另一环境的 OCC 包/DLL 时才填写 `SCANNING_APP_OCC_PREFIX`；两边 Python 主次版本必须一致。不要复制本机的 D 盘路径。
5. 如果系统环境变量中已有不再使用的 `SCANNING_APP_PYTHON_EXE` 或 `SCANNING_APP_OCC_PREFIX`，在本地配置中将它们设为空字符串，避免旧设置继续生效。

更多设置见 [ENVIRONMENT_NOTES.md](ENVIRONMENT_NOTES.md)。

## 本机已配置的方式

本机本地文件已配置为：test 环境解释器加载 Pythonocc 环境的包、DLL 和 PySide6，使用本机 RoboDK API。共用代码中没有写入这些绝对路径。

## 在每台机器上验证

在项目目录运行：

```powershell
.\start_software.cmd --check
```

确认输出的解释器、OCC 模块路径和 Qt 后端均属于预期环境，且退出码为 0。
启动预检使用与界面相同的 Qt 选择逻辑，并检查 OCC 状态列表的兼容遍历。

再执行车门无界面检查，将路径换成该机器上的实际模型位置：

```powershell
.\run_integrated_app.ps1 -ValidateModel 'D:\模型目录\CarDoor.step' -GridU 5 -GridV 7
```

本机指定的 CarDoor.step 在未显示模型时得到 35 个分区。分割输出应包含 mesh_grid
策略、接近 1 的分区/源网格面积比，且无分区错误。`UnorientableShape` 是保留的输入
BRep 诊断，不能为了消除提示而把模型改判为有效，也不等于本次的状态列表异常仍存在。

最后正常双击 `start_software.cmd`，执行导入、选面、分割、提取中心点与法向，检查显示。
GUI 的网格缓存行为与无界面检查不同；不能仅靠分区数量判定拓扑和扫描质量已经验收。

## 同步边界

- 应同步：三个启动文件、`runtime_environment.py`、`main.py` 的 Qt 选择接入、`surface_segmentation.py` 的状态遍历兼容、配置示例、相关测试与说明。
- 不同步：`config/local_environment.ps1`，不使用强制添加命令提交它。
- 不将三个启动文件加入忽略列表，也不对它们取消跟踪。
- 本次没有要求另一台机器升级 OCC；其具体旧版本仍需在该机器完成上述验证。
