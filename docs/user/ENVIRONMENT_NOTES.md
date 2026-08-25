# Portable Environment Configuration

The repository does not install packages or commit machine-specific absolute
paths. Use a complete existing Conda environment containing NumPy, PythonOCC,
and either PyQt5 or PySide6.

## Runtime selection

`run_integrated_app.ps1` discovers Conda from `PATH` and uses `base` by default.
Set `SCANNING_APP_CONDA_ENV` when another existing environment contains the
required stack. The bootstrap validates native imports before opening the UI;
it does not combine DLLs or `site-packages` from unrelated environments.

For persistent per-machine values, copy
`config/local_environment.example.ps1` to `config/local_environment.ps1`. The
copy is ignored by Git and loaded by the launcher before environment selection.

## RoboDK API discovery

RoboDK path import remains supported. The bundled API is resolved in this order:

1. `ROBODK_API_PATH` from the process or ignored local configuration;
2. `ROBODK_INSTALL_DIR`;
3. a RoboDK executable available on `PATH`;
4. Windows registry installation metadata;
5. standard operating-system program directories;
6. an importable RoboDK package in the selected Conda environment.

The legacy bundled API is preferred when found because it matches older RoboDK
station commands. No discovered path is written back to a tracked file.
