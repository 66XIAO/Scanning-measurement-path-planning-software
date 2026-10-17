# Portable Environment Configuration

The repository does not install packages or commit machine-specific absolute
paths. Keep the launcher code shared between machines and keep each machine's
paths in the ignored `config/local_environment.ps1`.

## Runtime selection

`run_integrated_app.ps1` discovers Conda from `PATH` and uses `base` by default.
Set `SCANNING_APP_CONDA_ENV` when another existing environment contains the
required stack. Prefer a complete environment containing NumPy, PythonOCC and
either PyQt5 or PySide6.

For persistent per-machine values, copy
`config/local_environment.example.ps1` to `config/local_environment.ps1`. The
copy is ignored by Git and loaded by the launcher before environment selection.

The supported local settings are:

| Setting | Purpose |
| --- | --- |
| `SCANNING_APP_CONDA_ENV` | Conda environment name; defaults to `base` |
| `SCANNING_APP_PYTHON_EXE` | Explicit interpreter path; takes precedence over the Conda name and does not require Conda on PATH |
| `SCANNING_APP_OCC_PREFIX` | Optional directory containing `Lib/site-packages/OCC` and `Library/bin`; normally leave unset |
| `SCANNING_APP_QT_BACKEND` | Optional `pyqt5` or `pyside6`; preflight and the GUI use the same binding |
| `ROBODK_API_PATH`, `ROBODK_INSTALL_DIR` | Optional local RoboDK locations |

`SCANNING_APP_OCC_PREFIX` is for an explicit interpreter workaround, such as
using a working interpreter with OCC packages from another environment. Both
must have the same Python major/minor version and compatible architecture and
native dependencies. A mismatch in available Conda Python metadata is rejected
before loading OCC. This option is not an automatic search or a silent fallback.
DLL search registrations remain alive for the application lifetime. When the
prefix has a Qt plugin directory matching the chosen binding, that local plugin
directory is used too.

For another machine, copy the **example** file and fill in that machine's values.
Usually only `SCANNING_APP_CONDA_ENV` is needed. Do not copy another machine's
`local_environment.ps1`, and do not ignore or untrack the shared launch scripts.
The local file is PowerShell code; use a trusted local configuration.

## Verification on each machine

Run `start_software.cmd --check` first. This checks imports, the selected Qt
binding and OCC status-list traversal, and prints the interpreter and actual
OCC module path. The displayed `OCC.VERSION` string alone may not identify the
installed build reliably.

Then run the same model through the configured launcher without opening a GUI:

```powershell
.\run_integrated_app.ps1 -ValidateModel 'D:\path\to\CarDoor.step' -GridU 5 -GridV 7
```

The model path is specific to each machine. A nonzero exit means the check
failed. Invalid-input BRep warnings remain visible and are not converted into
"valid geometry"; mesh-grid partition warnings cause the check to fail. Finish
with a normal GUI import, face selection and segmentation check on each machine.
No physical robot validation is implied by this check.

The common segmentation code supports both Python-iterable status containers and
native OCC list iterators without mutating analyzer-owned lists. Existing old
OCC constructor, meshing, worker-thread and rendering compatibility is retained;
neither machine is required to upgrade OCC by this change.

## ABC path planner dependencies

The ABC solver is included as `ABC.py` in this repository; no original-project
directory or extra `PYTHONPATH` is needed. Its required dependency is NumPy.
`progressbar` (from progressbar2) and Matplotlib are optional, used for console
progress and plot output when requested. Missing optional modules do not block
path planning. The solver no longer imports the unused `deap` package.
The startup preflight checks that the bundled module imports successfully.
See [ABC integration notes](../architecture/ABC_INTEGRATION.md) for provenance,
the preserved objective function, and regression checks.

## UI icons and spacing

SVG assets are resolved relative to `ui_theme.py` under `assets/`, independent
of the current working directory. The loader checks actual icon pixels: some
Qt installations can construct a QIcon but cannot load their SVG icon-engine
plugin. In that case the selected binding's QtSvg renderer creates transparent
icons at multiple resolutions from the same SVG files. Working native SVG
engines continue to supply vector icons. Missing or invalid assets produce a
path-specific error. Icons are owned by their callers rather than a global Qt
object cache, so they do not outlive the application during shutdown.

Compact, standard and large ribbon settings keep their existing preference
names. Spacing is reduced while retaining font sizes; ribbon height follows
the content and active Qt style, including room for horizontal scrolling.
The UI smoke check verifies visible icon pixels, English/Chinese labels,
button minimum sizes, layout recovery and the CAD/workstation workflow.

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
