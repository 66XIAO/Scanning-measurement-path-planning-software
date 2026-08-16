# Existing Conda Environment Notes

No package was installed during this integration.

## Observed environments

- `Pytorch`: Python 3.9, numpy/pandas/torch/tkinter; used for pure algorithms/tests.
- `test`: working Python 3.12 interpreter; used to launch the integrated GUI.
- `Pythonocc`: contains Python 3.12, pythonocc-core 7.9.3, PySide6 6.7.2 and Qt6,
  but its own `python.exe` exits with Windows status `0xC0000022`.

## Reuse strategy

`app_bootstrap.py` runs with `test\python.exe`, calls `os.add_dll_directory` for
the existing Pythonocc DLLs, adds the existing Pythonocc site-packages, sets the
Qt6 platform plugin path and selects the `pyside6` backend. This preserves the
user requirement to reuse Conda environments and avoid installing into base.

## RoboDK API compatibility

The installed RoboDK application is `4.0.0.13353`. Its bundled API under
`D:\RoboDK\Python37\lib\site-packages` uses the matching legacy station
commands, including `S_Frame_ptr`. The newer API present in the Pythonocc
environment sends `S_Link_ptr`; RoboDK 4.0.0 does not reply and the path import
times out in `program.setFrame(frame)`.

`robodk_bridge.py` therefore loads the two bundled API source modules directly
and prefers them over the newer environment package. Direct loading avoids the
bundled 2019 package `__init__.py`, whose removed `find_module()` call is not
compatible with the application's Python 3.12 interpreter. No package is
installed or modified by this compatibility layer.
