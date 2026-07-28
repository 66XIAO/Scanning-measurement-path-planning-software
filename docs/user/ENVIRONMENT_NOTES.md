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
