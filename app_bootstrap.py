"""Launch the integrated app using existing Conda environments only.

The historical Pythonocc environment's interpreter currently exits with
Windows status 0xC0000022.  The `test` Conda environment has the same Python
3.12 ABI, so this bootstrap uses its working interpreter and loads OCC/PySide6
from the existing Pythonocc environment.  No package installation is needed.
"""

import os
import sys


PYTHONOCC_PREFIX = os.environ.get(
    "PYTHONOCC_PREFIX", r"D:\Env\conda\2024\envs\Pythonocc")
PYTHONOCC_SITE_PACKAGES = os.path.join(PYTHONOCC_PREFIX, "Lib", "site-packages")
PYTHONOCC_DLLS = os.path.join(PYTHONOCC_PREFIX, "Library", "bin")
QT_PLUGIN_ROOT = os.path.join(PYTHONOCC_PREFIX, "Library", "lib", "qt6", "plugins")
QT_PLATFORM_PLUGINS = os.path.join(QT_PLUGIN_ROOT, "platforms")

if not os.path.isdir(PYTHONOCC_SITE_PACKAGES):
    raise RuntimeError("Pythonocc site-packages not found: {}".format(PYTHONOCC_SITE_PACKAGES))
if not os.path.isdir(PYTHONOCC_DLLS):
    raise RuntimeError("Pythonocc DLL directory not found: {}".format(PYTHONOCC_DLLS))

if hasattr(os, "add_dll_directory"):
    _occ_dll_handle = os.add_dll_directory(PYTHONOCC_DLLS)
    _occ_root_handle = os.add_dll_directory(PYTHONOCC_PREFIX)
if PYTHONOCC_SITE_PACKAGES not in sys.path:
    sys.path.insert(0, PYTHONOCC_SITE_PACKAGES)

os.environ.setdefault("QT_API", "pyside6")
os.environ.setdefault("PYTHONOCC_BACKEND", "pyside6")
os.environ.setdefault("QT_PLUGIN_PATH", QT_PLUGIN_ROOT)
os.environ.setdefault("QT_QPA_PLATFORM_PLUGIN_PATH", QT_PLATFORM_PLUGINS)
os.environ.setdefault("ROBODK_API_PATH", r"D:\RoboDK\Python37\lib\site-packages")

from main import run


if __name__ == "__main__":
    run()
