"""Portable discovery of RoboDK's bundled Python API.

Machine-specific installation paths belong in ``ROBODK_API_PATH`` or
``ROBODK_INSTALL_DIR``.  The fallback discovery uses PATH, Windows registry
metadata, and operating-system program directories without embedding a drive
letter or user-specific absolute path in the repository.
"""

import os
from pathlib import Path
import shutil


def _unique_paths(paths):
    seen = set()
    for value in paths:
        if not value:
            continue
        path = Path(os.path.expandvars(os.path.expanduser(str(value))))
        key = os.path.normcase(os.path.abspath(str(path)))
        if key in seen:
            continue
        seen.add(key)
        yield path


def _legacy_site_packages(path):
    """Return a legacy bundled-API root below *path*, if present."""
    candidates = (
        path,
        path / "Python37" / "lib" / "site-packages",
        path / "Python" / "lib" / "site-packages",
    )
    for candidate in _unique_paths(candidates):
        if ((candidate / "robodk" / "robodk.py").is_file()
                and (candidate / "robolink" / "robolink.py").is_file()):
            return str(candidate.resolve())
    return None


def _registry_install_roots():
    if os.name != "nt":
        return []
    try:
        import winreg
    except ImportError:
        return []
    roots = []
    locations = (
        (winreg.HKEY_CURRENT_USER, r"SOFTWARE\RoboDK"),
        (winreg.HKEY_LOCAL_MACHINE, r"SOFTWARE\RoboDK"),
        (winreg.HKEY_LOCAL_MACHINE, r"SOFTWARE\WOW6432Node\RoboDK"),
    )
    for hive, key_name in locations:
        try:
            with winreg.OpenKey(hive, key_name) as key:
                for value_name in ("Install_Dir", "InstallDir", "InstallPath", ""):
                    try:
                        value, _value_type = winreg.QueryValueEx(key, value_name)
                    except OSError:
                        continue
                    if value:
                        roots.append(value)
        except OSError:
            continue
    return roots


def _automatic_install_roots(environ):
    roots = [environ.get("ROBODK_INSTALL_DIR", "")]
    executable = shutil.which("RoboDK.exe") or shutil.which("RoboDK")
    if executable:
        roots.append(str(Path(executable).resolve().parent))
    for variable in ("ProgramFiles", "ProgramFiles(x86)", "LOCALAPPDATA"):
        base = environ.get(variable, "")
        if base:
            roots.append(str(Path(base) / "RoboDK"))
    roots.extend(_registry_install_roots())
    return roots


def discover_robodk_api_path(environ=None, install_roots=None):
    """Return RoboDK's legacy API site-packages path, or ``None``.

    ``install_roots`` is primarily useful for diagnostics and tests.  Normal
    callers should configure ``ROBODK_API_PATH``/``ROBODK_INSTALL_DIR`` or let
    automatic discovery inspect the current machine.
    """
    environ = os.environ if environ is None else environ
    explicit = environ.get("ROBODK_API_PATH", "").strip()
    if explicit:
        found = _legacy_site_packages(Path(explicit))
        if found:
            return found
    roots = (_automatic_install_roots(environ)
             if install_roots is None else install_roots)
    for root in _unique_paths(roots):
        found = _legacy_site_packages(root)
        if found:
            return found
    return None
