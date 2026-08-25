import os
from pathlib import Path
import tempfile
import unittest

from robodk_discovery import discover_robodk_api_path


def _make_legacy_api(root):
    site_packages = Path(root) / "Python37" / "lib" / "site-packages"
    (site_packages / "robodk").mkdir(parents=True)
    (site_packages / "robolink").mkdir(parents=True)
    (site_packages / "robodk" / "robodk.py").touch()
    (site_packages / "robolink" / "robolink.py").touch()
    return site_packages


class RoboDKDiscoveryTests(unittest.TestCase):
    def test_explicit_api_path_is_used(self):
        with tempfile.TemporaryDirectory() as directory:
            site_packages = _make_legacy_api(directory)
            result = discover_robodk_api_path(
                {"ROBODK_API_PATH": str(site_packages)}, install_roots=[])
            self.assertEqual(os.path.normcase(result),
                             os.path.normcase(str(site_packages.resolve())))

    def test_install_root_is_resolved_without_fixed_drive(self):
        with tempfile.TemporaryDirectory() as directory:
            site_packages = _make_legacy_api(directory)
            result = discover_robodk_api_path({}, install_roots=[directory])
            self.assertEqual(os.path.normcase(result),
                             os.path.normcase(str(site_packages.resolve())))

    def test_incomplete_api_is_not_returned(self):
        with tempfile.TemporaryDirectory() as directory:
            self.assertIsNone(discover_robodk_api_path(
                {"ROBODK_API_PATH": directory}, install_roots=[]))


if __name__ == "__main__":
    unittest.main()
