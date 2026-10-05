import unittest

from workers import pythonocc_version_tuple, supports_background_occ_objects


class OCCWorkerCompatibilityTests(unittest.TestCase):
    def test_version_parser_accepts_release_and_dev_versions(self):
        self.assertEqual(pythonocc_version_tuple("7.4.1-dev"), (7, 4, 1))
        self.assertEqual(pythonocc_version_tuple("7.9.3"), (7, 9, 3))

    def test_only_known_safe_runtime_uses_occ_worker_transfer(self):
        self.assertFalse(supports_background_occ_objects("7.4.1-dev"))
        self.assertFalse(supports_background_occ_objects("unknown"))
        self.assertTrue(supports_background_occ_objects("7.9.3"))
        self.assertTrue(supports_background_occ_objects("8.0.0"))


if __name__ == "__main__":
    unittest.main()
