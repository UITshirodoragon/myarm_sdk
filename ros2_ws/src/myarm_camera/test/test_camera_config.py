"""Manifest policy tests independent of a physical V4L2 device."""

import unittest

from myarm_camera.camera_config import camera_instance, resolve_camera_profile

SERVICES_CONFIG = "service/config/services.yaml"


class CameraConfigTests(unittest.TestCase):
    def test_none_profile_has_no_processes(self):
        self.assertEqual(resolve_camera_profile(SERVICES_CONFIG, "none"), ())

    def test_cam01_profile_is_enabled(self):
        self.assertEqual(resolve_camera_profile(SERVICES_CONFIG, "cam01"), ("cam01",))
        self.assertEqual(camera_instance(SERVICES_CONFIG, "cam01")["role"], "wrist")

    def test_disabled_cam02_is_rejected(self):
        with self.assertRaisesRegex(RuntimeError, "disabled"):
            resolve_camera_profile(SERVICES_CONFIG, "cam02")

    def test_fake_dual_profile_has_two_test_instances(self):
        self.assertEqual(
            resolve_camera_profile(SERVICES_CONFIG, "fake_dual"),
            ("fake_cam01", "fake_cam02"),
        )


if __name__ == "__main__":
    unittest.main()
