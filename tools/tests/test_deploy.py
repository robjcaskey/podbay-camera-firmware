#!/usr/bin/env python3
from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).resolve().parents[1] / "deploy.py"
SPEC = importlib.util.spec_from_file_location("podbay_deploy", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
DEPLOY = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = DEPLOY
SPEC.loader.exec_module(DEPLOY)


class DeployProviderTests(unittest.TestCase):
    def test_source_provider_is_the_only_build_path(self) -> None:
        args = DEPLOY.parse_args(
            [
                "build",
                "--output-dir",
                "/tmp/result",
                "--kernel-build",
                "/tmp/kernel",
            ]
        )
        self.assertFalse(hasattr(args, "sensor_provider"))
        self.assertEqual(args.kernel_build, Path("/tmp/kernel"))

    def test_source_provider_arguments(self) -> None:
        args = DEPLOY.parse_args(
            [
                "deploy",
                "--accept-camera-changes",
                "--kernel-build",
                "/tmp/kernel",
            ]
        )
        self.assertEqual(args.kernel_build, Path("/tmp/kernel"))

    def test_source_handoff_is_parameter_free_and_fail_closed(self) -> None:
        variant = DEPLOY.FIRMWARE_VARIANTS[0]
        script = DEPLOY.deployment_script(
            "1" * 64, "2" * 64, variant, DEPLOY.SOURCE_WARM_PROVIDER, "stock"
        )
        self.assertIn('insmod "$module"', script)
        self.assertNotIn('insmod "$module" chmap=', script)
        self.assertIn("grep -q '^podbay_imx582_warm ' /proc/modules", script)
        self.assertIn("camera_state=$(awk", script)
        self.assertIn("trap rollback_on_exit EXIT", script)
        self.assertIn("rollback_needed=0", script)

    def test_source_restart_gracefully_unloads_authored_provider(self) -> None:
        variant = DEPLOY.FIRMWARE_VARIANTS[0]
        script = DEPLOY.deployment_script(
            "1" * 64, "2" * 64, variant, DEPLOY.SOURCE_WARM_PROVIDER, "source"
        )
        self.assertIn("! pidof pw203-camera-service", script)
        self.assertIn("rmmod podbay_imx582_warm", script)
        self.assertNotIn("kill -KILL", script)

    def test_unknown_restart_state_is_rejected(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "unsupported camera runtime"):
            DEPLOY.deployment_script(
                "1" * 64,
                "2" * 64,
                DEPLOY.FIRMWARE_VARIANTS[0],
                DEPLOY.SOURCE_WARM_PROVIDER,
                "unknown",
            )

    def test_source_provider_refuses_unproven_firmware(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "not live-proven"):
            DEPLOY.build_warm_sensor_module(
                Path("/tmp/missing"), Path("/tmp"),
                DEPLOY.FirmwareVariant("unproven", {}, {})
            )


if __name__ == "__main__":
    unittest.main()
