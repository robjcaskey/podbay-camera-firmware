#!/usr/bin/env python3
from __future__ import annotations

import importlib.util
import subprocess
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock


MODULE_PATH = Path(__file__).resolve().parents[1] / "pw203_bootstrap_ssh.py"
SPEC = importlib.util.spec_from_file_location("pw203_bootstrap_ssh", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
BOOTSTRAP = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BOOTSTRAP)


class BootstrapTests(unittest.TestCase):
    def test_live_proven_usb_defaults(self) -> None:
        args = BOOTSTRAP.parser().parse_args([])
        self.assertEqual(args.usb_timeout_ms, 1500)
        self.assertEqual(args.usb_pause_ms, 300)

    @mock.patch.object(BOOTSTRAP, "configure_interface")
    @mock.patch.object(BOOTSTRAP, "wait_for_interface", return_value="enx1234")
    @mock.patch.object(BOOTSTRAP.Path, "is_dir", return_value=False)
    def test_udev_rename_is_resolved_once(
        self,
        _is_dir: mock.Mock,
        _wait: mock.Mock,
        configure: mock.Mock,
    ) -> None:
        configure.side_effect = [
            subprocess.CalledProcessError(1, ["ip", "link"]),
            None,
        ]
        args = SimpleNamespace(wait_interface_seconds=5.0)
        selected = BOOTSTRAP.configure_selected_interface(args, "usb0")
        self.assertEqual(selected, "enx1234")
        self.assertEqual(
            configure.call_args_list,
            [mock.call(args, "usb0"), mock.call(args, "enx1234")],
        )

    @mock.patch.object(BOOTSTRAP, "configure_interface")
    @mock.patch.object(BOOTSTRAP.Path, "is_dir", return_value=True)
    def test_existing_interface_error_is_not_retried(
        self,
        _is_dir: mock.Mock,
        configure: mock.Mock,
    ) -> None:
        failure = subprocess.CalledProcessError(1, ["ip", "link"])
        configure.side_effect = failure
        args = SimpleNamespace(wait_interface_seconds=5.0)
        with self.assertRaises(subprocess.CalledProcessError):
            BOOTSTRAP.configure_selected_interface(args, "enx1234")


if __name__ == "__main__":
    unittest.main()
