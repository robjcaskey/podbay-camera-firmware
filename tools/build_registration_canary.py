#!/usr/bin/env python3
"""Build the SigmaStar registration canary outside the repository."""

from __future__ import annotations

import argparse
import shutil
import subprocess
import tempfile
from pathlib import Path

from build_kernel_canary import ROOT, inside, verify_module_layout


SOURCE = ROOT / "sensor-provider" / "registration-canary"
MODULE = "podbay_registration_canary"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--kernel-build", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--cross-compile", default="arm-linux-gnueabihf-")
    parser.add_argument("--readelf", default="arm-linux-gnueabihf-readelf")
    args = parser.parse_args()

    output = args.output.resolve()
    if inside(output, ROOT):
        parser.error("output must remain outside the source repository")
    if output.suffix != ".ko":
        parser.error("output must use the .ko suffix")
    if not (args.kernel_build / "include" / "generated" / "autoconf.h").is_file():
        parser.error("kernel build tree is not configured/prepared")
    output.parent.mkdir(parents=True, exist_ok=True)

    with tempfile.TemporaryDirectory(prefix="podbay-registration-canary-") as temporary:
        build = Path(temporary)
        shutil.copy2(SOURCE / "Makefile", build / "Makefile")
        shutil.copy2(SOURCE / f"{MODULE}.c", build / f"{MODULE}.c")
        subprocess.run(
            [
                "make",
                "-C",
                str(args.kernel_build.resolve()),
                f"M={build}",
                "ARCH=arm",
                f"CROSS_COMPILE={args.cross_compile}",
                "modules",
            ],
            check=True,
        )
        module = build / f"{MODULE}.ko"
        try:
            verify_module_layout(args.readelf, module)
        except (OSError, subprocess.CalledProcessError, ValueError) as error:
            parser.exit(2, f"refusing built module: {error}\n")
        shutil.copy2(module, output)
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
