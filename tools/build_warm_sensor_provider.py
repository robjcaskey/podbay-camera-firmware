#!/usr/bin/env python3
"""Build the source-authored warm sensor provider outside the repository."""

from __future__ import annotations

import argparse
import shutil
import subprocess
import tempfile
from pathlib import Path

from build_kernel_canary import ROOT, inside, verify_module_layout


SOURCE = ROOT / "sensor-provider" / "warm-provider"
MODULE = "podbay_imx582_warm"
ALLOWED_UNDEFINED_SYMBOLS = {
    "DrvRegisterSensorDriverEx",
    "DrvRegisterSensorI2CSlaveID",
    "DrvSensorHandleVer",
    "DrvSensorI2CVer",
    "DrvSensorIFVer",
    "DrvSensorRelease",
    "__aeabi_unwind_cpp_pr0",
    "memset",
    "printk",
    "strscpy",
}


def verify_imports(readelf: str, module: Path) -> None:
    symbols = subprocess.run(
        [readelf, "-Ws", str(module)],
        check=True,
        stdout=subprocess.PIPE,
        text=True,
    ).stdout
    undefined = {
        fields[-1]
        for line in symbols.splitlines()
        if len(fields := line.split()) >= 8 and fields[6] == "UND" and fields[-1]
    }
    unexpected = sorted(undefined - ALLOWED_UNDEFINED_SYMBOLS)
    missing_vendor = sorted(
        symbol
        for symbol in ALLOWED_UNDEFINED_SYMBOLS
        if symbol.startswith("Drv") and symbol not in undefined
    )
    if unexpected or missing_vendor:
        raise ValueError(
            f"unexpected imports={unexpected}, missing vendor imports={missing_vendor}"
        )


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

    with tempfile.TemporaryDirectory(prefix="podbay-warm-provider-") as temporary:
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
            verify_imports(args.readelf, module)
        except (OSError, subprocess.CalledProcessError, ValueError) as error:
            parser.exit(2, f"refusing built module: {error}\n")
        shutil.copy2(module, output)
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
