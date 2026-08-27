#!/usr/bin/env python3
"""Build the registration-free Podbay kernel canary outside the repository."""

from __future__ import annotations

import argparse
import re
import shutil
import subprocess
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
SOURCE = ROOT / "sensor-provider" / "kernel-canary"
EXPECTED_THIS_MODULE_SIZE = 0x200
EXPECTED_INIT_OFFSET = 0x11C
EXPECTED_EXIT_OFFSET = 0x1A4

THIS_MODULE = re.compile(
    r"\.gnu\.linkonce\.this_module\s+PROGBITS\s+\S+\s+\S+\s+([0-9a-fA-F]+)"
)
RELOCATION = re.compile(
    r"^([0-9a-fA-F]+)\s+\S+\s+R_ARM_ABS32\s+\S+\s+(init_module|cleanup_module)$",
    re.MULTILINE,
)


def inside(child: Path, parent: Path) -> bool:
    try:
        child.resolve().relative_to(parent.resolve())
    except ValueError:
        return False
    return True


def verify_module_layout(readelf: str, module: Path) -> None:
    sections = subprocess.run(
        [readelf, "-SW", str(module)],
        check=True,
        stdout=subprocess.PIPE,
        text=True,
    ).stdout
    match = THIS_MODULE.search(sections)
    if not match:
        raise ValueError("built module has no .gnu.linkonce.this_module section")
    size = int(match.group(1), 16)

    relocations_text = subprocess.run(
        [readelf, "-rW", str(module)],
        check=True,
        stdout=subprocess.PIPE,
        text=True,
    ).stdout
    relocations = {
        symbol: int(offset, 16)
        for offset, symbol in RELOCATION.findall(relocations_text)
    }
    expected = {
        "init_module": EXPECTED_INIT_OFFSET,
        "cleanup_module": EXPECTED_EXIT_OFFSET,
    }
    if size != EXPECTED_THIS_MODULE_SIZE or relocations != expected:
        raise ValueError(
            "kernel configuration does not match reviewed PW203 module layout: "
            f"this_module={size:#x} relocations={relocations}; "
            f"expected {EXPECTED_THIS_MODULE_SIZE:#x} and {expected}"
        )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--kernel-build", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument(
        "--cross-compile", default="arm-linux-gnueabihf-", help="compiler prefix"
    )
    parser.add_argument(
        "--readelf", default="arm-linux-gnueabihf-readelf", help="ARM ELF readelf"
    )
    args = parser.parse_args()

    output = args.output.resolve()
    if inside(output, ROOT):
        parser.error("output must remain outside the source repository")
    if output.suffix != ".ko":
        parser.error("output must use the .ko suffix")
    if not (args.kernel_build / "Makefile").is_file() or not (
        args.kernel_build / "include" / "generated" / "autoconf.h"
    ).is_file():
        parser.error("kernel build tree is not configured/prepared")
    output.parent.mkdir(parents=True, exist_ok=True)

    with tempfile.TemporaryDirectory(prefix="podbay-kernel-canary-") as temporary:
        build = Path(temporary)
        shutil.copy2(SOURCE / "Makefile", build / "Makefile")
        shutil.copy2(
            SOURCE / "podbay_kernel_canary.c", build / "podbay_kernel_canary.c"
        )
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
        module = build / "podbay_kernel_canary.ko"
        try:
            verify_module_layout(args.readelf, module)
        except (OSError, subprocess.CalledProcessError, ValueError) as error:
            parser.exit(2, f"refusing built module: {error}\n")
        shutil.copy2(module, output)
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
