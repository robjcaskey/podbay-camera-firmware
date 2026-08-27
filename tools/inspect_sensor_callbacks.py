#!/usr/bin/env python3
"""Verify PW203 SigmaStar callback offsets using ELF relocation facts.

The report contains symbol names and destination offsets only. Instructions,
literal contents, and section bytes from the owner-supplied module are neither
printed nor written by this tool.
"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
from pathlib import Path


INIT_SYMBOL = "cus_camsensor_init_handle_linear"

# Independently measured from the PW203 4.4.2.2 module identified in
# docs/SENSOR_PROVIDER.md. These are byte offsets in the framework-owned handle.
EXPECTED_CALLBACKS = {
    "pCus_poweron": 0x09C4,
    "pCus_poweroff": 0x09C8,
    "pCus_init_8m_30fps_10bit_24mhz_800mbps_with_pdaf_init_table_4lane_linear": 0x09CC,
    "cus_camsensor_release_handle": 0x09D4,
    "imx582_SetPatternMode": 0x09D8,
    "pCus_GetSensorID": 0x09DC,
    "pCus_GetVideoRes": 0x09E0,
    "pCus_GetCurVideoRes": 0x09E4,
    "pCus_SetVideoRes": 0x09E8,
    "pCus_GetOrien": 0x09EC,
    "pCus_SetOrien": 0x09F0,
    "pCus_AEStatusNotify": 0x09F8,
    "pCus_GetAEUSecs": 0x09FC,
    "pCus_SetAEUSecs": 0x0A00,
    "pCus_GetAEGain": 0x0A04,
    "pCus_SetAEGain": 0x0A08,
    "pCus_GetAEMinMaxUSecs": 0x0A0C,
    "pCus_GetAEMinMaxGain": 0x0A10,
    "pCus_GetFPS": 0x0A14,
    "pCus_SetFPS": 0x0A18,
    "IMX582_GetShutterInfo": 0x0A24,
    "pCus_GetVideoResNum": 0x0A28,
    "pCus_sensor_CustDefineFunction": 0x0A2C,
}

LOAD_LITERAL = re.compile(
    r"^\s*[0-9a-f]+:\s+.*\bldr(?:\.w)?\s+(r(?:1[0-2]|[0-9])),"
    r"\s*\[pc,\s*#\d+\].*\(([0-9a-f]+)\b"
)
STORE_HANDLE = re.compile(
    r"^\s*[0-9a-f]+:\s+.*\bstr\.w\s+(r(?:1[0-2]|[0-9])),"
    r"\s*\[r4,\s*#(\d+)\]"
)
RELOCATION = re.compile(r"^\s*([0-9a-f]+):\s+R_ARM_ABS32\s+(\S+)\s*$")


def disassemble(objdump: str, module: Path) -> str:
    command = [objdump, "-dr", f"--disassemble={INIT_SYMBOL}", str(module)]
    try:
        result = subprocess.run(
            command,
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
    except FileNotFoundError as error:
        raise ValueError(f"objdump executable not found: {objdump}") from error
    except subprocess.CalledProcessError as error:
        detail = error.stderr.strip() or f"exit status {error.returncode}"
        raise ValueError(f"objdump refused module: {detail}") from error
    return result.stdout


def callback_offsets(disassembly: str) -> dict[str, int]:
    literal_destinations: dict[int, int] = {}
    relocations: dict[int, str] = {}
    pending_literals: dict[str, int] = {}

    for line in disassembly.splitlines():
        load = LOAD_LITERAL.match(line)
        if load:
            pending_literals[load.group(1)] = int(load.group(2), 16)
            continue
        store = STORE_HANDLE.match(line)
        if store and store.group(1) in pending_literals:
            literal = pending_literals.pop(store.group(1))
            literal_destinations[literal] = int(store.group(2))
            continue
        relocation = RELOCATION.match(line)
        if relocation:
            relocations[int(relocation.group(1), 16)] = relocation.group(2)

    observed: dict[str, int] = {}
    for literal, destination in literal_destinations.items():
        symbol = relocations.get(literal)
        if symbol in EXPECTED_CALLBACKS:
            observed[symbol] = destination
    return observed


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("module", type=Path, help="owner-supplied PW203 sensor module")
    parser.add_argument(
        "--objdump", default="arm-linux-gnueabihf-objdump", help="ARM ELF objdump"
    )
    args = parser.parse_args()

    try:
        observed = callback_offsets(disassemble(args.objdump, args.module))
    except (OSError, UnicodeError, ValueError) as error:
        parser.exit(2, f"refusing input: {error}\n")

    missing = sorted(set(EXPECTED_CALLBACKS) - set(observed))
    mismatched = {
        name: {"expected": EXPECTED_CALLBACKS[name], "observed": observed[name]}
        for name in sorted(observed)
        if observed[name] != EXPECTED_CALLBACKS[name]
    }
    report = {
        "init_symbol": INIT_SYMBOL,
        "minimum_handle_bytes": max(EXPECTED_CALLBACKS.values()) + 4,
        "callbacks": {name: observed[name] for name in sorted(observed)},
        "missing": missing,
        "mismatched": mismatched,
        "verified": not missing and not mismatched,
    }
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["verified"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
