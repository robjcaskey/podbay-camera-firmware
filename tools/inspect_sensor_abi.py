#!/usr/bin/env python3
"""Report bounded ELF ABI facts from an owner-supplied PW203 sensor module.

This tool hashes the input and reports metadata, section sizes, and undefined
symbol names. It does not copy section contents, instructions, or register data.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import struct
from dataclasses import dataclass
from pathlib import Path


ELF32_HEADER = struct.Struct("<16sHHIIIIIHHHHHH")
ELF32_SECTION = struct.Struct("<IIIIIIIIII")
ELF32_SYMBOL = struct.Struct("<IIIBBH")
SHT_SYMTAB = 2
SHN_UNDEF = 0
EXPECTED_MACHINE = 40  # EM_ARM

SENSOR_ABI_SYMBOLS = (
    "DrvRegisterSensorDriverEx",
    "DrvRegisterSensorI2CSlaveID",
    "DrvSensorHandleVer",
    "DrvSensorIFVer",
    "DrvSensorI2CVer",
    "DrvSensorRelease",
)


@dataclass(frozen=True)
class Section:
    name: str
    section_type: int
    offset: int
    size: int
    link: int
    entry_size: int


def terminated(data: bytes, offset: int) -> str:
    if offset < 0 or offset >= len(data):
        raise ValueError(f"string offset {offset} outside table")
    end = data.find(b"\0", offset)
    if end < 0:
        raise ValueError("unterminated ELF string")
    return data[offset:end].decode("ascii", errors="strict")


def parse_sections(image: bytes) -> tuple[int, list[Section]]:
    if len(image) < ELF32_HEADER.size:
        raise ValueError("file is shorter than an ELF32 header")
    header = ELF32_HEADER.unpack_from(image)
    ident = header[0]
    if ident[:4] != b"\x7fELF" or ident[4] != 1 or ident[5] != 1:
        raise ValueError("expected a little-endian ELF32 object")
    machine = header[2]
    section_offset = header[6]
    section_entry_size = header[11]
    section_count = header[12]
    names_index = header[13]
    if section_entry_size != ELF32_SECTION.size:
        raise ValueError(f"unexpected section-header size {section_entry_size}")
    if not 0 < section_count <= 4096 or names_index >= section_count:
        raise ValueError("invalid section table dimensions")
    table_end = section_offset + section_count * section_entry_size
    if section_offset < ELF32_HEADER.size or table_end > len(image):
        raise ValueError("section table is outside the file")

    raw = [
        ELF32_SECTION.unpack_from(image, section_offset + index * section_entry_size)
        for index in range(section_count)
    ]
    names_offset = raw[names_index][4]
    names_size = raw[names_index][5]
    if names_offset + names_size > len(image):
        raise ValueError("section-name table is outside the file")
    names = image[names_offset : names_offset + names_size]

    sections: list[Section] = []
    for item in raw:
        offset, size = item[4], item[5]
        if item[1] != 8 and offset + size > len(image):  # SHT_NOBITS has no bytes
            raise ValueError("section content is outside the file")
        sections.append(
            Section(
                name=terminated(names, item[0]),
                section_type=item[1],
                offset=offset,
                size=size,
                link=item[6],
                entry_size=item[9],
            )
        )
    return machine, sections


def undefined_symbols(image: bytes, sections: list[Section]) -> list[str]:
    names: set[str] = set()
    for section in sections:
        if section.section_type != SHT_SYMTAB:
            continue
        if section.entry_size != ELF32_SYMBOL.size or section.link >= len(sections):
            raise ValueError("invalid ELF32 symbol table")
        strings_section = sections[section.link]
        strings = image[
            strings_section.offset : strings_section.offset + strings_section.size
        ]
        for offset in range(section.offset, section.offset + section.size, section.entry_size):
            name_offset, _value, _size, _info, _other, index = ELF32_SYMBOL.unpack_from(
                image, offset
            )
            if index == SHN_UNDEF and name_offset:
                names.add(terminated(strings, name_offset))
    return sorted(names)


def module_info(image: bytes, sections: list[Section]) -> dict[str, list[str]]:
    result: dict[str, list[str]] = {}
    for section in sections:
        if section.name != ".modinfo":
            continue
        values = image[section.offset : section.offset + section.size]
        for raw in values.split(b"\0"):
            if not raw:
                continue
            text = raw.decode("ascii", errors="strict")
            key, separator, value = text.partition("=")
            if separator:
                result.setdefault(key, []).append(value)
    return result


def inspect(path: Path) -> dict[str, object]:
    image = path.read_bytes()
    machine, sections = parse_sections(image)
    symbols = undefined_symbols(image, sections)
    info = module_info(image, sections)
    symbol_set = set(symbols)
    return {
        "sha256": hashlib.sha256(image).hexdigest(),
        "size": len(image),
        "elf": {"class": 32, "endianness": "little", "machine": machine},
        "module": {
            "name": info.get("name", []),
            "vermagic": info.get("vermagic", []),
        },
        "section_sizes": {
            section.name: section.size
            for section in sections
            if section.name in {".text", ".rodata", ".data", ".bss", ".modinfo"}
        },
        "sensor_abi": {
            "required": list(SENSOR_ABI_SYMBOLS),
            "present": [name for name in SENSOR_ABI_SYMBOLS if name in symbol_set],
            "missing": [name for name in SENSOR_ABI_SYMBOLS if name not in symbol_set],
        },
        "undefined_symbols": symbols,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("module", type=Path, help="owner-supplied ELF32 sensor module")
    parser.add_argument(
        "--allow-incomplete-sensor-abi",
        action="store_true",
        help="return success even when a known sensor registration symbol is absent",
    )
    args = parser.parse_args()

    try:
        report = inspect(args.module)
    except (OSError, UnicodeError, ValueError, struct.error) as error:
        parser.exit(2, f"refusing input: {error}\n")
    print(json.dumps(report, indent=2, sort_keys=True))
    if report["elf"]["machine"] != EXPECTED_MACHINE:
        parser.exit(2, "refusing non-ARM sensor module\n")
    if report["sensor_abi"]["missing"] and not args.allow_incomplete_sensor_abi:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
