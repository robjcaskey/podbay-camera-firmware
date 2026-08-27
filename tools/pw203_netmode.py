#!/usr/bin/env python3
"""Switch a PW203 into temporary USB Ethernet mode with bounded transfers.

Every RPC message is assembled from named protocol fields and computed CRCs.
This file contains no captured USB payloads. Nothing is written to persistent
camera storage. A power cycle returns the camera to its ordinary USB mode.
"""

from __future__ import annotations

import argparse
import importlib
import ipaddress
import struct
import sys
import time
from dataclasses import dataclass
from typing import Any, Callable


NORMAL_VID = 0x3564
NORMAL_PID = 0xFEFB
NETWORK_VID = 0x0525
NETWORK_PID = 0xA4A2
INTERFACE = 0
ENTITY = 2
RPC_SELECTOR = 2
INFO_SELECTOR = 6
GET_CUR = 0x81
SET_CUR = 0x01
TRANSFER_SIZE = 60


@dataclass(frozen=True)
class Step:
    label: str
    selector: int
    request: int
    message: bytes | None = None
    read_after: bool = False


def crc16(data: bytes) -> int:
    """Return the complemented Modbus-form CRC used by the PW203 RPC."""

    value = 0xFFFF
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ 0xA001 if value & 1 else value >> 1
    return (~value) & 0xFFFF


def packed_command(command_set: int, command_id: int) -> int:
    if not 0 <= command_set < 64:
        raise ValueError("command_set must fit in six bits")
    if not 0 <= command_id < 1024:
        raise ValueError("command_id must fit in ten bits")
    return command_set | (command_id << 6)


def rpc_header(
    *,
    frame_type: int,
    sequence: int,
    sender: int,
    receiver: int,
    command_set: int,
    command_id: int,
) -> bytes:
    header = bytearray(12)
    header[0] = 0xAA
    header[1] = frame_type
    struct.pack_into("<H", header, 2, sequence)
    struct.pack_into("<H", header, 4, len(header))
    header[8] = sender
    header[9] = receiver
    struct.pack_into("<H", header, 10, packed_command(command_set, command_id))
    struct.pack_into("<H", header, 6, crc16(header))
    return bytes(header)


def rpc_message(
    *,
    frame_type: int,
    sequence: int,
    sender: int,
    receiver: int,
    command_set: int,
    command_id: int,
    payload: bytes | None = None,
) -> bytes:
    header = rpc_header(
        frame_type=frame_type,
        sequence=sequence,
        sender=sender,
        receiver=receiver,
        command_set=command_set,
        command_id=command_id,
    )
    logical = bytearray(header)
    if payload is not None:
        payload_block = bytearray(4 + len(payload))
        struct.pack_into("<H", payload_block, 0, len(payload))
        payload_block[4:] = payload
        struct.pack_into("<H", payload_block, 2, crc16(payload_block))
        logical.extend(payload_block)
    if len(logical) > TRANSFER_SIZE:
        raise ValueError("RPC message exceeds the extension-unit transfer size")
    return bytes(logical).ljust(TRANSFER_SIZE, b"\0")


def handshake_steps(device_ip: str) -> list[Step]:
    packed_ip = ipaddress.IPv4Address(device_ip).packed[::-1]

    def query(
        label: str,
        sequence: int,
        receiver: int,
        command_set: int,
        command_id: int,
    ) -> Step:
        return Step(
            label,
            RPC_SELECTOR,
            SET_CUR,
            rpc_message(
                frame_type=0x01,
                sequence=sequence,
                sender=10,
                receiver=receiver,
                command_set=command_set,
                command_id=command_id,
            ),
            read_after=True,
        )

    return [
        query("device identity", 0, 13, 8, 96),
        query("firmware version", 1, 13, 8, 16),
        query("device information", 2, 13, 8, 101),
        query("serial number", 3, 13, 8, 99),
        Step("extension capability", INFO_SELECTOR, GET_CUR),
        query("current USB mode", 4, 2, 2, 385),
        Step(
            "prepare USB mode",
            RPC_SELECTOR,
            SET_CUR,
            rpc_message(
                frame_type=0x25,
                sequence=5,
                sender=10,
                receiver=2,
                command_set=2,
                command_id=386,
                payload=struct.pack("<I", 1),
            ),
        ),
        Step(
            "set address and enter USB Ethernet",
            RPC_SELECTOR,
            SET_CUR,
            rpc_message(
                frame_type=0x25,
                sequence=6,
                sender=10,
                receiver=13,
                command_set=8,
                command_id=100,
                payload=packed_ip + b"\x02",
            ),
        ),
    ]


def import_pyusb() -> tuple[Any, Any]:
    try:
        return importlib.import_module("usb.core"), importlib.import_module("usb.util")
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "PyUSB is not installed; install it explicitly using your operating system's "
            "trusted package source"
        ) from exc


def xu_index(entity: int, interface: int) -> int:
    return (entity << 8) | interface


def xu_get(
    device: Any,
    *,
    interface: int,
    entity: int,
    selector: int,
    timeout_ms: int,
) -> bytes:
    return bytes(
        device.ctrl_transfer(
            0xA1,
            GET_CUR,
            selector << 8,
            xu_index(entity, interface),
            TRANSFER_SIZE,
            timeout=timeout_ms,
        )
    )


def xu_set(
    device: Any,
    message: bytes,
    *,
    interface: int,
    entity: int,
    selector: int,
    timeout_ms: int,
) -> int:
    return int(
        device.ctrl_transfer(
            0x21,
            SET_CUR,
            selector << 8,
            xu_index(entity, interface),
            message,
            timeout=timeout_ms,
        )
    )


def wait_for_network_gadget(usb_core: Any, seconds: float) -> bool:
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if usb_core.find(idVendor=NETWORK_VID, idProduct=NETWORK_PID) is not None:
            return True
        time.sleep(0.1)
    return False


def run_handshake(
    device: Any,
    usb_core: Any,
    *,
    device_ip: str,
    interface: int,
    entity: int,
    timeout_ms: int,
    pause_ms: int,
    wait_network_seconds: float,
    emit: Callable[[str], None] = print,
) -> None:
    steps = handshake_steps(device_ip)
    for index, step in enumerate(steps):
        if step.request == GET_CUR:
            reply = xu_get(
                device,
                interface=interface,
                entity=entity,
                selector=step.selector,
                timeout_ms=timeout_ms,
            )
            if len(reply) != TRANSFER_SIZE:
                raise RuntimeError(f"{step.label}: short read ({len(reply)} bytes)")
            emit(f"{step.label}: read {len(reply)} bytes")
        else:
            assert step.message is not None
            try:
                written = xu_set(
                    device,
                    step.message,
                    interface=interface,
                    entity=entity,
                    selector=step.selector,
                    timeout_ms=timeout_ms,
                )
            except usb_core.USBError:
                if index == len(steps) - 1 and wait_for_network_gadget(
                    usb_core, wait_network_seconds
                ):
                    emit(f"{step.label}: camera re-enumerated during transfer completion")
                    return
                raise
            if written != TRANSFER_SIZE:
                raise RuntimeError(f"{step.label}: short write ({written} bytes)")
            emit(f"{step.label}: wrote {written} bytes")
            if step.read_after:
                reply = xu_get(
                    device,
                    interface=interface,
                    entity=entity,
                    selector=step.selector,
                    timeout_ms=timeout_ms,
                )
                if len(reply) != TRANSFER_SIZE:
                    raise RuntimeError(f"{step.label}: short response ({len(reply)} bytes)")
                emit(f"{step.label}: read {len(reply)} bytes")
        if pause_ms:
            time.sleep(pause_ms / 1000.0)


def run_switch(args: argparse.Namespace) -> None:
    usb_core, usb_util = import_pyusb()
    normal_devices = list(
        usb_core.find(find_all=True, idVendor=args.vid, idProduct=args.pid) or []
    )
    network_devices = list(
        usb_core.find(find_all=True, idVendor=NETWORK_VID, idProduct=NETWORK_PID) or []
    )
    if len(normal_devices) != 1:
        raise RuntimeError(
            f"expected exactly one PW203 normal-mode USB device "
            f"({args.vid:04x}:{args.pid:04x}), found {len(normal_devices)}"
        )
    if network_devices:
        raise RuntimeError(
            "refusing ambiguous USB state: normal PW203 and USB Ethernet "
            f"identities are both present ({len(network_devices)} network device(s))"
        )
    device = normal_devices[0]

    detached = False
    claimed = False
    try:
        try:
            if device.is_kernel_driver_active(args.interface):
                device.detach_kernel_driver(args.interface)
                detached = True
        except (NotImplementedError, usb_core.USBError):
            pass
        usb_util.claim_interface(device, args.interface)
        claimed = True
        run_handshake(
            device,
            usb_core,
            device_ip=args.device_ip,
            interface=args.interface,
            entity=args.entity,
            timeout_ms=args.timeout_ms,
            pause_ms=args.pause_ms,
            wait_network_seconds=args.wait_network_seconds,
        )
    finally:
        if claimed:
            try:
                usb_util.release_interface(device, args.interface)
            except usb_core.USBError:
                pass
        if detached:
            try:
                device.attach_kernel_driver(args.interface)
            except usb_core.USBError:
                pass
        usb_util.dispose_resources(device)


def parse_int(value: str) -> int:
    return int(value, 0)


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--vid", type=parse_int, default=NORMAL_VID)
    result.add_argument("--pid", type=parse_int, default=NORMAL_PID)
    result.add_argument("--interface", type=int, default=INTERFACE)
    result.add_argument("--entity", type=int, default=ENTITY)
    result.add_argument("--device-ip", default="192.168.88.10")
    result.add_argument("--timeout-ms", type=int, default=750)
    result.add_argument("--pause-ms", type=int, default=120)
    result.add_argument("--wait-network-seconds", type=float, default=5.0)
    result.add_argument("--dry-run", action="store_true")
    result.add_argument(
        "--accept-camera-mode-switch",
        action="store_true",
        help="acknowledge the temporary USB mode change",
    )
    return result


def main() -> int:
    args = parser().parse_args()
    if args.timeout_ms <= 0 or args.pause_ms < 0:
        raise SystemExit("timeout must be positive and pause must be non-negative")
    if args.dry_run:
        for step in handshake_steps(args.device_ip):
            action = "read" if step.request == GET_CUR else "generated write"
            suffix = " plus response read" if step.read_after else ""
            print(f"{step.label}: selector={step.selector}, {action}{suffix}")
        return 0
    if not args.accept_camera_mode_switch:
        raise SystemExit("refusing USB mode change without --accept-camera-mode-switch")
    try:
        run_switch(args)
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
