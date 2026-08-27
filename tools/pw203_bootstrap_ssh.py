#!/usr/bin/env python3
"""Enter PW203 USB Ethernet mode and start a temporary key-only SSH service.

This host-side tool generates its USB messages, SSH host key, and camera shell
script locally. It does not contain or download camera files. It uses services
already present in the user's installed firmware; all camera changes are under
/run or /tmp and disappear on power cycle.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import socket
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request
from pathlib import Path


REPOSITORY = Path(__file__).resolve().parents[1]
SWITCHER = REPOSITORY / "tools" / "pw203_netmode.py"
NETWORK_USB_IDS = ("0525", "a4a2")
DEFAULT_KNOWN_HOSTS = Path("/tmp/podbay-pw203-known-hosts")


def run(command: list[str], **kwargs) -> subprocess.CompletedProcess:
    print("+ " + " ".join(command), file=sys.stderr, flush=True)
    kwargs.setdefault("check", True)
    return subprocess.run(command, **kwargs)


def require_program(name: str) -> None:
    if shutil.which(name) is None:
        raise RuntimeError(f"required program is not installed: {name}")


def read_text(path: Path) -> str | None:
    try:
        return path.read_text(encoding="utf-8").strip()
    except OSError:
        return None


def usb_ids_for_interface(interface: str) -> tuple[str, str] | None:
    link = Path("/sys/class/net") / interface / "device"
    try:
        device = link.resolve(strict=True)
    except OSError:
        return None
    for candidate in (device, *device.parents):
        vendor = read_text(candidate / "idVendor")
        product = read_text(candidate / "idProduct")
        if vendor and product:
            return vendor.lower(), product.lower()
    return None


def network_interfaces() -> list[str]:
    root = Path("/sys/class/net")
    if not root.is_dir():
        return []
    return sorted(
        candidate.name
        for candidate in root.iterdir()
        if usb_ids_for_interface(candidate.name) == NETWORK_USB_IDS
    )


def wait_for_interface(seconds: float) -> str:
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        candidates = network_interfaces()
        if len(candidates) == 1:
            return candidates[0]
        if len(candidates) > 1:
            raise RuntimeError(
                "multiple matching USB Ethernet interfaces are present; pass --interface"
            )
        time.sleep(0.25)
    raise RuntimeError("PW203 USB Ethernet interface did not appear")


def privileged(args: argparse.Namespace, command: list[str]) -> list[str]:
    if os.geteuid() == 0 or args.no_sudo:
        return command
    return [args.sudo, *command]


def interface_has_address(interface: str, cidr: str) -> bool:
    result = subprocess.run(
        ["ip", "-o", "-4", "address", "show", "dev", interface],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    )
    return any(len(line.split()) > 3 and line.split()[3] == cidr for line in result.stdout.splitlines())


def configure_interface(args: argparse.Namespace, interface: str) -> None:
    cidr = f"{args.host_ip}/{args.prefix_length}"
    run(privileged(args, ["ip", "link", "set", interface, "up"]))
    if not interface_has_address(interface, cidr):
        run(privileged(args, ["ip", "address", "add", cidr, "dev", interface]))
    run(privileged(args, ["ip", "neighbour", "flush", "dev", interface]), check=False)


def configure_selected_interface(args: argparse.Namespace, interface: str) -> str:
    try:
        configure_interface(args, interface)
        return interface
    except subprocess.CalledProcessError:
        # A newly enumerated USB NIC can be observed as usb0 just before udev
        # applies its stable enx* name. Retry only when the selected sysfs
        # interface actually disappeared; permission/configuration failures
        # must still fail closed.
        if (Path("/sys/class/net") / interface).is_dir():
            raise
        replacement = wait_for_interface(args.wait_interface_seconds)
        if replacement == interface:
            raise
        configure_interface(args, replacement)
        return replacement


def socket_open(host: str, port: int, timeout: float = 1.0) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def preflight_normal_uvc(args: argparse.Namespace) -> None:
    owner = subprocess.run(
        ["fuser", args.video_device],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        timeout=args.uvc_query_seconds,
    )
    if owner.returncode == 0:
        detail = " ".join((owner.stdout + " " + owner.stderr).split())
        raise RuntimeError(f"refusing USB mode switch: {args.video_device} is owned ({detail})")
    if owner.returncode != 1:
        raise RuntimeError(
            f"cannot establish ownership state for {args.video_device}: "
            f"fuser exit {owner.returncode}"
        )
    try:
        query = subprocess.run(
            ["v4l2-ctl", "-d", args.video_device, "--all"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=args.uvc_query_seconds,
        )
    except subprocess.TimeoutExpired as exc:
        raise RuntimeError(
            f"refusing USB mode switch: bounded UVC query timed out after "
            f"{args.uvc_query_seconds:g}s"
        ) from exc
    if query.returncode:
        detail = query.stderr.strip() or query.stdout.strip() or "no diagnostic"
        raise RuntimeError(f"refusing USB mode switch: stock UVC query failed: {detail}")


def request_json(method: str, url: str, body: bytes | None = None, timeout: float = 5.0) -> dict:
    headers = {"Content-Type": "text/plain"} if body is not None else {}
    request = urllib.request.Request(url, data=body, headers=headers, method=method)
    with urllib.request.urlopen(request, timeout=timeout) as response:
        return json.loads(response.read().decode("utf-8", "replace"))


def wait_for_http(args: argparse.Namespace) -> None:
    url = f"http://{args.device_ip}:{args.http_port}/query_device_info"
    deadline = time.monotonic() + args.wait_http_seconds
    while time.monotonic() < deadline:
        try:
            response = request_json("GET", url, timeout=2.0)
            if response.get("errorcode") == "200":
                return
        except (OSError, ValueError, urllib.error.URLError):
            pass
        time.sleep(0.5)
    raise RuntimeError(f"installed firmware service did not answer {url}")


def read_public_key(path: Path) -> str:
    try:
        first_line = path.read_text(encoding="utf-8").splitlines()[0].strip()
    except (OSError, IndexError) as exc:
        raise RuntimeError(f"cannot read public key: {path}") from exc
    if not (first_line.startswith("ssh-") or first_line.startswith("ecdsa-")):
        raise RuntimeError(f"file does not contain an OpenSSH public key: {path}")
    return first_line


def make_host_key(directory: Path) -> tuple[str, str]:
    key = directory / "ssh_host_ed25519_key"
    run(["ssh-keygen", "-q", "-t", "ed25519", "-N", "", "-f", str(key)])
    public_key = run(
        ["ssh-keygen", "-y", "-f", str(key)],
        stdout=subprocess.PIPE,
        text=True,
    ).stdout.strip()
    return key.read_text(encoding="utf-8"), public_key


def write_known_host(path: Path, host: str, public_key: str) -> None:
    if not public_key.startswith("ssh-ed25519 "):
        raise RuntimeError("generated SSH host public key is not Ed25519")
    path.parent.mkdir(parents=True, exist_ok=True)
    staged = path.with_name(path.name + ".next")
    staged.write_text(f"{host} {public_key}\n", encoding="utf-8")
    staged.chmod(0o600)
    staged.replace(path)


def verify_ssh(args: argparse.Namespace) -> None:
    run(
        [
            "ssh",
            "-F",
            "/dev/null",
            "-o",
            "BatchMode=yes",
            "-o",
            "ConnectTimeout=5",
            "-o",
            "StrictHostKeyChecking=yes",
            "-o",
            f"UserKnownHostsFile={args.known_hosts}",
            "-i",
            str(args.private_key),
            f"root@{args.device_ip}",
            "true",
        ]
    )


def shell_payload(public_key: str, host_private_key: str) -> bytes:
    script = f"""#!/bin/sh
set -eu
umask 077
mkdir -p /run/pw203-ssh /run/sshd
cat > /run/pw203-ssh/authorized_keys <<'__PW203_PUBLIC_KEY__'
{public_key}
__PW203_PUBLIC_KEY__
cat > /run/pw203-ssh/ssh_host_ed25519_key <<'__PW203_HOST_KEY__'
{host_private_key.rstrip()}
__PW203_HOST_KEY__
chmod 600 /run/pw203-ssh/authorized_keys /run/pw203-ssh/ssh_host_ed25519_key
mkdir -p /run/.ssh
cp /run/pw203-ssh/authorized_keys /run/.ssh/authorized_keys
chmod 700 /run/.ssh
chmod 600 /run/.ssh/authorized_keys
/usr/sbin/sshd -t -f /etc/ssh/sshd_config -h /run/pw203-ssh/ssh_host_ed25519_key
/usr/sbin/sshd -f /etc/ssh/sshd_config -h /run/pw203-ssh/ssh_host_ed25519_key -E /tmp/pw203-sshd.log
"""
    return script.encode("utf-8")


def start_ssh(args: argparse.Namespace, public_key: str, host_private_key: str) -> None:
    url = f"http://{args.device_ip}:{args.http_port}/run_shell_script"
    response = request_json(
        "POST",
        url,
        shell_payload(public_key, host_private_key),
        timeout=args.http_timeout_seconds,
    )
    if response.get("errorcode") != "200" or response.get("runcode") != 0:
        raise RuntimeError(f"temporary SSH bootstrap was rejected: {response!r}")


def wait_for_ssh(args: argparse.Namespace) -> None:
    deadline = time.monotonic() + args.wait_ssh_seconds
    while time.monotonic() < deadline:
        if socket_open(args.device_ip, 22):
            return
        time.sleep(0.5)
    raise RuntimeError("temporary SSH service did not open port 22")


def switch_to_network(args: argparse.Namespace) -> None:
    command = [
        sys.executable,
        str(SWITCHER),
        "--device-ip",
        args.device_ip,
        "--timeout-ms",
        str(args.usb_timeout_ms),
        "--pause-ms",
        str(args.usb_pause_ms),
        "--accept-camera-mode-switch",
    ]
    run(privileged(args, command))


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--device-ip", default="192.168.88.10")
    result.add_argument("--host-ip", default="192.168.88.20")
    result.add_argument("--prefix-length", type=int, default=24)
    result.add_argument("--http-port", type=int, default=10086)
    result.add_argument("--video-device", default="/dev/video0")
    result.add_argument("--uvc-query-seconds", type=float, default=5.0)
    result.add_argument("--interface")
    result.add_argument("--public-key", type=Path, default=Path("~/.ssh/id_ed25519.pub"))
    result.add_argument("--private-key", type=Path, default=Path("~/.ssh/id_ed25519"))
    result.add_argument("--known-hosts", type=Path, default=DEFAULT_KNOWN_HOSTS)
    result.add_argument("--sudo", default="sudo")
    result.add_argument("--no-sudo", action="store_true")
    result.add_argument("--usb-timeout-ms", type=int, default=1500)
    result.add_argument("--usb-pause-ms", type=int, default=300)
    result.add_argument("--wait-interface-seconds", type=float, default=30.0)
    result.add_argument("--wait-http-seconds", type=float, default=60.0)
    result.add_argument("--http-timeout-seconds", type=float, default=10.0)
    result.add_argument("--wait-ssh-seconds", type=float, default=30.0)
    result.add_argument("--dry-run", action="store_true")
    result.add_argument(
        "--accept-camera-bootstrap",
        action="store_true",
        help="acknowledge temporary USB mode and camera process changes",
    )
    return result


def main() -> int:
    args = parser().parse_args()
    args.public_key = args.public_key.expanduser().resolve()
    args.private_key = args.private_key.expanduser().resolve()
    args.known_hosts = args.known_hosts.expanduser().resolve()
    if args.dry_run:
        print("1. require an unowned video device and a bounded healthy stock UVC query")
        print("2. require one unambiguous normal-mode USB identity")
        print("3. generate the complete field-level USB RPC sequence")
        print("4. switch the camera to temporary USB Ethernet mode with bounded transfers")
        print("5. configure the unique selected host interface")
        print("6. generate a temporary SSH host key and upload a key-only startup script")
        print("7. pin that key in an isolated temporary trust file and verify strict SSH")
        print("8. no camera file is downloaded or persisted")
        return 0
    if not args.accept_camera_bootstrap:
        raise SystemExit("refusing camera bootstrap without --accept-camera-bootstrap")

    if args.uvc_query_seconds <= 0:
        raise SystemExit("--uvc-query-seconds must be positive")
    for program in ("ip", "ssh-keygen", "fuser", "v4l2-ctl"):
        require_program(program)
    if os.geteuid() != 0 and not args.no_sudo:
        require_program(args.sudo)
    public_key = read_public_key(args.public_key)

    try:
        candidates = network_interfaces()
        if args.interface:
            interface = args.interface
        elif len(candidates) == 1:
            interface = candidates[0]
        else:
            if len(candidates) > 1:
                raise RuntimeError("multiple matching USB Ethernet interfaces; pass --interface")
            preflight_normal_uvc(args)
            switch_to_network(args)
            interface = wait_for_interface(args.wait_interface_seconds)
        interface = configure_selected_interface(args, interface)
        wait_for_http(args)
        if not socket_open(args.device_ip, 22):
            with tempfile.TemporaryDirectory(prefix="pw203-ssh.") as temporary:
                host_private_key, host_public_key = make_host_key(Path(temporary))
                start_ssh(args, public_key, host_private_key)
                write_known_host(args.known_hosts, args.device_ip, host_public_key)
            wait_for_ssh(args)
        elif not args.known_hosts.is_file():
            raise RuntimeError(
                "temporary SSH is already open but its isolated host-key trust record is absent; "
                "return the camera to stock mode and rerun bootstrap"
            )
        verify_ssh(args)
    except (OSError, RuntimeError, subprocess.CalledProcessError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    print(
        "SSH ready: ssh -F /dev/null -o StrictHostKeyChecking=yes "
        f"-o UserKnownHostsFile={args.known_hosts} "
        f"-i {args.private_key} root@{args.device_ip}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
