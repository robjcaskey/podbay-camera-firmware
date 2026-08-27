#!/usr/bin/env python3
"""Install and manage the PW203 fail-closed one-shot boot hook."""

from __future__ import annotations

import argparse
import hashlib
import shlex
import subprocess
import sys
from pathlib import Path


BEGIN = "# PW203_ONE_SHOT_BOOT_HOOK_V1_BEGIN"
END = "# PW203_ONE_SHOT_BOOT_HOOK_V1_END"
INSERT_BEFORE = "# start camera"
BOOTAPP = "/app/bootapp"
STOCK_BACKUP = "/app/private/bootapp.stock"
HOOK_BACKUP = "/app/private/bootapp.pre-one-shot-hook"
PENDING = "/app/private/run-next-reboot.sh"
CLAIMED = "/app/private/run-next-reboot.claimed"
STAGED_HOOK = "/tmp/bootapp.one-shot-hook.block"
STAGED_PAYLOAD = "/tmp/run-next-reboot.next"
EXPECTED_STOCK_BOOTAPP_SHA256 = (
    "88dc5743f3223ad5c410e4e301b83f526f3b88f108e6e143ab9de521c036c672"
)
EXPECTED_HOOK_BLOCK_SHA256 = (
    "191e13c8dfdd9741dd0e2d1ee475e407442db184ce4ffc35c7c8e0b8f6871316"
)
COMPATIBLE_PREVIOUS_HOOKED_BOOTAPP_SHA256 = (
    "f592bf6c04af7c4c41085eef22f3717b63d72a14c6d05a9387be6a981e5940c0"
)

HOOK = f"""{BEGIN}
PW203_ONCE_PENDING={PENDING}
PW203_ONCE_CLAIMED={CLAIMED}
PW203_ONCE_RUNTIME=/tmp/run-next-reboot.sh
if [ -f "$PW203_ONCE_PENDING" ]; then
    if /bin/mv "$PW203_ONCE_PENDING" "$PW203_ONCE_CLAIMED"; then
        /bin/sync
        if /bin/cp "$PW203_ONCE_CLAIMED" "$PW203_ONCE_RUNTIME"; then
            /bin/chmod 700 "$PW203_ONCE_RUNTIME"
            /bin/rm -f "$PW203_ONCE_CLAIMED"
            /bin/sync
            /bin/sh "$PW203_ONCE_RUNTIME"
        fi
    fi
fi
{END}
"""


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


class Camera:
    def __init__(self, host: str, key: str) -> None:
        self.ssh = [
            "ssh",
            "-F",
            "/dev/null",
            "-o",
            "BatchMode=yes",
            "-o",
            "ConnectTimeout=5",
            "-o",
            "StrictHostKeyChecking=no",
            "-o",
            "UserKnownHostsFile=/dev/null",
            "-i",
            key,
            f"root@{host}",
        ]

    def run(self, command: str, *, data: bytes | None = None) -> bytes:
        result = subprocess.run(
            [*self.ssh, command],
            input=data,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        if result.returncode:
            detail = result.stderr.decode(errors="replace").strip()
            raise RuntimeError(f"remote command failed ({result.returncode}): {detail}")
        return result.stdout

    def read(self, path: str) -> bytes:
        return self.run(f"cat {shlex.quote(path)}")

    def upload(self, path: str, data: bytes) -> None:
        self.run(f"cat > {shlex.quote(path)}", data=data)


def patched_bootapp(stock: bytes) -> bytes:
    text = stock.decode()
    if text.count(INSERT_BEFORE) != 1:
        raise ValueError(f"expected exactly one {INSERT_BEFORE!r} insertion point")
    return text.replace(INSERT_BEFORE, HOOK + "\n" + INSERT_BEFORE).encode()


def expected_hooked_hash(stock: bytes) -> str:
    hook_hash = sha256(HOOK.encode())
    if hook_hash != EXPECTED_HOOK_BLOCK_SHA256:
        raise RuntimeError(
            "refusing operation: rendered PW203 hook checksum is not pinned "
            f"({hook_hash} != {EXPECTED_HOOK_BLOCK_SHA256})"
        )
    return sha256(patched_bootapp(stock))


def install(camera: Camera) -> None:
    current = camera.read(BOOTAPP)
    stock = camera.read(STOCK_BACKUP)
    current_hash = sha256(current)
    stock_hash = sha256(stock)
    if stock_hash != EXPECTED_STOCK_BOOTAPP_SHA256:
        raise RuntimeError(
            "refusing install: bootapp.stock checksum is not the pinned stock "
            f"checksum ({stock_hash} != {EXPECTED_STOCK_BOOTAPP_SHA256})"
        )

    patched = patched_bootapp(stock)
    patched_hash = expected_hooked_hash(stock)
    if current_hash == patched_hash:
        print("one-shot boot hook already installed and checksum-valid; no change")
        return
    if current_hash == COMPATIBLE_PREVIOUS_HOOKED_BOOTAPP_SHA256:
        print("compatible previous one-shot boot hook is checksum-valid; no change")
        return
    if current_hash != EXPECTED_STOCK_BOOTAPP_SHA256:
        raise RuntimeError(
            "refusing install: live bootapp checksum is not the pinned stock "
            f"checksum ({current_hash} != {EXPECTED_STOCK_BOOTAPP_SHA256})"
        )

    hook_bytes = HOOK.encode()
    hook_hash = sha256(hook_bytes)
    camera.upload(STAGED_HOOK, hook_bytes)
    script = f"""
set -eu
current=$(sha256sum {BOOTAPP} | awk '{{print $1}}')
stock=$(sha256sum {STOCK_BACKUP} | awk '{{print $1}}')
hook=$(sha256sum {STAGED_HOOK} | awk '{{print $1}}')
[ "$current" = {current_hash} ]
[ "$stock" = {stock_hash} ]
[ "$hook" = {hook_hash} ]
[ "$(grep -c '^# start camera$' {BOOTAPP})" = 1 ]
mount -o remount,rw /
trap 'mount -o remount,ro / >/dev/null 2>&1 || true' EXIT
if [ ! -f {HOOK_BACKUP} ]; then
    cp -p {BOOTAPP} {HOOK_BACKUP}
    sync
fi
/usr/bin/awk -v hook={STAGED_HOOK} '
$0 == "# start camera" {{
    while ((getline line < hook) > 0) print line
    close(hook)
    print ""
}}
{{ print }}
' {BOOTAPP} > {BOOTAPP}.next
chmod 700 {BOOTAPP}.next
[ "$(sha256sum {BOOTAPP}.next | awk '{{print $1}}')" = {patched_hash} ]
mv -f {BOOTAPP}.next {BOOTAPP}
sync
mount -o remount,ro /
trap - EXIT
installed=$(sha256sum {BOOTAPP} | awk '{{print $1}}')
if [ "$installed" != {patched_hash} ]; then
    echo "FATAL: installed bootapp checksum mismatch; DO NOT REBOOT" >&2
    echo "expected={patched_hash} actual=$installed" >&2
    exit 90
fi
"""
    camera.run(script)
    print(f"installed one-shot boot hook sha256={patched_hash}")
    print(f"no payload queued; {PENDING} is absent unless added later")


def status(camera: Camera) -> None:
    current = camera.read(BOOTAPP)
    stock = camera.read(STOCK_BACKUP)
    stock_hash = sha256(stock)
    current_hash = sha256(current)
    expected_hooked = (
        expected_hooked_hash(stock)
        if stock_hash == EXPECTED_STOCK_BOOTAPP_SHA256
        else None
    )
    if current_hash == expected_hooked:
        hook_state = "installed"
    elif current_hash == COMPATIBLE_PREVIOUS_HOOKED_BOOTAPP_SHA256:
        hook_state = "installed-compatible-previous"
    elif current_hash == EXPECTED_STOCK_BOOTAPP_SHA256:
        hook_state = "absent"
    else:
        hook_state = "unexpected-checksum"
    print(f"bootapp_sha256={current_hash}")
    print(f"expected_stock_sha256={EXPECTED_STOCK_BOOTAPP_SHA256}")
    print(f"expected_hooked_sha256={expected_hooked or 'unavailable-stock-mismatch'}")
    print(f"hook={hook_state}")
    script = f"""
set -u
[ -e {PENDING} ] && echo pending=yes || echo pending=no
[ -e {CLAIMED} ] && echo claimed=yes || echo claimed=no
mount | grep ' on / '
"""
    sys.stdout.buffer.write(camera.run(script))


def queue(camera: Camera, payload_path: str) -> None:
    payload = Path(payload_path).read_bytes()
    if not payload:
        raise RuntimeError("refusing to queue an empty payload")
    stock = camera.read(STOCK_BACKUP)
    stock_hash = sha256(stock)
    if stock_hash != EXPECTED_STOCK_BOOTAPP_SHA256:
        raise RuntimeError("refusing queue: bootapp.stock checksum is not pinned stock")
    current_hash = sha256(camera.read(BOOTAPP))
    expected_hooked = expected_hooked_hash(stock)
    if current_hash not in {
        expected_hooked,
        COMPATIBLE_PREVIOUS_HOOKED_BOOTAPP_SHA256,
    }:
        raise RuntimeError(
            "refusing queue: live bootapp checksum does not match exact hooked "
            f"checksums ({current_hash})"
        )
    payload_hash = sha256(payload)
    camera.upload(STAGED_PAYLOAD, payload)
    script = f"""
set -eu
[ "$(sha256sum {BOOTAPP} | awk '{{print $1}}')" = {current_hash} ]
[ ! -e {PENDING} ]
[ ! -e {CLAIMED} ]
[ "$(sha256sum {STAGED_PAYLOAD} | awk '{{print $1}}')" = {payload_hash} ]
cp {STAGED_PAYLOAD} {PENDING}.next
chmod 700 {PENDING}.next
[ "$(sha256sum {PENDING}.next | awk '{{print $1}}')" = {payload_hash} ]
mv -f {PENDING}.next {PENDING}
sync
[ "$(sha256sum {PENDING} | awk '{{print $1}}')" = {payload_hash} ]
"""
    camera.run(script)
    print(f"queued one-shot payload sha256={payload_hash}")


def uninstall(camera: Camera) -> None:
    current = camera.read(BOOTAPP)
    backup = camera.read(HOOK_BACKUP)
    stock = camera.read(STOCK_BACKUP)
    current_hash = sha256(current)
    backup_hash = sha256(backup)
    stock_hash = sha256(stock)
    if backup_hash != stock_hash:
        raise RuntimeError(
            f"refusing uninstall: hook backup {backup_hash} != stock {stock_hash}"
        )
    if current_hash == stock_hash:
        print("one-shot boot hook already absent and checksum-valid; no change")
        return
    expected_hooked = expected_hooked_hash(stock)
    if current_hash not in {
        expected_hooked,
        COMPATIBLE_PREVIOUS_HOOKED_BOOTAPP_SHA256,
    }:
        raise RuntimeError(
            "refusing uninstall: live bootapp checksum is neither exact stock "
            f"nor exact hooked checksum ({current_hash})"
        )
    script = f"""
set -eu
[ "$(sha256sum {HOOK_BACKUP} | awk '{{print $1}}')" = {backup_hash} ]
[ "$(sha256sum {STOCK_BACKUP} | awk '{{print $1}}')" = {stock_hash} ]
mount -o remount,rw /
trap 'mount -o remount,ro / >/dev/null 2>&1 || true' EXIT
cp {HOOK_BACKUP} {BOOTAPP}.next
chmod 700 {BOOTAPP}.next
[ "$(sha256sum {BOOTAPP}.next | awk '{{print $1}}')" = {stock_hash} ]
mv -f {BOOTAPP}.next {BOOTAPP}
sync
mount -o remount,ro /
trap - EXIT
"""
    camera.run(script)
    print(f"restored stock bootapp sha256={stock_hash}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="192.168.88.10")
    parser.add_argument("--key", default=str(Path.home() / ".ssh/id_ed25519"))
    subparsers = parser.add_subparsers(dest="action", required=True)
    subparsers.add_parser("install")
    subparsers.add_parser("status")
    queue_parser = subparsers.add_parser("queue")
    queue_parser.add_argument("payload")
    subparsers.add_parser("uninstall")
    args = parser.parse_args()

    camera = Camera(args.host, args.key)
    if args.action == "install":
        install(camera)
    elif args.action == "status":
        status(camera)
    elif args.action == "queue":
        queue(camera, args.payload)
    elif args.action == "uninstall":
        uninstall(camera)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
