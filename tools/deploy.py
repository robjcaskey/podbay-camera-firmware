#!/usr/bin/env python3
"""Build and temporarily deploy Podbay using inputs read from an owned PW203.

This program never uses the web and never places camera-provided files in the
repository. It requires an already reachable SSH service on the camera. Four
link libraries and the immutable stock sensor module are read into a private
temporary directory, checked against pinned PW203 firmware hashes, used only
for linking, identity verification, and rollback validation, then deleted when
the operation ends.

`build` is read-only with respect to the camera and writes artifacts only to an
explicit directory outside this repository. `deploy` additionally takes over
from the firmware camera or gracefully refreshes an already-running Podbay
service, replaces the loaded sensor module temporarily, and starts the custom
service. A power cycle restores the firmware boot path.
"""

from __future__ import annotations

import argparse
import hashlib
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
from dataclasses import dataclass
from pathlib import Path


REPOSITORY = Path(__file__).resolve().parents[1]
TARGET = "armv7-unknown-linux-gnueabihf"
PROTOCOL_RESPONSE = b"OK CUSTOM_CAMERA 26\n"
STOCK_MODULE_PATH = "/lib/modules/5.10.61/extra/remo_sensor_imx582.ko"
DEFAULT_KNOWN_HOSTS = Path("/tmp/podbay-pw203-known-hosts")
WARM_PROVIDER_CANONICAL_SHA256 = (
    "e21e2015889d34939ff33895524c8260d9c77d83821bf27a218dadf3452c663a"
)
WARM_PROVIDER_VARIANTS = frozenset({"pw203-two-resolution-2024-08"})


@dataclass(frozen=True)
class FirmwareVariant:
    name: str
    inputs: dict[str, str]
    identity: dict[str, object]


@dataclass(frozen=True)
class SensorProvider:
    name: str
    loaded_module: str


SOURCE_WARM_PROVIDER = SensorProvider(
    name="source-warm",
    loaded_module="podbay_imx582_warm",
)


FIRMWARE_VARIANTS = (
    FirmwareVariant(
        name="pw203-two-resolution-2024-08",
        inputs={
            "/lib/libmi_sys.so": "30c3aeb7b0a780abb32637070c5255b6eca2a9bf02d92e9e3ee3542fcd622b39",
            "/lib/libmi_sensor.so": "5a1650291a2a08cf1097bff26650c5af1aeca353443923591e6b61c0dcd542f3",
            "/lib/libmi_vif.so": "6c910655be3d978c3766444a3495910f78cd3221747d8d686c7652693e4734c0",
            "/lib/libcam_os_wrapper.so": "dfbb1f885791f5ec889c48383d6f13a84b188ceb32e7827bff16461083b951cb",
            STOCK_MODULE_PATH: "59b042523ea9c3a855b78d6b0ea4e8fa2cacb598e23d044ca9ac4f803399ec08",
        },
        identity={
            "platform": "PW203",
            "product": "Obsbot_meet2",
            "version": "4.4.2.2",
            "branch": "OA_E",
            "socver": 1,
            "systype": 1,
            "kernel_release": "5.10.61",
        },
    ),
)
CAMERA_PATHS = tuple(FIRMWARE_VARIANTS[0].inputs)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def run(command: list[str], **kwargs) -> subprocess.CompletedProcess:
    printable = " ".join(command[:1] + ["…"] if command and command[0] == "ssh" else command)
    print(f"+ {printable}", file=sys.stderr, flush=True)
    kwargs.setdefault("check", True)
    return subprocess.run(command, **kwargs)


def require_program(name: str) -> None:
    if shutil.which(name) is None:
        raise RuntimeError(f"required program is not installed: {name}")


def ssh_command(args: argparse.Namespace) -> list[str]:
    command = [
        "ssh",
        "-F",
        "/dev/null",
        "-o",
        "BatchMode=yes",
        "-o",
        f"ConnectTimeout={args.connect_timeout}",
        "-o",
        "StrictHostKeyChecking=yes",
        "-o",
        f"UserKnownHostsFile={args.known_hosts}",
    ]
    if args.key:
        command.extend(("-i", str(args.key)))
    command.append(f"root@{args.host}")
    return command


def read_camera_file(ssh: list[str], remote: str, destination: Path) -> None:
    with destination.open("wb") as output:
        run([*ssh, f"cat {remote}"], stdout=output)


def fetch_camera_inputs(ssh: list[str], directory: Path) -> dict[str, Path]:
    fetched: dict[str, Path] = {}
    for index, remote in enumerate(CAMERA_PATHS):
        destination = directory / f"input-{index}-{Path(remote).name}"
        read_camera_file(ssh, remote, destination)
        fetched[remote] = destination
    return fetched


def read_camera_identity(args: argparse.Namespace, ssh: list[str]) -> dict[str, object]:
    url = f"http://{args.host}:{args.http_port}/query_device_info"
    try:
        with urllib.request.urlopen(url, timeout=args.connect_timeout) as response:
            document = json.load(response)
    except (OSError, ValueError, urllib.error.URLError) as error:
        raise RuntimeError(f"camera identity query failed at {url}: {error}") from error
    if document.get("errorcode") != "200":
        raise RuntimeError(f"camera identity query was rejected: errorcode={document.get('errorcode')!r}")
    safe_fields = ("platform", "product", "version", "branch", "socver", "systype")
    identity = {field: document.get(field) for field in safe_fields}
    identity["kernel_release"] = run(
        [*ssh, "uname -r"], stdout=subprocess.PIPE, text=True
    ).stdout.strip()
    return identity


def identify_variant(
    inputs: dict[str, Path], identity: dict[str, object]
) -> FirmwareVariant:
    actual = {remote: sha256(path) for remote, path in inputs.items()}
    for variant in FIRMWARE_VARIANTS:
        if actual == variant.inputs:
            if variant.identity and identity != variant.identity:
                raise RuntimeError(
                    f"firmware hashes match {variant.name} but safe identity fields do not: "
                    f"actual={identity!r} expected={variant.identity!r}"
                )
            print(f"firmware variant: {variant.name}")
            print(f"firmware identity: {identity}")
            return variant
    detail = "\n".join(f"  {remote} {digest}" for remote, digest in actual.items())
    raise RuntimeError(f"unsupported PW203 firmware input set:\n{detail}")


def ensure_toolchain() -> None:
    for program in ("cargo", "rustup"):
        require_program(program)
    configured_zig = os.environ.get("ZIG")
    if configured_zig:
        if not Path(configured_zig).is_file():
            raise RuntimeError(f"configured ZIG executable does not exist: {configured_zig}")
    else:
        require_program("zig")
    installed = run(
        ["rustup", "target", "list", "--installed"],
        stdout=subprocess.PIPE,
        text=True,
    ).stdout.splitlines()
    if TARGET not in installed:
        raise RuntimeError(
            f"Rust target {TARGET} is not installed; install it explicitly before running Podbay"
        )


def normalize_wrapper_dependency(binary: Path, wrapper: Path) -> None:
    old = str(wrapper).encode()
    new = b"libcam_os_wrapper.so"
    data = bytearray(binary.read_bytes())
    where = data.find(old + b"\0")
    if where >= 0:
        if len(new) > len(old):
            raise RuntimeError("temporary wrapper path is too short to normalize safely")
        data[where : where + len(old)] = new + b"\0" * (len(old) - len(new))
        binary.write_bytes(data)
    elif data.find(new + b"\0") < 0:
        raise RuntimeError("camera wrapper dependency was not present in the service executable")


def build_service(inputs: dict[str, Path], directory: Path) -> Path:
    ensure_toolchain()
    library_dir = directory / "link-libraries"
    library_dir.mkdir(mode=0o700)
    for remote in CAMERA_PATHS:
        if remote == STOCK_MODULE_PATH:
            continue
        shutil.copyfile(inputs[remote], library_dir / Path(remote).name)
    wrapper = library_dir / "libcam_os_wrapper.so"
    target_dir = directory / "cargo-target"
    environment = os.environ.copy()
    environment["CARGO_TARGET_DIR"] = str(target_dir)
    environment["CARGO_TARGET_ARMV7_UNKNOWN_LINUX_GNUEABIHF_LINKER"] = str(
        REPOSITORY / "tools/pw203-zigcc.sh"
    )
    environment["RUSTFLAGS"] = (
        f"-L native={library_dir} -C link-arg=-Wl,-rpath,/lib "
        "-C link-arg=-Wl,--no-as-needed -C link-arg=-lcam_os_wrapper "
        "-C link-arg=-Wl,--as-needed"
    )
    run(
        [
            "cargo",
            "build",
            "--offline",
            "--release",
            "--target",
            TARGET,
            "--manifest-path",
            str(REPOSITORY / "camera-service/Cargo.toml"),
        ],
        env=environment,
    )
    binary = target_dir / TARGET / "release" / "podbay-camera-service"
    if not binary.is_file():
        raise RuntimeError(f"Cargo did not produce {binary}")
    normalize_wrapper_dependency(binary, wrapper)
    return binary


def build_warm_sensor_module(
    kernel_build: Path | None, directory: Path, variant: FirmwareVariant
) -> Path:
    if variant.name not in WARM_PROVIDER_VARIANTS:
        raise RuntimeError(
            f"source-warm is not live-proven for firmware variant {variant.name}"
        )
    if kernel_build is None:
        raise RuntimeError("source-warm requires --kernel-build")
    kernel_build = kernel_build.expanduser().resolve()
    if not (kernel_build / "include" / "generated" / "autoconf.h").is_file():
        raise RuntimeError(f"kernel build is not configured/prepared: {kernel_build}")
    result = directory / "podbay_imx582_warm.ko"
    run(
        [
            sys.executable,
            str(REPOSITORY / "tools" / "build_warm_sensor_provider.py"),
            "--kernel-build",
            str(kernel_build),
            "--output",
            str(result),
        ]
    )
    canonical = directory / "podbay_imx582_warm.canonical.ko"
    shutil.copyfile(result, canonical)
    run(["arm-linux-gnueabihf-strip", "--strip-debug", str(canonical)])
    without_build_id = directory / "podbay_imx582_warm.no-build-id.ko"
    run(
        [
            "arm-linux-gnueabihf-objcopy",
            "--remove-section=.note.gnu.build-id",
            str(canonical),
            str(without_build_id),
        ]
    )
    canonical_digest = sha256(without_build_id)
    if canonical_digest != WARM_PROVIDER_CANONICAL_SHA256:
        raise RuntimeError(
            "source-warm module failed its canonical reproducible checksum: "
            f"{canonical_digest} != {WARM_PROVIDER_CANONICAL_SHA256}"
        )
    print(f"source-warm canonical sha256: {canonical_digest}")
    canonical.unlink()
    without_build_id.unlink()
    return result


def copy_artifacts(
    service: Path,
    module: Path,
    output_dir: Path,
    variant: FirmwareVariant,
    provider: SensorProvider,
) -> None:
    resolved = output_dir.resolve()
    if resolved == REPOSITORY or REPOSITORY in resolved.parents:
        raise RuntimeError("refusing to place generated artifacts inside the source repository")
    resolved.mkdir(parents=True, exist_ok=True)
    copied_service = resolved / "pw203-camera-service"
    shutil.copyfile(service, copied_service)
    copied_service.chmod(0o700)
    shutil.copyfile(module, resolved / "pw203-sensor.ko")
    manifest = {
        "schema": "podbay-pw203-local-build-v2",
        "firmware_variant": variant.name,
        "sensor_provider": provider.name,
        "loaded_module": provider.loaded_module,
        "service_sha256": sha256(service),
        "sensor_module_sha256": sha256(module),
        "stock_module_sha256": variant.inputs[STOCK_MODULE_PATH],
    }
    manifest["sensor_module_canonical_sha256"] = WARM_PROVIDER_CANONICAL_SHA256
    (resolved / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"artifacts: {resolved}")


def upload(ssh: list[str], source: Path, remote: str) -> str:
    digest = sha256(source)
    with source.open("rb") as payload:
        run([*ssh, f"cat > {remote}"], stdin=payload)
    return digest


def deployment_script(
    service_hash: str,
    module_hash: str,
    variant: FirmwareVariant,
    provider: SensorProvider,
    runtime: str,
) -> str:
    if runtime == "stock":
        quiesce = """
[ "$(readlink /proc/$(pidof camera)/exe)" = /app/bin/camera ]
grep -q '^remo_sensor_imx582 ' /proc/modules
camera_pid=$(pidof camera)
kill -KILL "$camera_pid"
camera_state=unknown
for attempt in 1 2 3 4 5; do
    if [ ! -d "/proc/$camera_pid" ]; then
        camera_state=gone
        break
    fi
    camera_state=$(awk '/^State:/{print $2}' "/proc/$camera_pid/status")
    if [ "$camera_state" = Z ]; then
        break
    fi
    sleep 1
done
if [ "$camera_state" != gone ] && [ "$camera_state" != Z ]; then
    echo "camera did not quiesce: $camera_state" >&2
    reboot
    exit 73
fi
rmmod remo_sensor_imx582
"""
    elif runtime == "source":
        quiesce = """
! pidof camera >/dev/null 2>&1
! pidof pw203-camera-service >/dev/null 2>&1
grep -q '^podbay_imx582_warm ' /proc/modules
rmmod podbay_imx582_warm
"""
    else:
        raise RuntimeError(f"unsupported camera runtime: {runtime}")
    return f"""
set -eu
stock={STOCK_MODULE_PATH}
service=/tmp/pw203-camera-service
module=/tmp/pw203-sensor.ko
[ "$(sha256sum "$stock" | awk '{{print $1}}')" = {variant.inputs[STOCK_MODULE_PATH]} ]
[ "$(sha256sum "$service.next" | awk '{{print $1}}')" = {service_hash} ]
[ "$(sha256sum "$module.next" | awk '{{print $1}}')" = {module_hash} ]
rollback_needed=1
rollback_on_exit() {{
    status=$?
    if [ "$rollback_needed" = 1 ]; then
        sync
        reboot
    fi
    exit "$status"
}}
trap rollback_on_exit EXIT
{quiesce}
mv "$service.next" "$service"
mv "$module.next" "$module"
chmod 700 "$service"
if ! insmod "$module"; then
    insmod "$stock" chmap=1 lane_num=4 hdr_lane_num=4 mipi_user_def=1 i2c_slave_id=0 || true
    reboot
    exit 80
fi
grep -q '^{provider.loaded_module} ' /proc/modules
nohup env LD_LIBRARY_PATH=/lib "$service" \
    --tile 8000x576 --origin 0,2712 --settle 1 \
    --frame-length 700 --coarse 650 --gain 994 \
    --sensor-res 0 --sensor-fps 10 --sensor-binning 1 \
    </dev/null >/tmp/pw203-camera-service.log 2>&1 &
rollback_needed=0
"""


def reboot_to_stock(ssh: list[str]) -> str:
    diagnostic = run(
        [
            *ssh,
            "echo MODULES; grep -E '^(podbay_imx582_warm|remo_sensor_imx582) ' "
            "/proc/modules || true; echo SERVICE_LOG; "
            "tail -80 /tmp/pw203-camera-service.log 2>/dev/null || true",
        ],
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    ).stdout
    run(
        [
            *ssh,
            "pkill -f '^/tmp/pw203-camera-service ' 2>/dev/null || true; "
            "echo 'Podbay rollback: health-check-failed' >/tmp/podbay-rollback-reason; "
            "sync; reboot",
        ],
        check=False,
    )
    return diagnostic


def detect_runtime(ssh: list[str]) -> str:
    result = run(
        [
            *ssh,
            "if grep -q '^podbay_imx582_warm ' /proc/modules && "
            "pidof pw203-camera-service >/dev/null; then echo source; "
            "elif grep -q '^remo_sensor_imx582 ' /proc/modules && "
            "pidof camera >/dev/null; then echo stock; else echo unknown; fi",
        ],
        stdout=subprocess.PIPE,
        text=True,
    ).stdout.strip()
    if result not in {"stock", "source"}:
        raise RuntimeError(f"camera is not in a supported restart state: {result}")
    return result


def shutdown_source_service(args: argparse.Namespace) -> None:
    with socket.create_connection((args.host, 5001), timeout=2.0) as connection:
        connection.sendall(b"SHUTDOWN\n")
        response = connection.recv(128)
    if response != b"OK SHUTDOWN\n":
        raise RuntimeError(f"source service refused graceful shutdown: {response!r}")


def wait_for_source_shutdown(
    ssh: list[str], timeout: float
) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = run(
            [*ssh, "pidof pw203-camera-service >/dev/null 2>&1"], check=False
        )
        if result.returncode != 0:
            return
        time.sleep(0.1)
    raise RuntimeError("source service did not complete graceful graph teardown")


def deploy(
    args: argparse.Namespace,
    ssh: list[str],
    service: Path,
    module: Path,
    variant: FirmwareVariant,
    provider: SensorProvider,
) -> None:
    if not args.accept_camera_changes:
        raise RuntimeError("deploy requires --accept-camera-changes")
    service_hash = upload(ssh, service, "/tmp/pw203-camera-service.next")
    module_hash = upload(ssh, module, "/tmp/pw203-sensor.ko.next")
    runtime = detect_runtime(ssh)
    if runtime == "source":
        shutdown_source_service(args)
        wait_for_source_shutdown(ssh, args.health_timeout)
    remote = deployment_script(service_hash, module_hash, variant, provider, runtime)
    run([*ssh, remote])
    deadline = time.monotonic() + args.health_timeout
    last_error = "service did not answer"
    while time.monotonic() < deadline:
        try:
            with socket.create_connection((args.host, 5001), timeout=1.0) as connection:
                connection.sendall(b"VERSION\n")
                response = connection.recv(128)
                if response == PROTOCOL_RESPONSE:
                    print(
                        f"PW203 camera service ready at {args.host}:5001 "
                        f"with {provider.name}"
                    )
                    return
                last_error = f"unexpected protocol response: {response!r}"
        except OSError as error:
            last_error = str(error)
        time.sleep(0.25)
    diagnostic = reboot_to_stock(ssh)
    raise RuntimeError(
        f"deployment health check failed: {last_error}; requested stock reboot\n"
        f"{diagnostic}"
    )


def add_build_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--kernel-build",
        type=Path,
        required=True,
        help="configured reviewed Infinity6C kernel tree required by the source provider",
    )


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.88.10")
    parser.add_argument("--key", type=Path)
    parser.add_argument("--known-hosts", type=Path, default=DEFAULT_KNOWN_HOSTS)
    parser.add_argument("--connect-timeout", type=int, default=5)
    parser.add_argument("--http-port", type=int, default=10086)
    subparsers = parser.add_subparsers(dest="action", required=True)
    build = subparsers.add_parser("build", help="read camera inputs and build outside the repository")
    build.add_argument("--output-dir", required=True, type=Path)
    add_build_arguments(build)
    deploy_parser = subparsers.add_parser("deploy", help="build and temporarily start the service")
    deploy_parser.add_argument("--accept-camera-changes", action="store_true")
    deploy_parser.add_argument("--health-timeout", type=float, default=10.0)
    add_build_arguments(deploy_parser)
    return parser.parse_args(argv)


def main() -> int:
    args = parse_args()
    args.known_hosts = args.known_hosts.expanduser().resolve()
    if args.action == "deploy" and not args.accept_camera_changes:
        raise RuntimeError("deploy requires --accept-camera-changes")
    require_program("ssh")
    if not args.known_hosts.is_file():
        raise RuntimeError(
            f"isolated temporary SSH trust record is absent: {args.known_hosts}; "
            "run tools/pw203_bootstrap_ssh.py first"
        )
    ssh = ssh_command(args)
    with tempfile.TemporaryDirectory(prefix="podbay-pw203-") as temporary:
        directory = Path(temporary)
        directory.chmod(0o700)
        inputs = fetch_camera_inputs(ssh, directory)
        identity = read_camera_identity(args, ssh)
        variant = identify_variant(inputs, identity)
        service = build_service(inputs, directory)
        provider = SOURCE_WARM_PROVIDER
        module = build_warm_sensor_module(args.kernel_build, directory, variant)
        if args.action == "build":
            copy_artifacts(service, module, args.output_dir, variant, provider)
        else:
            deploy(args, ssh, service, module, variant, provider)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError, subprocess.CalledProcessError) as error:
        print(f"error: {error}", file=sys.stderr)
        raise SystemExit(1)
