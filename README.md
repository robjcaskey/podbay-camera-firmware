# Podbay Camera Firmware

Podbay is a source-only clean repository for a custom PW203 camera-side RAW10
service, its fail-closed boot-hook installer, a temporary build/deploy workflow,
and a tiny compatible browser proof client.
It intentionally contains no firmware, vendor program, binary driver, patched
kernel module, SDK, shared library, model, capture, or generated executable.

Current contents:

- `camera-service/`: the authored standalone Rust sensor/VIF service. Its wire
  protocol is unchanged at version 26 so the existing eye-tracking client can
  connect to cameras running this service.
- `tools/install.py`: the authored one-shot boot-hook manager. This is retained
  as source, but it is not yet a turnkey service installer.
- `tools/deploy.py`: a host-side, no-web build/deploy workflow. It requires SSH
  to be available already, reads the exact required files from the user's own
  camera into a private temporary directory, verifies pinned hashes, builds the
  source-authored provider locally, uploads the generated artifacts, and
  removes the temporary directory. It does not
  install packages or download anything from the web.
- `tools/pw203_netmode.py`: generates the complete bounded UVC extension-unit
  handshake from named protocol fields and computed CRCs, then temporarily
  switches the camera into USB Ethernet mode. It contains no captured packet
  payloads and makes no persistent camera change.
- `tools/pw203_bootstrap_ssh.py`: runs on the user's computer, configures the
  USB Ethernet interface, and asks services already installed in the user's
  firmware to start a temporary key-only SSH daemon. It generates the shell
  script and SSH host key locally, downloads nothing, and stores nothing
  persistently on the camera.
- `tools/web_viewer.py`: a dependency-free proof viewer that runs on the user's
  computer, connects to the camera over TCP, and serves the browser UI locally.
  It must not be copied to or run on the camera. It uses the same camera
  protocol and never invokes a vendor camera application or UVC.
- `sensor-provider/`: the source-authored, host-tested IMX582 provider core and
  SigmaStar warm-handoff kernel adapter. The adapter has produced verified
  8000x576 RAW10 through the normal Podbay service, but still requires stock
  firmware to initialize the sensor before handoff.

Generated build artifacts remain outside this source repository. The browser
viewer, clean-room USB handshake, temporary strict-trust SSH bootstrap, guarded
build/deploy path, and protocol/control acceptance checks have passed against an
owned PW203 running firmware 4.4.2.2. The deployment is RAM-only and a power
cycle restores the stock firmware boot path. Do not install the boot hook or
make persistent camera changes until the source, recovery path, target-device
checks, and complete install transaction have been separately reviewed and
exercised on test hardware.

## Source-only build boundary

The repository contains no link libraries or sensor modules. `tools/deploy.py`
requires an owned PW203 already reachable over SSH. Its `build` action is
read-only with respect to the camera and refuses to put generated artifacts
inside this repository:

```bash
python3 tools/deploy.py --key ~/.ssh/id_ed25519 build \
  --kernel-build /path/to/configured/infinity6c-linux \
  --output-dir /tmp/podbay-source-build
```

The script does not install Rust, its ARM target, Zig, or any other dependency;
those must already be installed explicitly. Cargo is forced offline. The
`deploy` action additionally changes the camera's temporary runtime state and
therefore requires `--accept-camera-changes`.

```bash
python3 tools/deploy.py --key ~/.ssh/id_ed25519 deploy \
  --kernel-build /path/to/configured/infinity6c-linux \
  --accept-camera-changes
```

The source path rechecks the exact firmware identity and immutable input
hashes, builds through the kernel layout/import gates, requires the firmware
camera to reach a gone or zombie state before module replacement, and verifies
protocol version 26. Any failure after quiescing the stock camera requests a
normal reboot; a post-start health failure collects diagnostics and also
reboots to the immutable stock path. The immutable stock module is validated
only for firmware identity and emergency rollback; Podbay never transforms it
or selects it as the normal provider.

For a camera in ordinary USB mode, inspect and then run the source-only SSH
bootstrap on the user's computer:

```bash
python3 tools/pw203_bootstrap_ssh.py --dry-run
python3 tools/pw203_bootstrap_ssh.py --accept-camera-bootstrap
```

The active switch requires PyUSB and libusb to be installed already; the tool
does not install them. The bootstrap uses the HTTP shell endpoint and SSH daemon
already present in the supported installed firmware. It does not include those
programs, start the firmware camera application, or place files on persistent
camera storage. Before switching it refuses an owned `/dev/video0`, requires a
bounded healthy stock UVC query, and requires an unambiguous USB identity. It
pins the freshly generated temporary SSH host key in the isolated mode-0600
`/tmp/podbay-pw203-known-hosts` file; deploy requires that strict trust record
and never uses the user's persistent `known_hosts`. Power cycling removes the
camera's temporary network/SSH state.

Once a compatible service is already running on the camera at
`192.168.88.10:5001`, run the proof viewer on the user's computer:

```bash
python3 tools/web_viewer.py
```

On that same computer, visit `http://127.0.0.1:8080/`. The browser connects only
to the local viewer; the viewer connects to the camera over TCP.

See `docs/PROTOCOL.md` for the compatibility surface,
`docs/PROVENANCE.md` for the source-only boundary, and
`docs/SENSOR_PROVIDER.md` for the stock-module replacement gates.
