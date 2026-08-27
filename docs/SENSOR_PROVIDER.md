# Sensor-provider replacement

## Objective

Replace the generated `remo_sensor_imx582.ko` derivative with source-authored
code while preserving the current RAM-only deployment and recovery behavior.
The replacement is split into a portable sensor core and a thin target-specific
adapter. Only the adapter needs SigmaStar's registration interface.

## Current result

`sensor-provider/` implements and tests the first layer: construction of a
bounded IMX582 ROI register plan for 1x1, 2x2, and 4x4 readout. It validates the
physical array bounds, keeps the sensor stopped, uses group hold around the
coherent change, and leaves stream start to the future receiver-aware adapter.
No vendor header, library, binary, register table, or generated module is
included.

`tools/inspect_sensor_abi.py` records non-copyrightable ELF interface facts
from a module supplied by the camera owner. It will not extract code or data.
Its report will be suitable for comparing firmware variants and refusing a
build when a target differs from the reviewed ABI.

`tools/inspect_sensor_callbacks.py` goes one level deeper. It asks an ARM ELF
disassembler for the relocation facts in the module's handle initializer and
reports only callback names and destination offsets. For firmware 4.4.2.2 it
verifies 23 callbacks from offset `0x09c4` through `0x0a2c`, implying a minimum
framework-owned handle extent of `0x0a30` bytes. The offsets are non-contiguous;
this is why substituting an unverified SDK record is unsafe even when all export
names and vermagic strings match.

Inspection of the owner-supplied `mi.ko` framework closes the size question:
`DrvRegisterSensorDriverEx` selects one of three pad records and clears exactly
`0x0a44` bytes (2,628 bytes) before invoking the provider initializer. Thus
`0x0a30` is only the minimum extent touched by the observed IMX582 callbacks;
the exact framework handle is `0x0a44`. The provider-supplied private state is a
separate 132-byte allocation whose pointer is stored at handle offset `0x28`.
The framework also installs its sensor-interface API pointer at offset `0x3c`
after the provider initializer returns.

For the owned PW203 running firmware 4.4.2.2, a read-only stream of the immutable
stock module produced SHA-256 `59b042523ea9c3a855b78d6b0ea4e8fa2cacb598e23d044ca9ac4f803399ec08`,
ARM ELF32, 18,148 bytes, and vermagic
`5.10.61 preempt mod_unload ARMv7 thumb2 p2v8`. All six currently known sensor
registration/version/release symbols were present. This proves the coarse
module boundary, not the callback-record layout.

The same read-only inspection found no `/proc/config.gz`, kernel build/source
link, kernel BTF image, or readable `/proc/kallsyms` on the camera. Consequently
the repository cannot yet reproduce a loadable module from the installed
device alone. The portable core remains useful and testable while the exact
kernel configuration and adapter ABI are obtained independently.

The device tree identifies the target more precisely as
`sstar,infinity6c`, model `INFINITY6C SSC027D-S01A`. The GPL-2.0 OpenIPC Linux
branch `sigmastar-infinity6c` at commit
`d17f67f1f90259dab41b6b6abb28fb64348d83e9` is Linux 5.10.61 and contains
SSC027D USB-camera defconfigs. It is a reproducible candidate build tree, but
it is not yet claimed to be OBSBOT's exact configured source: the installed
kernel was built separately with SigmaStar GCC 11.1 and reports build number
`#8`. Candidate-module compatibility must therefore be established by ELF
attributes and a registration-only canary on recoverable test hardware, not by
matching the release number alone.

The registration-free canary provided the missing configuration evidence. The
unmodified public USB-camera defconfig generated a `0x240`-byte module record
with its exit callback at `0x218`; it loaded, but the live kernel read a null
exit pointer at `0x1a4` and correctly exposed the canary as `[permanent]`.
Reboot removed it. Disabling `CONFIG_KALLSYMS` and `CONFIG_PERF_EVENTS` (which
also disables derived `CONFIG_MODULES_TREE_LOOKUP`) generated the stock-proven
`0x200` record, init offset `0x11c`, and exit offset `0x1a4`. That corrected
canary loaded and unloaded successfully while the stock camera and sensor
module remained healthy. `tools/build_kernel_canary.py` now enforces all three
layout facts before releasing an artifact.

The next gate is `registration-canary/`. Unlike the first canary it calls the
six already verified SigmaStar lifecycle exports, but its provider callback
always returns `-ENODEV` without dereferencing the framework handle. It contains
no sensor access. This isolates registration/release correctness from the much
riskier handle population and sensor sequencing work.

The registration canary passed on the owned PW203 after the stock camera was
stopped and its stock sensor provider unloaded. All three ABI version checks,
driver registration, slave-ID bookkeeping, the deliberate `-ENODEV` callback,
release, and module unload completed. A normal reboot then restored the stock
camera and stock sensor module. No sensor register or persistent state was
accessed. This narrows the remaining work to opaque-handle population and
actual sensor sequencing.

The source-authored handle layer now encodes that measured boundary without a
vendor header. Host tests populate an exact `0x0a44` buffer, verify all 23
callback offsets, construct three 72-byte framework resolution records at
offset `0x90`, and assert that the framework-owned private pointer at `0x28`
and API pointer at `0x3c` are preserved. The three records cover the service's
fine (index 0), scaled preview (index 1), and coarse 4x4 (index 2) paths.

`warm-provider/` wraps the same facts in a loadable ARM kernel adapter. Its
callbacks implement metadata, selection, orientation, and bounded AE state,
but power, initialization, release, and pattern callbacks are hardware no-ops.
The build gate permits only the six reviewed SigmaStar lifecycle imports plus
ordinary kernel/compiler helpers; any new import is rejected. It contains no
register table and no I2C, GPIO, reset, clock, stream, firmware, or persistent
write path. A candidate built against the corrected public kernel configuration
has the expected `0x200` module record and reviewed init/exit relocations.

This is intentionally a warm-handoff stage, not yet a cold-start provider. The
stock shutdown path may assert reset or remove MCLK before module replacement,
which would discard the sensor's initialized state. The next live trial must
therefore first prove whether a safe framework release can preserve that state.
If it cannot, the next bounded implementation is a deployment-time volatile
table supplied from the owner-provided module; the opaque 828-write table still
does not belong in this repository. A fully source-authored cold-start sequence
remains the preferred endpoint.

The first hardware lifecycle trial passed on the owned PW203. After a physical
USB power cycle, the bounded Podbay network/SSH bootstrap completed with full
60-byte responses. The exact stock-module and candidate hashes and read-only
root were rechecked before mutation. `SIGKILL` left the firmware camera as a
zombie rather than removing its PID immediately; the gate required that state
before unloading the stock provider, ensuring no user-space camera threads
remained. The warm provider then registered its `0x0a44` handle, appeared in
`/proc/modules`, released, and unloaded successfully. Kernel logs contained the
expected inherited proprietary-module taint plus all three Podbay lifecycle
messages. No MI user-space consumer was started, so no power, initialization,
or sensor callback was exercised. A normal reboot returned USB identity
`3564:fefb`; a bounded read-only UVC query reported the ordinary 1920x1080
MJPEG/30 fps stock pipeline. This proves handle population and release, but not
yet a live warm image handoff or cold initialization.

The next RAM-only trial proved the live warm image handoff. The exact pinned
4.4.2.2 libraries rebuilt the existing protocol-v26 service to its previously
accepted SHA-256 `fb514dffa49b1b99444a96a00ab759e583e2bc1d81297b9acbca60e08fe39be0`.
The stock camera was killed to a zombie without running its cleanup path, the
stock provider was unloaded, and the source-authored warm provider was loaded.
The service initialized MI_SNR through the new handle, directly programmed and
attested the 8000x576 physical 1x1 RAW10 path, and exposed both protocol ports.
`SENSOR GET` returned physical crop `0,2712 8000x576`, VCM `PING` returned
`OK VCM`, and `CAPTURE_CURRENT` returned a valid 64-byte ORT1 header followed
by exactly 5,760,000 payload bytes (stride 10,000), timestamp
`1689984112123300680`, and SHA-256
`c1d61e223c8e727e4d4f93d6db21b43e0eab220415589e4a0aeca0fcbc1c2199`.
No kernel error appeared in the bounded log. A normal reboot again restored
the stock 1920x1080 MJPEG/30 fps UVC pipeline. This proves that cold register
initialization can remain outside the source provider for this handoff path;
standalone cold boot without first running the stock firmware remains unproven.

During the subsequent attempt to restore temporary USB Ethernet, the bounded
UVC handshake received a one-byte response to the current-mode query instead
of the required 60 bytes. The tool stopped immediately without retry or
`usbreset`. Per the established recovery procedure, a real USB power removal is
required before further live testing. This is a recoverable control-path wedge,
not persistent damage.

Source references:

- <https://github.com/OpenIPC/linux/tree/sigmastar-infinity6c>
- <https://github.com/OpenIPC/waybeam_venc#building-maruko-sensor-drivers-from-source>

Inspect another owner-supplied module without retaining its bytes in this
repository:

```sh
python3 tools/inspect_sensor_abi.py /path/to/remo_sensor_imx582.ko
python3 tools/inspect_sensor_callbacks.py /path/to/remo_sensor_imx582.ko
```

## Remaining gates for cold-start independence

1. Replace or independently derive the cold 828-write initialization sequence;
   the current adapter intentionally relies on the firmware's prior boot-time
   initialization and contains no opaque table.
2. Add source-owned power/MCLK/reset sequencing only after its framework API
   offsets and rollback ordering are independently verified.
3. Exercise invalid resolution, I2C NACK, receiver-lock failure, service crash,
   and repeated source-to-stock recovery before making source-warm the default.
4. Keep all deployment RAM-only until standalone cold boot and automatic
   rollback have passed on recoverable hardware.

`tools/deploy.py` now integrates the adapter as the sole guarded provider path
for the live-proven 4.4.2.2 firmware identity. It builds
the provider through the reviewed kernel layout/import gates and automatically
requests a stock reboot on handoff or protocol-health failure.

The integrated deployment path passed on hardware and was left running for the
existing viewer client. `tools/deploy.py deploy`
independently fetched and pinned the installed firmware inputs, rebuilt service
SHA-256 `fb514dffa49b1b99444a96a00ab759e583e2bc1d81297b9acbca60e08fe39be0`,
built the source provider through the module-record/import gates, required the
stock camera to quiesce, replaced only the RAM-loaded provider, and received
`OK CUSTOM_CAMERA 26`. The deployed source module was
`podbay_imx582_warm`; service PID 2148 reported physical 1x1 RAW10 geometry
`0,2712 8000x576`. ORT1 sequence 58202 returned exactly 5,760,000 bytes,
stride 10,000, timestamp `1689985365498366662`, and SHA-256
`557ed95798d4af28ae67037cba28a816986a2c190fdbf852d851a1d5f18b4ef3`.
VCM `PING` also passed and the root filesystem remained read-only.

The resident fine-to-coarse path re-arms only the VIF output port after the
first coarse exposure boundary; it does not repeat sensor programming or tear
down the resident MI sensor/MIPI group. On the owned camera, aligning that
re-arm to 120 ms reduced the measured mode-switch phase from 211 ms to about
130 ms. Ten consecutive complete coarse-thumbnail/fine-restore cycles had a
319.5 ms median (319.3-336.1 ms), returned valid OTH1 packets, and were followed
by a valid 8000x576 ORT1 fine frame.

Kernel debug metadata and GNU build IDs make the unstripped module's whole-file
hash vary with temporary build paths. Deployment therefore pins a canonical
digest after stripping debug sections and excluding only `.note.gnu.build-id`:
`e21e2015889d34939ff33895524c8260d9c77d83821bf27a218dadf3452c663a`.
The actual unmodified upload is still hashed immediately before transfer and
rechecked byte-for-byte on the camera. ABI layout and import checks apply to
the original module before either digest is accepted.

## Failure and brick model

The likely failures are deliberately separated from genuinely persistent
damage:

| Failure | Mechanism | Expected recovery | Prevention |
| --- | --- | --- | --- |
| Kernel exception or jump to garbage | Callback pointer written at the wrong handle offset | Normal reboot; physical power removal if USB/control is lost | Exact relocation-offset verifier and registration-only canary |
| Kernel use-after-free on unload | Framework retains callbacks or private state after module exit | Reboot or power removal | Prove release ordering; never unload during an active MI graph |
| Sensor/CSI wedge | Bad clock, reset, stream, lane, or I2C ordering | Usually full power removal | Keep stream stopped until receiver configuration; bounded I2C and rollback |
| Watchdog/reset loop | Driver blocks a kernel/vendor worker or boot-time consumer | Restore stock boot path by power cycle | RAM-only opt-in load after stock boot; leave stock module immutable |
| Persistent brick | Writing sensor OTP, MCU/boot flash, boot hooks, or writable root state | May require external recovery or replacement | These interfaces and writes are out of scope and prohibited |

The portable implementation cannot trigger any row in the table. The warm
provider is loadable and registers callbacks, so an ABI error could trigger the
first two rows, but it has no path to the sensor or persistent storage. It
remains RAM-only and opt-in; it must not be made part of the firmware boot path.
