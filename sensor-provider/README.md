# Source-authored IMX582 provider core

This directory contains the source-authored replacement sensor provider.
It contains a portable, side-effect-free IMX582 mode planner, an independently
measured SigmaStar handle builder, and host tests.

The separation is deliberate: sensor register planning can be tested on the
host, while the small future SigmaStar adapter remains isolated behind an ABI
gate. The planner emits a stopped-sensor transaction and never emits the stream
start write. A future adapter must configure and verify the receiver before it
starts streaming, and must leave the stock module available for recovery.

Run the host tests with:

```sh
make -C sensor-provider test
```

See `docs/SENSOR_PROVIDER.md` for the implementation and hardware gates.

`kernel-canary/` is a registration-free compatibility probe. It logs load and
unload and has no device-facing operation. Build it outside the repository only
through `tools/build_kernel_canary.py`; it is not the sensor adapter.

For the reviewed public Infinity6C kernel candidate, layer
`kernel-config.fragment` over the SSC027D SPI-NAND USB-camera defconfig before
`modules_prepare`. The build tool additionally refuses output unless the
generated module record is exactly `0x200` bytes with init at `0x11c` and exit
at `0x1a4`.

`registration-canary/` is the next bounded gate. It exercises only the
SigmaStar version, registration, slave-ID bookkeeping, and release calls. Its
provider callback always returns `-ENODEV` without dereferencing the opaque
handle. It must be tested only with the stock camera stopped and stock sensor
provider unloaded; reboot restores the ordinary camera afterward.

`warm-provider/` is a loadable but still hardware-inert adapter. It populates
the exact `0x0a44` handle, three 72-byte resolution records, and all 23 observed
callbacks. Its callbacks never access I2C, GPIO, reset, MCLK, or stream state.
Consequently it is suitable for the next ABI/lifecycle trial, but it is not a
cold-start camera driver: it can produce images only if an already initialized
sensor and its clocks survive the handoff. Build it outside the repository
with `tools/build_warm_sensor_provider.py`.
