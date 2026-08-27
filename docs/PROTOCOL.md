# Camera service protocol

The copied camera service implements protocol version `26` on TCP port `5001`
and a line-oriented control endpoint on TCP port `5002`. This document records
only the interface required to keep existing clients compatible.

Send `VERSION\n` on port 5001. A compatible server replies:

```text
OK CUSTOM_CAMERA 26
```

Image commands and their binary responses include:

| Command | Response | Purpose |
| --- | --- | --- |
| `SENSOR GET` | text | Active physical crop, binning, and output dimensions |
| `SENSOR SET X Y W H B` | text, after transition | Select sensor crop and binning |
| `SENSOR PREVIEW` | text, after transition | Select the built-in preview geometry |
| `CAPTURE_CURRENT SEQ [SETTLE]` | `ORT1` | One packed RAW10 frame |
| `CAPTURE_THUMB SEQ X Y SETTLE W H` | `OTH1` | One little-endian gray16 thumbnail |
| `CAPTURE_COARSE SEQ W H` | `OTH1` | One coarse full-sensor thumbnail |
| `CAPTURE_GLOBAL SEQ W H` | `OTH1` | One low-bandwidth full-field thumbnail using native 4x4 binning and 2x sensor scaling |
| `STREAM_EYES SEQ LX LY RX RY W H [FRAMES [EVERY CW CH]]` | `ORE1`, optionally `OTH1` and `OTM1` | Paired RAW10 eye stream used by the existing tracker |
| `QUIT` | none | Close the client connection |

The control endpoint accepts one line-oriented command per TCP connection:

| Command | Purpose |
| --- | --- |
| `PING` | Check the VCM/control endpoint |
| `FOCUS GET` | Read the current 0..1023 VCM position |
| `FOCUS STATUS` | Read target, estimated readback, settled state, remaining time, and generation |
| `FOCUS SET P` | Set an absolute host-bounded VCM position |
| `FOCUS STEP D` | Apply a signed host-bounded VCM step |
| `EXPOSURE GET` | Read coarse exposure lines and frame length |
| `EXPOSURE SET L` | Set coarse exposure lines and extend frame length when required |
| `EXPOSURE STEP D` | Apply a signed exposure-line step |
| `ROI SET LX LY RX RY W H` | Set absolute left/right eye regions used by the host tracking loop |

Focus commands expose the actuator control surface used by a host autofocus
loop; the camera service does not claim to implement an autofocus algorithm.
Mutating focus and eye ROI is disabled while the sensor is in binned
acquisition mode.

Binary image packets begin with a 64-byte little-endian header:

```text
0x00  char[4] magic: ORT1, OTH1, ORE1, or OTM1
0x04  u16     packet version (1)
0x06  u16     header bytes (64)
0x08  u32/i32 status, or 1-based eye region id for ORE1
0x0c  u32     format (1=packed RAW10 LE40, 3=gray16, 4=telemetry)
0x10  u64     sequence
0x18  u64     timestamp_ns
0x20  u32     sensor_x
0x24  u32     sensor_y
0x28  u32     width
0x2c  u32     height
0x30  u32     stride
0x34  u32     payload_bytes
0x38  u32     pixel format / bit depth
0x3c  u32     settle or plane count
```

The web proof client runs on the user's computer and connects to this TCP
service on the camera. It checks the version before every capture and uses
`SENSOR GET` plus `CAPTURE_THUMB`; it does not alter camera geometry. The full
eye tracker remains compatible because the copied server retains the unchanged
`STREAM_EYES` and `ORE1` implementation.

`CAPTURE_GLOBAL` covers the complete 8000x6000 photosite array while emitting
only 1000x750 RAW10 over CSI/DMA. The service phase-safely averages 2x2 Bayer
cells into the requested gray16 dimensions (up to 1000x750); dimensions above
500x375 are compatibility resampling and do not contain additional sensor
detail. The pre-existing `CAPTURE_COARSE` command remains unchanged.
