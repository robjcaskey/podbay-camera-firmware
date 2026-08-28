use std::collections::VecDeque;
use std::env;
use std::ffi::c_void;
use std::fs::{File, OpenOptions};
use std::io::{BufRead, BufReader, Read, Write};
use std::net::{TcpListener, TcpStream};
use std::os::fd::AsRawFd;
use std::os::raw::{c_int, c_ulong};
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

const SENSOR_WIDTH: u32 = 8000;
const SENSOR_HEIGHT: u32 = 6000;
const PREVIEW_SENSOR_X: u32 = 160;
const PREVIEW_SENSOR_Y: u32 = 840;
const PREVIEW_SENSOR_WIDTH: u32 = 7680;
const PREVIEW_SENSOR_HEIGHT: u32 = 4320;
const PREVIEW_WIDTH: u32 = 1920;
const PREVIEW_HEIGHT: u32 = 1080;
const RAW10_RG: u32 = 32;
const MI_MODULE_ID_VIF: u32 = 6;
const I2C_SLAVE_FORCE: c_ulong = 0x0706;
const SENSOR_I2C_ADDRESS: c_ulong = 0x10;
const VCM_I2C_ADDRESS: u16 = 0x0c;
const VCM_SETTLE_BASE_MS: u64 = 250;

fn vcm_settle_duration(from: u16, to: u16) -> Duration {
    Duration::from_millis(VCM_SETTLE_BASE_MS + from.abs_diff(to) as u64 / 4)
}

struct FocusStatus {
    target: u16,
    readback: u16,
    settled: bool,
    remaining_ms: u64,
    generation: u64,
}
const I2C_RDWR: c_ulong = 0x0707;
const I2C_M_RD: u16 = 0x0001;
const PACKET_HEADER_BYTES: usize = 64;
const PACKET_FORMAT_RAW10_LE40: u32 = 1;
const PACKET_FORMAT_GRAY16: u32 = 3;
const PACKET_FORMAT_TELEMETRY: u32 = 4;
const PROTOCOL_VERSION: u32 = 26;
const MIN_RAW_ROI_SETS_PER_SECOND: usize = 10;
const MIN_LONG_EXPOSURE_RAW_ROI_SETS_PER_SECOND: usize = 2;
const TELEMETRY_VERSION: u16 = 1;
const TELEMETRY_BUCKETS: usize = 16;
const TELEMETRY_PREFIX_BYTES: usize = 16;
const TELEMETRY_RECORD_BYTES: usize = 64;
const TELEMETRY_EVERY_RAW_SETS: u32 = 2;
const TELEMETRY_WINDOW: Duration = Duration::from_secs(1);
// Mode transitions validate a real frame before acknowledging the host. These
// short hardware-settle intervals avoid paying the original conservative
// 200/300/300 ms sleeps on top of that frame-level proof.
const PIPELINE_STOP_SETTLE: Duration = Duration::from_millis(200);
const SENSOR_ENABLE_SETTLE: Duration = Duration::from_millis(300);
const VIF_ENABLE_SETTLE: Duration = Duration::from_millis(300);
const TELEMETRY_BUCKET_LIMITS_US: [u32; TELEMETRY_BUCKETS] = [
    8,
    16,
    32,
    64,
    128,
    256,
    512,
    1_000,
    2_000,
    4_000,
    8_000,
    16_000,
    32_000,
    64_000,
    128_000,
    u32::MAX,
];

const CAMERA_STAGE_SENSOR_INTERVAL: u8 = 1;
const CAMERA_STAGE_SENSOR_ACQUIRE: u8 = 2;
const CAMERA_STAGE_CONTEXT_BUILD: u8 = 3;
const CAMERA_STAGE_ROI_SLICE: u8 = 4;
const CAMERA_STAGE_STREAM_WRITE: u8 = 5;
const CAMERA_STAGE_VCM_COMMAND: u8 = 6;
const CAMERA_STAGE_VCM_REMAINING: u8 = 7;
const CAMERA_STAGE_IDS: [u8; 7] = [
    CAMERA_STAGE_SENSOR_INTERVAL,
    CAMERA_STAGE_SENSOR_ACQUIRE,
    CAMERA_STAGE_CONTEXT_BUILD,
    CAMERA_STAGE_ROI_SLICE,
    CAMERA_STAGE_STREAM_WRITE,
    CAMERA_STAGE_VCM_COMMAND,
    CAMERA_STAGE_VCM_REMAINING,
];

#[derive(Clone, Debug, Default)]
struct TimingHistogram {
    buckets: [u16; TELEMETRY_BUCKETS],
    count: u32,
    max_us: u32,
    last_us: Option<u32>,
    max_jitter_us: u32,
}

impl TimingHistogram {
    fn record(&mut self, duration: Duration) {
        self.record_us(duration.as_micros().min(u32::MAX as u128) as u32);
    }

    fn record_us(&mut self, sample_us: u32) {
        let bucket = TELEMETRY_BUCKET_LIMITS_US
            .iter()
            .position(|limit| sample_us <= *limit)
            .unwrap_or(TELEMETRY_BUCKETS - 1);
        self.buckets[bucket] = self.buckets[bucket].saturating_add(1);
        self.count = self.count.saturating_add(1);
        self.max_us = self.max_us.max(sample_us);
        if let Some(previous) = self.last_us {
            self.max_jitter_us = self.max_jitter_us.max(previous.abs_diff(sample_us));
        }
        self.last_us = Some(sample_us);
    }

    fn percentile(&self, numerator: u32, denominator: u32) -> u32 {
        if self.count == 0 {
            return 0;
        }
        let target = self
            .count
            .saturating_mul(numerator)
            .saturating_add(denominator - 1)
            / denominator;
        let mut cumulative = 0u32;
        for (index, count) in self.buckets.iter().enumerate() {
            cumulative = cumulative.saturating_add(*count as u32);
            if cumulative >= target {
                return TELEMETRY_BUCKET_LIMITS_US[index].min(self.max_us);
            }
        }
        self.max_us
    }

    fn clear(&mut self) {
        *self = Self::default();
    }
}

struct CameraTelemetry {
    window_started: Instant,
    window_sequence: u32,
    stages: [TimingHistogram; CAMERA_STAGE_IDS.len()],
}

impl CameraTelemetry {
    fn new() -> Self {
        Self {
            window_started: Instant::now(),
            window_sequence: 0,
            stages: std::array::from_fn(|_| TimingHistogram::default()),
        }
    }

    fn reset(&mut self, now: Instant) {
        self.window_started = now;
        self.window_sequence = self.window_sequence.wrapping_add(1);
        for stage in &mut self.stages {
            stage.clear();
        }
    }

    fn record(&mut self, stage_id: u8, duration: Duration) {
        if let Some(index) = CAMERA_STAGE_IDS.iter().position(|id| *id == stage_id) {
            self.stages[index].record(duration);
        }
    }

    fn record_us(&mut self, stage_id: u8, sample_us: u32) {
        if let Some(index) = CAMERA_STAGE_IDS.iter().position(|id| *id == stage_id) {
            self.stages[index].record_us(sample_us);
        }
    }

    fn payload(&mut self, now: Instant) -> Vec<u8> {
        let window_expired = now.duration_since(self.window_started) >= TELEMETRY_WINDOW;
        let populated = self
            .stages
            .iter()
            .filter(|histogram| histogram.count != 0)
            .count();
        let mut payload =
            Vec::with_capacity(TELEMETRY_PREFIX_BYTES + populated * TELEMETRY_RECORD_BYTES);
        payload.extend_from_slice(&TELEMETRY_VERSION.to_le_bytes());
        payload.extend_from_slice(&(populated as u16).to_le_bytes());
        payload.extend_from_slice(&self.window_sequence.to_le_bytes());
        let elapsed_us = now
            .duration_since(self.window_started)
            .as_micros()
            .min(u32::MAX as u128) as u32;
        payload.extend_from_slice(&elapsed_us.to_le_bytes());
        payload.extend_from_slice(&0u32.to_le_bytes());
        for (&stage_id, histogram) in CAMERA_STAGE_IDS.iter().zip(&self.stages) {
            if histogram.count == 0 {
                continue;
            }
            payload.push(stage_id);
            payload.push(TELEMETRY_BUCKETS as u8);
            payload.extend_from_slice(&1u16.to_le_bytes());
            payload.extend_from_slice(&histogram.count.to_le_bytes());
            for quantile in [
                histogram.percentile(1, 4),
                histogram.percentile(1, 2),
                histogram.percentile(3, 4),
                histogram.percentile(95, 100),
                histogram.max_us,
                histogram.max_jitter_us,
            ] {
                payload.extend_from_slice(&quantile.to_le_bytes());
            }
            for count in histogram.buckets {
                payload.extend_from_slice(&count.to_le_bytes());
            }
        }
        if window_expired {
            self.reset(now);
        }
        payload
    }
}

fn record_camera_timing(telemetry: &Arc<Mutex<CameraTelemetry>>, stage_id: u8, duration: Duration) {
    if let Ok(mut telemetry) = telemetry.lock() {
        telemetry.record(stage_id, duration);
    }
}

fn record_camera_sample_us(telemetry: &Arc<Mutex<CameraTelemetry>>, stage_id: u8, sample_us: u32) {
    if let Ok(mut telemetry) = telemetry.lock() {
        telemetry.record_us(stage_id, sample_us);
    }
}
// A vertical crop takes effect at a sensor frame boundary and an occasional
// queued-frame miss can make the first deliverable new-origin frame span three
// periods.  A 576-line full-width band covers both 256-line eye rectangles
// plus gross-motion headroom while a 700-line frame keeps that verified third
// exposure below the 100 ms switch budget.  This remains physical 1x1 RAW10:
// no binning, skipping, digital scaling, or host-side pixel fabrication.
const FINE_FRAME_LENGTH: u16 = 700;
const FINE_EXPOSURE: u16 = 650;
const FINE_GAIN: u16 = 994;
const FINE_LINE_LENGTH: u16 = 0x3970;
// The stock 4x4 readout uses a 0x0b60 line length. Retain the proven fine-mode
// PLL and MIPI lane rate, but remove the needless 1x1 full-width horizontal
// blanking while acquiring the 2000x1500 full-sensor coarse frame.
const COARSE_LINE_LENGTH: u16 = 0x0b60;
const COARSE_FRAME_LENGTH: u16 = 1616;
const COARSE_EXPOSURE: u16 = 1566;
const COARSE_GAIN: u16 = 930;
const COARSE_EXPOSURE_MARGIN_LINES: u16 = 50;
// A valid full-sensor 4x4 exposure arrives in 124-137 ms on this camera. Keep
// the one-shot wait bounded so a VIF output-port activation miss can be
// re-armed in the same acquisition cycle instead of hitching for 1.5 seconds.
const COARSE_FRAME_WAIT: Duration = Duration::from_millis(450);
// Begin the VIF output re-arm just before the measured 124-137 ms first coarse
// frame boundary. MI_VIF_DisableOutputPort then synchronizes to that boundary;
// waiting 145 ms here can enter the following exposure and add another 40-50
// ms before the disable completes.
const COARSE_REARM_DWELL_MS: u64 = 120;
const SCAN_MIN_BLANKING_LINES: u32 = 116;
const SCAN_EXPOSURE_MARGIN_LINES: u16 = 50;
const SCAN_GAIN: u16 = 930;
const SCAN_ORIGIN_SETTLE_FRAMES: u32 = 2;
// A live origin update can leave both a queued exposure and one sensor/VIF
// exposure in flight at the old origin.  Consume those buffers without
// touching their pixels; the following STREAM buffer is then the same third
// post-command exposure that CAPTURE_SCAN proved against independent bands.
const LIVE_ORIGIN_DISCARD_FRAMES: usize = SCAN_ORIGIN_SETTLE_FRAMES as usize;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct ExposureTiming {
    frame_length: u16,
    coarse_lines: u16,
}

impl ExposureTiming {
    fn from_config(config: &Config) -> Self {
        Self {
            frame_length: config.frame_length,
            coarse_lines: config.coarse,
        }
    }

    fn pack(self) -> u32 {
        (u32::from(self.frame_length) << 16) | u32::from(self.coarse_lines)
    }

    fn unpack(word: u32) -> Self {
        Self {
            frame_length: (word >> 16) as u16,
            coarse_lines: word as u16,
        }
    }
}

fn matching_coarse_exposure(fine: ExposureTiming) -> ExposureTiming {
    // The fine and 4x4 modes deliberately retain the same PLL. Preserve
    // physical integration time across the temporary geometry switch by
    // converting line counts through their different line lengths. Merely
    // copying the fine line count would make the global frame about 5x darker.
    let integration_clocks = u64::from(fine.coarse_lines.max(1)) * u64::from(FINE_LINE_LENGTH);
    let rounded_lines =
        (integration_clocks + u64::from(COARSE_LINE_LENGTH) / 2) / u64::from(COARSE_LINE_LENGTH);
    let maximum_exposure = u16::MAX - COARSE_EXPOSURE_MARGIN_LINES;
    let coarse_lines = rounded_lines.clamp(1, u64::from(maximum_exposure)) as u16;
    ExposureTiming {
        frame_length: COARSE_FRAME_LENGTH
            .max(coarse_lines.saturating_add(COARSE_EXPOSURE_MARGIN_LINES)),
        coarse_lines,
    }
}

#[repr(C)]
struct I2cMessage {
    address: u16,
    flags: u16,
    length: u16,
    buffer: *mut u8,
}

#[repr(C)]
struct I2cTransaction {
    messages: *mut I2cMessage,
    count: u32,
}

struct Vcm {
    file: File,
    last_position: Option<u16>,
    target_position: Option<u16>,
    settle_deadline: Option<Instant>,
    generation: u64,
}

impl Vcm {
    fn open() -> Result<Self, String> {
        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .open("/dev/i2c-1")
            .map_err(|error| format!("open VCM /dev/i2c-1: {error}"))?;
        let mut vcm = Self {
            file,
            last_position: None,
            target_position: None,
            settle_deadline: None,
            generation: 0,
        };
        let chip_id = vcm.read_register(0x00)?;
        vcm.write_register(0x02, 0x00)?;
        thread::sleep(Duration::from_millis(1));
        vcm.write_register(0x02, 0x02)?;
        thread::sleep(Duration::from_millis(1));
        vcm.last_position = vcm.read_position().ok();
        vcm.target_position = vcm.last_position;
        eprintln!("VCM active: GT97xx id=0x{chip_id:02x} control=0x02");
        Ok(vcm)
    }

    fn transact(&self, messages: &mut [I2cMessage]) -> Result<(), String> {
        let mut transaction = I2cTransaction {
            messages: messages.as_mut_ptr(),
            count: messages.len() as u32,
        };
        let result = unsafe { ioctl(self.file.as_raw_fd(), I2C_RDWR, &mut transaction) };
        if result < 0 {
            Err(std::io::Error::last_os_error().to_string())
        } else {
            Ok(())
        }
    }

    fn write_register(&self, register: u8, value: u8) -> Result<(), String> {
        let mut bytes = [register, value];
        let mut messages = [I2cMessage {
            address: VCM_I2C_ADDRESS,
            flags: 0,
            length: bytes.len() as u16,
            buffer: bytes.as_mut_ptr(),
        }];
        self.transact(&mut messages)
            .map_err(|error| format!("VCM write reg 0x{register:02x}: {error}"))
    }

    fn read_register(&self, register: u8) -> Result<u8, String> {
        let mut register_buffer = [register];
        let mut value = [0u8];
        let mut messages = [
            I2cMessage {
                address: VCM_I2C_ADDRESS,
                flags: 0,
                length: 1,
                buffer: register_buffer.as_mut_ptr(),
            },
            I2cMessage {
                address: VCM_I2C_ADDRESS,
                flags: I2C_M_RD,
                length: 1,
                buffer: value.as_mut_ptr(),
            },
        ];
        self.transact(&mut messages)
            .map_err(|error| format!("VCM read reg 0x{register:02x}: {error}"))?;
        Ok(value[0])
    }

    fn read_position(&self) -> Result<u16, String> {
        let high = self.read_register(0x03)? as u16 & 0x03;
        let low = self.read_register(0x04)? as u16;
        Ok((high << 8) | low)
    }

    fn write_position(&self, position: u16) -> Result<(), String> {
        let mut bytes = [
            0x03,
            ((position >> 8) & 0x03) as u8,
            (position & 0xff) as u8,
        ];
        let mut messages = [I2cMessage {
            address: VCM_I2C_ADDRESS,
            flags: 0,
            length: bytes.len() as u16,
            buffer: bytes.as_mut_ptr(),
        }];
        self.transact(&mut messages)
            .map_err(|error| format!("VCM write DAC position {position}: {error}"))
    }

    fn set_position(&mut self, position: u16) -> Result<u16, String> {
        if position > 1023 {
            return Err(format!("VCM position {position} is outside 0..1023"));
        }
        let from = self
            .read_position()
            .ok()
            .or(self.last_position)
            .unwrap_or(position);
        // The host bounds autofocus moves and avoids the mechanical endpoints.
        // Send one atomic target here: rapid software writes race the GT97xx's
        // own actuator transition and can leave its readable DAC lagging for
        // seconds. Mechanical stillness is represented by the conservative
        // deadline below and confirmed by matching target/readback.
        self.write_position(position)?;
        self.last_position = Some(position);
        self.target_position = Some(position);
        self.generation = self.generation.wrapping_add(1);
        self.settle_deadline = Some(Instant::now() + vcm_settle_duration(from, position));
        Ok(self.read_position().unwrap_or(position))
    }

    fn focus_status(&mut self) -> Result<FocusStatus, String> {
        let readback = self
            .read_position()
            .or_else(|error| self.last_position.ok_or(error))?;
        let target = self.target_position.unwrap_or(readback);
        let remaining = self
            .settle_deadline
            .map(|deadline| deadline.saturating_duration_since(Instant::now()))
            .unwrap_or_default();
        Ok(FocusStatus {
            target,
            readback,
            settled: remaining.is_zero() && readback == target,
            remaining_ms: remaining.as_millis().min(u64::MAX as u128) as u64,
            generation: self.generation,
        })
    }
}

#[repr(C)]
#[derive(Clone, Copy, Default)]
struct WindowRect {
    x: u16,
    y: u16,
    width: u16,
    height: u16,
}

#[repr(C)]
#[derive(Clone, Copy, Default)]
struct WindowSize {
    width: u16,
    height: u16,
}

#[repr(C)]
#[derive(Clone, Copy, Default)]
struct SensorPlaneInfo {
    plane_id: u32,
    sensor_name: [i8; 32],
    capture: WindowRect,
    bayer_id: u32,
    pixel_precision: u32,
    hdr_source: u32,
    shutter_us: u32,
    sensor_gain_x1024: u32,
    compression_gain: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Default)]
struct VifDevAttr {
    words: [u32; 12],
}

#[repr(C)]
#[derive(Clone, Copy, Default)]
struct VifOutputPortAttr {
    words: [u32; 12],
}

#[repr(C)]
#[derive(Clone, Copy, Default)]
struct VifGroupAttr {
    words: [u32; 8],
}

#[repr(C)]
#[derive(Clone, Copy, Default)]
struct ChnPort {
    module_id: u32,
    device_id: u32,
    channel_id: u32,
    port_id: u32,
}

#[link(name = "mi_sys")]
unsafe extern "C" {
    fn MI_SYS_Init(soc: u16) -> i32;
    fn MI_SYS_Exit(soc: u16) -> i32;
    fn MI_SYS_SetChnOutputPortDepth(
        soc: u16,
        port: *mut ChnPort,
        user_depth: u32,
        total_depth: u32,
    ) -> i32;
    fn MI_SYS_GetFd(port: *mut ChnPort, fd: *mut i32) -> i32;
    fn MI_SYS_CloseFd(fd: i32) -> i32;
    fn MI_SYS_ChnOutputPortGetBuf(port: *mut ChnPort, info: *mut c_void, handle: *mut usize)
        -> i32;
    fn MI_SYS_ChnOutputPortPutBuf(handle: usize) -> i32;
    fn MI_SYS_FlushInvCache(address: *mut c_void, size: u32) -> i32;
}

#[link(name = "mi_sensor")]
unsafe extern "C" {
    fn MI_SNR_InitDev() -> i32;
    fn MI_SNR_DeInitDev() -> i32;
    fn MI_SNR_SetPlaneMode(pad: u32, enable: u32) -> i32;
    fn MI_SNR_SetOrien(pad: u32, mirror: u32, flip: u32) -> i32;
    fn MI_SNR_GetRes(pad: u32, index: u8, info: *mut c_void) -> i32;
    fn MI_SNR_SetRes(pad: u32, index: u8) -> i32;
    fn MI_SNR_SetFps(pad: u32, fps: u32) -> i32;
    fn MI_SNR_Enable(pad: u32) -> i32;
    fn MI_SNR_Disable(pad: u32) -> i32;
    fn MI_SNR_GetPlaneInfo(pad: u32, plane: u32, info: *mut SensorPlaneInfo) -> i32;
}

#[link(name = "mi_vif")]
unsafe extern "C" {
    fn MI_VIF_CreateDevGroup(group: u32, attr: *mut VifGroupAttr) -> i32;
    fn MI_VIF_DestroyDevGroup(group: u32) -> i32;
    fn MI_VIF_SetDevAttr(device: u32, attr: *mut VifDevAttr) -> i32;
    fn MI_VIF_EnableDev(device: u32) -> i32;
    fn MI_VIF_DisableDev(device: u32) -> i32;
    fn MI_VIF_SetOutputPortAttr(device: u32, port: u32, attr: *mut VifOutputPortAttr) -> i32;
    fn MI_VIF_EnableOutputPort(device: u32, port: u32) -> i32;
    fn MI_VIF_DisableOutputPort(device: u32, port: u32) -> i32;
}

unsafe extern "C" {
    fn ioctl(fd: c_int, request: c_ulong, ...) -> c_int;
}

#[derive(Debug, Clone)]
struct Config {
    listen: String,
    vcm_control: String,
    tile_width: u32,
    tile_height: u32,
    initial_x: u32,
    initial_y: u32,
    settle_frames: u32,
    frame_timeout: Duration,
    frame_length: u16,
    coarse: u16,
    gain: u16,
    sensor_resolution: u8,
    sensor_fps: u32,
    sensor_binning: u32,
    sensor_scale: u32,
    direct_full_raw: bool,
    pixel_format: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum SensorTransition {
    Unchanged,
    LiveOrigin,
    LiveGeometry,
    Rebuild,
}

fn sensor_transition(current: &Config, next: &Config) -> SensorTransition {
    let same_pipeline = current.tile_width == next.tile_width
        && current.tile_height == next.tile_height
        && current.settle_frames == next.settle_frames
        && current.frame_timeout == next.frame_timeout
        && current.frame_length == next.frame_length
        && current.coarse == next.coarse
        && current.gain == next.gain
        && current.sensor_resolution == next.sensor_resolution
        && current.sensor_fps == next.sensor_fps
        && current.sensor_binning == next.sensor_binning
        && current.sensor_scale == next.sensor_scale
        && current.direct_full_raw == next.direct_full_raw
        && current.pixel_format == next.pixel_format;
    if same_pipeline && current.initial_x == next.initial_x && current.initial_y == next.initial_y {
        SensorTransition::Unchanged
    } else if same_pipeline
        && current.direct_full_raw
        && current.sensor_resolution != 1
        && current.sensor_binning == 1
    {
        SensorTransition::LiveOrigin
    } else if current.direct_full_raw
        && next.direct_full_raw
        && current.sensor_resolution != 1
        && next.sensor_resolution != 1
        && matches!(current.sensor_binning, 1 | 4)
        && matches!(next.sensor_binning, 1 | 4)
        && current.pixel_format == next.pixel_format
    {
        // The sensor's direct RAW modes share the same MIPI electrical timing
        // during a fast transition.  Retarget the IMX582 and VIF directly
        // without disabling/re-enabling MI_SNR or reloading a kernel table.
        SensorTransition::LiveGeometry
    } else {
        SensorTransition::Rebuild
    }
}

#[derive(Clone, Copy, Debug)]
struct LiveEyeRoi {
    // Absolute physical-sensor coordinates.  The sensor/VIF graph captures a
    // host-selected band; the camera service performs the horizontal and
    // final vertical slice extraction before anything crosses USB Ethernet.
    eyes: [(u32, u32); 2],
    eye_width: u32,
    eye_height: u32,
    generation: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct SensorMode {
    sensor_x: u32,
    sensor_y: u32,
    physical_width: u32,
    physical_height: u32,
    binning: u32,
    output_width: u32,
    output_height: u32,
}

impl SensorMode {
    fn from_config(config: &Config) -> Self {
        Self {
            sensor_x: config.initial_x,
            sensor_y: config.initial_y,
            physical_width: config.tile_width * config.sensor_binning * config.sensor_scale,
            physical_height: config.tile_height * config.sensor_binning * config.sensor_scale,
            binning: config.sensor_binning * config.sensor_scale,
            output_width: config.tile_width,
            output_height: config.tile_height,
        }
    }

    fn response(self) -> String {
        format!(
            "OK SENSOR {} {} {} {} {} {} {}\n",
            self.sensor_x,
            self.sensor_y,
            self.physical_width,
            self.physical_height,
            self.binning,
            self.output_width,
            self.output_height,
        )
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct ContextStream {
    every: u32,
    width: u32,
    height: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum CaptureFormat {
    Gray16,
    Raw10,
}

impl CaptureFormat {
    fn parse(value: Option<&str>) -> Result<Self, String> {
        match value {
            None => Ok(Self::Gray16),
            Some(value) if value.eq_ignore_ascii_case("GRAY16") => Ok(Self::Gray16),
            Some(value) if value.eq_ignore_ascii_case("RAW10") => Ok(Self::Raw10),
            Some(value) => Err(format!(
                "capture format must be GRAY16 or RAW10, got {value}"
            )),
        }
    }

    fn label(self) -> &'static str {
        match self {
            Self::Gray16 => "GRAY16",
            Self::Raw10 => "RAW10",
        }
    }
}

enum ClientAction {
    Continue,
    Shutdown,
    SensorMode {
        stream: TcpStream,
        config: Config,
    },
    CoarseCapture {
        stream: TcpStream,
        sequence: u64,
        width: u32,
        height: u32,
        sensor_scaled: bool,
        format: CaptureFormat,
    },
}

impl Default for Config {
    fn default() -> Self {
        Self {
            listen: "0.0.0.0:5001".to_string(),
            vcm_control: "0.0.0.0:5002".to_string(),
            tile_width: 1000,
            tile_height: 1000,
            initial_x: 3500,
            initial_y: 2500,
            settle_frames: 1,
            frame_timeout: Duration::from_millis(1500),
            frame_length: FINE_FRAME_LENGTH,
            coarse: FINE_EXPOSURE,
            gain: FINE_GAIN,
            sensor_resolution: 0,
            sensor_fps: 15,
            sensor_binning: 1,
            sensor_scale: 1,
            direct_full_raw: true,
            pixel_format: RAW10_RG,
        }
    }
}

fn sensor_crop_config(
    base: &Config,
    sensor_x: u32,
    sensor_y: u32,
    physical_width: u32,
    physical_height: u32,
    binning: u32,
) -> Result<Config, String> {
    if !matches!(binning, 1 | 4) {
        return Err("sensor crop binning must be 1 or 4".to_string());
    }
    if physical_width == 0
        || physical_height == 0
        || sensor_x % binning != 0
        || sensor_y % binning != 0
        || physical_width % binning != 0
        || physical_height % binning != 0
    {
        return Err(format!(
            "physical crop {sensor_x},{sensor_y} {physical_width}x{physical_height} must align to {binning}x{binning} readout"
        ));
    }
    let mut next = base.clone();
    next.initial_x = sensor_x;
    next.initial_y = sensor_y;
    next.tile_width = physical_width / binning;
    next.tile_height = physical_height / binning;
    next.sensor_binning = binning;
    next.sensor_scale = 1;
    match binning {
        1 => {
            next.sensor_resolution = 0;
            next.sensor_fps = 10;
            next.frame_length = FINE_FRAME_LENGTH;
            next.coarse = FINE_EXPOSURE;
            next.gain = FINE_GAIN;
        }
        4 => {
            next.sensor_resolution = 2;
            next.sensor_fps = 15;
            next.frame_length = COARSE_FRAME_LENGTH;
            // The full-sensor 4x4 thumbnail is used only for brief subject
            // acquisition. Preserve a near-frame-length exposure and use the
            // proven scan gain to compensate for the much shorter native 4x4
            // line time.
            next.coarse = COARSE_EXPOSURE;
            next.gain = COARSE_GAIN;
        }
        _ => unreachable!(),
    }
    validate_config(&next)?;
    Ok(next)
}

fn full_sensor_scaled_config(base: &Config) -> Result<Config, String> {
    let mut next = sensor_crop_config(base, 0, 0, SENSOR_WIDTH, SENSOR_HEIGHT, 4)?;
    // The native 4x4 path is 2000x1500. Apply an exact source-owned 2x MIPI
    // CCS scaler ratio so the sensor sends 1000x750 while retaining the full
    // 8000x6000 field. This is deliberately separate from the cropped stock
    // preview descriptor.
    next.tile_width = 1000;
    next.tile_height = 750;
    next.sensor_scale = 2;
    validate_config(&next)?;
    Ok(next)
}

fn scaled_acquisition_config(base: &Config) -> Result<Config, String> {
    let mut next = base.clone();
    next.initial_x = PREVIEW_SENSOR_X;
    next.initial_y = PREVIEW_SENSOR_Y;
    next.tile_width = PREVIEW_WIDTH;
    next.tile_height = PREVIEW_HEIGHT;
    // Effective physical-pixel scale: the proven module table performs 2x2
    // sensor binning and a further 2x scaler reduction.
    next.sensor_binning = 4;
    next.sensor_scale = 1;
    next.sensor_resolution = 1;
    next.sensor_fps = 60;
    next.direct_full_raw = true;
    validate_config(&next)?;
    Ok(next)
}

struct Sensor {
    file: File,
    width: u32,
    height: u32,
    binning: u32,
    scale: u32,
}

impl Sensor {
    fn open(width: u32, height: u32, binning: u32, scale: u32) -> Result<Self, String> {
        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .open("/dev/i2c-1")
            .map_err(|e| format!("open /dev/i2c-1: {e}"))?;
        let ret = unsafe { ioctl(file.as_raw_fd(), I2C_SLAVE_FORCE, SENSOR_I2C_ADDRESS) };
        if ret < 0 {
            return Err(format!(
                "I2C_SLAVE_FORCE failed: {}",
                std::io::Error::last_os_error()
            ));
        }
        Ok(Self {
            file,
            width,
            height,
            binning,
            scale,
        })
    }

    fn physical_width(&self) -> u32 {
        self.width * self.binning * self.scale
    }

    fn physical_height(&self) -> u32 {
        self.height * self.binning * self.scale
    }

    fn write_register(&mut self, register: u16, value: u8) -> Result<(), String> {
        self.file
            .write_all(&[(register >> 8) as u8, register as u8, value])
            .map_err(|e| format!("write IMX582 register 0x{register:04x}: {e}"))
    }

    fn read_register(&mut self, register: u16) -> Result<u8, String> {
        self.file
            .write_all(&[(register >> 8) as u8, register as u8])
            .map_err(|e| format!("select IMX582 register 0x{register:04x}: {e}"))?;
        let mut value = [0u8; 1];
        self.file
            .read_exact(&mut value)
            .map_err(|e| format!("read IMX582 register 0x{register:04x}: {e}"))?;
        Ok(value[0])
    }

    fn verify_full_raw_mode(&mut self, x: u32, y: u32) -> Result<(), String> {
        if self.binning != 1 {
            return Err(format!(
                "physical 1x1 RAW attestation cannot accept configured {}x{} binning",
                self.binning, self.binning,
            ));
        }
        self.validate_origin(x, y)?;
        let x1 = x + self.width - 1;
        let y1 = y + self.height - 1;
        let expected = [
            (0x0100, 0x01),
            (0x0112, 0x0a),
            (0x0113, 0x0a),
            (0x0344, (x >> 8) as u8),
            (0x0345, x as u8),
            (0x0346, (y >> 8) as u8),
            (0x0347, y as u8),
            (0x0348, (x1 >> 8) as u8),
            (0x0349, x1 as u8),
            (0x034a, (y1 >> 8) as u8),
            (0x034b, y1 as u8),
            (0x034c, (self.width >> 8) as u8),
            (0x034d, self.width as u8),
            (0x034e, (self.height >> 8) as u8),
            (0x034f, self.height as u8),
            // Odd increments of one prove that neither row nor column
            // subsampling is being used ahead of the RAW10 output stage.
            (0x0381, 0x01),
            (0x0383, 0x01),
            (0x0385, 0x01),
            (0x0387, 0x01),
            // The digital scaler is disabled and its crop is exactly the
            // sensor output geometry.  The fine eye payload therefore cannot
            // be an enlargement of a smaller sensor image.
            (0x0401, 0x00),
            (0x0408, 0x00),
            (0x0409, 0x00),
            (0x040a, 0x00),
            (0x040b, 0x00),
            (0x040c, (self.width >> 8) as u8),
            (0x040d, self.width as u8),
            (0x040e, (self.height >> 8) as u8),
            (0x040f, self.height as u8),
            // Sony binning enable is off; 0x11 is the 1x1 horizontal/vertical
            // ratio rather than the 0x44 used by the coarse 4x4 experiment.
            (0x0900, 0x00),
            (0x0901, 0x11),
            (0x0902, 0x0a),
            (0x3246, 0x01),
            (0x3247, 0x01),
        ];
        for (register, wanted) in expected {
            let actual = self.read_register(register)?;
            if actual != wanted {
                return Err(format!(
                    "FATAL physical 1x1 RAW attestation failed: IMX582 0x{register:04x}=0x{actual:02x}, expected 0x{wanted:02x} for {x},{y} {}x{}",
                    self.width, self.height,
                ));
            }
        }
        Ok(())
    }

    fn verify_scaled_global_mode(&mut self) -> Result<(), String> {
        if self.binning != 4 || self.scale != 2 || self.width != 1000 || self.height != 750 {
            return Err(format!(
                "scaled-global attestation requires 1000x750, 4x4 binning and 2x scaling; got {}x{} {}x4 binning {}x scaling",
                self.width, self.height, self.binning, self.scale,
            ));
        }
        let expected = [
            (0x0100, 0x01),
            (0x0112, 0x0a),
            (0x0113, 0x0a),
            (0x0344, 0x00),
            (0x0345, 0x00),
            (0x0346, 0x00),
            (0x0347, 0x00),
            (0x0348, 0x1f),
            (0x0349, 0x3f),
            (0x034a, 0x17),
            (0x034b, 0x6f),
            (0x034c, 0x03),
            (0x034d, 0xe8),
            (0x034e, 0x02),
            (0x034f, 0xee),
            (0x0401, 0x02),
            (0x0404, 0x00),
            (0x0405, 0x20),
            (0x0408, 0x00),
            (0x0409, 0x00),
            (0x040a, 0x00),
            (0x040b, 0x00),
            (0x040c, 0x07),
            (0x040d, 0xd0),
            (0x040e, 0x05),
            (0x040f, 0xdc),
            (0x0900, 0x01),
            (0x0901, 0x44),
            (0x0902, 0x08),
        ];
        for (register, wanted) in expected {
            let actual = self.read_register(register)?;
            if actual != wanted {
                return Err(format!(
                    "scaled-global attestation failed: IMX582 0x{register:04x}=0x{actual:02x}, expected 0x{wanted:02x}",
                ));
            }
        }
        eprintln!(
            "attested IMX582 full-field 4x4 binning plus 2x digital scaling: 8000x6000 -> 2000x1500 -> 1000x750"
        );
        Ok(())
    }

    fn validate_origin(&self, x: u32, y: u32) -> Result<(), String> {
        if x & 3 != 0 || y & 1 != 0 {
            return Err(format!(
                "sensor origin must be x/4 and y/2 aligned, got {x},{y}"
            ));
        }
        let physical_width = self.physical_width();
        let physical_height = self.physical_height();
        if x > SENSOR_WIDTH
            || y > SENSOR_HEIGHT
            || physical_width > SENSOR_WIDTH - x
            || physical_height > SENSOR_HEIGHT - y
        {
            return Err(format!(
                "{}x binned tile {x},{y} output={}x{} physical={}x{} exceeds {}x{} sensor",
                self.binning,
                self.width,
                self.height,
                physical_width,
                physical_height,
                SENSOR_WIDTH,
                SENSOR_HEIGHT
            ));
        }
        Ok(())
    }

    fn apply_full_raw_mode(&mut self, x: u32, y: u32) -> Result<(), String> {
        if self.binning != 1 {
            return Err(format!(
                "1x1 full-raw programming cannot use {}x binning",
                self.binning
            ));
        }
        self.validate_origin(x, y)?;
        let x1 = x + self.width - 1;
        let y1 = y + self.height - 1;
        let registers: &[(u16, u8)] = &[
            (0x0100, 0x00),
            (0x0112, 0x0a),
            (0x0113, 0x0a),
            (0x0303, 0x04),
            (0x0306, 0x01),
            (0x0307, 0x68),
            (0x030d, 0x06),
            (0x030e, 0x02),
            (0x030f, 0x71),
            (0x0340, 0x17),
            (0x0341, 0xac),
            (0x0342, 0x39),
            (0x0343, 0x70),
            (0x0900, 0x00),
            (0x0901, 0x11),
            (0x0902, 0x0a),
            (0x3246, 0x01),
            (0x3247, 0x01),
            (0x3620, 0x00),
            (0x3c13, 0x2a),
            (0x3f0c, 0x00),
            (0x3f14, 0x01),
            (0x3f80, 0x02),
            (0x3f81, 0x00),
            (0x3f8c, 0x01),
            (0x3f8d, 0x00),
            (0x0344, (x >> 8) as u8),
            (0x0345, x as u8),
            (0x0346, (y >> 8) as u8),
            (0x0347, y as u8),
            (0x0348, (x1 >> 8) as u8),
            (0x0349, x1 as u8),
            (0x034a, (y1 >> 8) as u8),
            (0x034b, y1 as u8),
            (0x034c, (self.width >> 8) as u8),
            (0x034d, self.width as u8),
            (0x034e, (self.height >> 8) as u8),
            (0x034f, self.height as u8),
            (0x0381, 0x01),
            (0x0383, 0x01),
            (0x0385, 0x01),
            (0x0387, 0x01),
            (0x0401, 0x00),
            (0x0408, 0x00),
            (0x0409, 0x00),
            (0x040a, 0x00),
            (0x040b, 0x00),
            (0x040c, (self.width >> 8) as u8),
            (0x040d, self.width as u8),
            (0x040e, (self.height >> 8) as u8),
            (0x040f, self.height as u8),
            (0x0100, 0x01),
        ];
        for &(register, value) in registers {
            self.write_register(register, value)?;
            thread::sleep(Duration::from_millis(1));
        }
        thread::sleep(Duration::from_millis(100));
        Ok(())
    }

    fn start_configured_module_mode(&mut self) -> Result<(), String> {
        // MI_SNR_SetRes programs the proven stock scaler table but leaves the
        // IMX582 in standby for this standalone graph.  Do not overwrite that
        // table's crop, scaler, PLL, frame length, or exposure here.
        self.write_register(0x0100, 0x01)?;
        thread::sleep(Duration::from_millis(100));
        Ok(())
    }

    fn apply_binned_raw_mode(&mut self, x: u32, y: u32) -> Result<(), String> {
        if self.binning != 4 {
            return Err(format!(
                "direct binned acquisition supports 4x4, got {}x{}",
                self.binning, self.binning
            ));
        }
        self.validate_origin(x, y)?;
        let x1 = x + self.physical_width() - 1;
        let y1 = y + self.physical_height() - 1;
        // MI_SNR_SetRes loads the custom resolution-2 table, but this
        // standalone graph does not transition that table out of standby.
        // Reassert the complete window/output tuple before starting it.  The
        // module supplies the matching PLL, line length and frame length.
        let registers: &[(u16, u8)] = &[
            (0x0100, 0x00),
            (0x0112, 0x0a),
            (0x0113, 0x0a),
            (0x0114, 0x03),
            (0x0344, (x >> 8) as u8),
            (0x0345, x as u8),
            (0x0346, (y >> 8) as u8),
            (0x0347, y as u8),
            (0x0348, (x1 >> 8) as u8),
            (0x0349, x1 as u8),
            (0x034a, (y1 >> 8) as u8),
            (0x034b, y1 as u8),
            (0x0900, 0x01),
            (0x0901, 0x44),
            (0x0902, 0x08),
            (0x3246, 0x89),
            (0x3247, 0x89),
            (0x0408, 0x00),
            (0x0409, 0x00),
            (0x040a, 0x00),
            (0x040b, 0x00),
            (0x040c, (self.width >> 8) as u8),
            (0x040d, self.width as u8),
            (0x040e, (self.height >> 8) as u8),
            (0x040f, self.height as u8),
            (0x034c, (self.width >> 8) as u8),
            (0x034d, self.width as u8),
            (0x034e, (self.height >> 8) as u8),
            (0x034f, self.height as u8),
            (0x0100, 0x01),
        ];
        for &(register, value) in registers {
            self.write_register(register, value)?;
            thread::sleep(Duration::from_millis(1));
        }
        thread::sleep(Duration::from_millis(100));
        Ok(())
    }

    fn set_exposure(&mut self, frame_length: u16, coarse: u16, gain: u16) -> Result<(), String> {
        let coarse = coarse.clamp(1, frame_length.saturating_sub(1));
        self.write_register(0x0104, 0x01)?;
        let result = (|| {
            self.write_register(0x0340, (frame_length >> 8) as u8)?;
            self.write_register(0x0341, frame_length as u8)?;
            self.write_register(0x0202, (coarse >> 8) as u8)?;
            self.write_register(0x0203, coarse as u8)?;
            self.write_register(0x0204, (gain >> 8) as u8)?;
            self.write_register(0x0205, gain as u8)
        })();
        let release = self.write_register(0x0104, 0x00);
        result.and(release)
    }

    fn set_origin_live(&mut self, x: u32, y: u32) -> Result<(), String> {
        self.validate_origin(x, y)?;
        let x1 = x + self.physical_width() - 1;
        let y1 = y + self.physical_height() - 1;
        self.write_register(0x0104, 0x01)?;
        let result = (|| {
            for &(register, value) in &[
                (0x0344, (x >> 8) as u8),
                (0x0345, x as u8),
                (0x0346, (y >> 8) as u8),
                (0x0347, y as u8),
                (0x0348, (x1 >> 8) as u8),
                (0x0349, x1 as u8),
                (0x034a, (y1 >> 8) as u8),
                (0x034b, y1 as u8),
            ] {
                self.write_register(register, value)?;
            }
            Ok(())
        })();
        let release = self.write_register(0x0104, 0x00);
        result.and(release)
    }

    fn prepare_geometry_live(&mut self, config: &Config) -> Result<(), String> {
        if !config.direct_full_raw
            || !matches!(config.sensor_binning, 1 | 4)
            || !matches!(config.sensor_scale, 1 | 2)
            || (config.sensor_binning == 1 && config.sensor_scale != 1)
        {
            return Err("live geometry requires a direct 1x1 or 4x4 RAW mode".to_string());
        }
        let physical_width = config
            .tile_width
            .checked_mul(config.sensor_binning)
            .and_then(|value| value.checked_mul(config.sensor_scale))
            .ok_or_else(|| "live sensor width overflow".to_string())?;
        let physical_height = config
            .tile_height
            .checked_mul(config.sensor_binning)
            .and_then(|value| value.checked_mul(config.sensor_scale))
            .ok_or_else(|| "live sensor height overflow".to_string())?;
        if config.initial_x & 3 != 0
            || config.initial_y & 1 != 0
            || config.initial_x > SENSOR_WIDTH.saturating_sub(physical_width)
            || config.initial_y > SENSOR_HEIGHT.saturating_sub(physical_height)
        {
            return Err(format!(
                "live sensor geometry {},{} {}x{} exceeds the aligned {}x{} array",
                config.initial_x,
                config.initial_y,
                physical_width,
                physical_height,
                SENSOR_WIDTH,
                SENSOR_HEIGHT,
            ));
        }

        let x1 = config.initial_x + physical_width - 1;
        let y1 = config.initial_y + physical_height - 1;
        let (bin_enable, bin_type, bin_weight, quad_a, quad_b) = if config.sensor_binning == 1 {
            (0x00, 0x11, 0x0a, 0x01, 0x01)
        } else {
            (0x01, 0x44, 0x08, 0x89, 0x89)
        };
        let (mode_38a8, mode_38a9, mode_38aa, mode_38ab) = if config.sensor_binning == 1 {
            (0x01, 0xe0, 0x01, 0x68)
        } else {
            (0x00, 0xf0, 0x00, 0xb4)
        };
        let (mode_3c13, mode_3f14, mode_3f80, mode_3f8c, mode_3ff5, mode_3ffc, mode_3ffd) =
            if config.sensor_binning == 1 {
                (0x2a, 0x01, 0x02, 0x01, 0x00, 0x04, 0xb0)
            } else {
                (0x00, 0x00, 0x00, 0x00, 0x4c, 0x00, 0x00)
            };
        let coarse = config
            .coarse
            .clamp(1, config.frame_length.saturating_sub(1));
        let line_length = if config.sensor_binning == 1 {
            FINE_LINE_LENGTH
        } else {
            COARSE_LINE_LENGTH
        };
        let scaler_input_width = config.tile_width * config.sensor_scale;
        let scaler_input_height = config.tile_height * config.sensor_scale;
        let scale_m = (config.sensor_scale * 16) as u16;

        // Geometry/binning registers do not become active through group hold
        // on this sensor (origin-only changes do).  Pulse sensor standby
        // directly, but keep the proven fine-mode PLL, line length and MIPI
        // lane rate fixed so the CSI receiver never has to relock.
        self.write_register(0x0100, 0x00)?;
        let result = (|| {
            for &(register, value) in &[
                (0x0112, 0x0a),
                (0x0113, 0x0a),
                (0x0114, 0x03),
                (0x0340, (config.frame_length >> 8) as u8),
                (0x0341, config.frame_length as u8),
                (0x0342, (line_length >> 8) as u8),
                (0x0343, line_length as u8),
                (0x0202, (coarse >> 8) as u8),
                (0x0203, coarse as u8),
                (0x0204, (config.gain >> 8) as u8),
                (0x0205, config.gain as u8),
                (0x0344, (config.initial_x >> 8) as u8),
                (0x0345, config.initial_x as u8),
                (0x0346, (config.initial_y >> 8) as u8),
                (0x0347, config.initial_y as u8),
                (0x0348, (x1 >> 8) as u8),
                (0x0349, x1 as u8),
                (0x034a, (y1 >> 8) as u8),
                (0x034b, y1 as u8),
                (0x034c, (config.tile_width >> 8) as u8),
                (0x034d, config.tile_width as u8),
                (0x034e, (config.tile_height >> 8) as u8),
                (0x034f, config.tile_height as u8),
                (0x0401, if config.sensor_scale == 1 { 0x00 } else { 0x02 }),
                (0x0404, (scale_m >> 8) as u8),
                (0x0405, scale_m as u8),
                (0x0408, 0x00),
                (0x0409, 0x00),
                (0x040a, 0x00),
                (0x040b, 0x00),
                (0x040c, (scaler_input_width >> 8) as u8),
                (0x040d, scaler_input_width as u8),
                (0x040e, (scaler_input_height >> 8) as u8),
                (0x040f, scaler_input_height as u8),
                (0x0900, bin_enable),
                (0x0901, bin_type),
                (0x0902, bin_weight),
                (0x3246, quad_a),
                (0x3247, quad_b),
                (0x38a8, mode_38a8),
                (0x38a9, mode_38a9),
                (0x38aa, mode_38aa),
                (0x38ab, mode_38ab),
                (0x3c13, mode_3c13),
                (0x3f14, mode_3f14),
                (0x3f80, mode_3f80),
                (0x3f8c, mode_3f8c),
                (0x3ff5, mode_3ff5),
                (0x3ffc, mode_3ffc),
                (0x3ffd, mode_3ffd),
            ] {
                self.write_register(register, value)?;
            }
            Ok(())
        })();
        if let Err(error) = result {
            // Do not strand the sensor in standby if a register write fails.
            let _ = self.write_register(0x0100, 0x01);
            return Err(error);
        }
        self.width = config.tile_width;
        self.height = config.tile_height;
        self.binning = config.sensor_binning;
        self.scale = config.sensor_scale;
        Ok(())
    }

    fn restart_geometry_live(&mut self) -> Result<(), String> {
        self.write_register(0x0100, 0x01)
    }
}

struct RawFrame {
    bytes: Vec<u8>,
    stride: u32,
    timestamp_ns: u64,
}

struct RawEyeSet {
    payloads: [Vec<u8>; 2],
    context: Option<Vec<u8>>,
    timestamp_ns: u64,
    sensor_wait: Duration,
    roi_slice: Duration,
    context_build: Duration,
}

struct Graph {
    sensor: Option<Sensor>,
    port: ChnPort,
    output_fd: i32,
    sys_initialized: bool,
    snr_initialized: bool,
    sensor_enabled: bool,
    group_created: bool,
    device_enabled: bool,
    port_enabled: bool,
    depth_enabled: bool,
    width: u32,
    height: u32,
    frame_timeout: Duration,
    live_origin: bool,
    pixel_format: u32,
    metadata_logged: bool,
    origin_x: u32,
    origin_y: u32,
}

impl Graph {
    fn open(config: &Config) -> Result<Self, String> {
        let mut graph = Self {
            sensor: None,
            port: ChnPort {
                module_id: MI_MODULE_ID_VIF,
                device_id: 0,
                channel_id: 0,
                port_id: 0,
            },
            output_fd: -1,
            sys_initialized: false,
            snr_initialized: false,
            sensor_enabled: false,
            group_created: false,
            device_enabled: false,
            port_enabled: false,
            depth_enabled: false,
            width: config.tile_width,
            height: config.tile_height,
            frame_timeout: config.frame_timeout,
            live_origin: config.direct_full_raw,
            pixel_format: config.pixel_format,
            metadata_logged: false,
            origin_x: config.initial_x,
            origin_y: config.initial_y,
        };

        mi("MI_SYS_Init", unsafe { MI_SYS_Init(0) })?;
        graph.sys_initialized = true;
        mi("MI_SNR_InitDev", unsafe { MI_SNR_InitDev() })?;
        graph.snr_initialized = true;

        unsafe {
            MI_VIF_DisableOutputPort(0, 0);
            MI_VIF_DisableDev(0);
            MI_VIF_DestroyDevGroup(0);
            MI_SNR_Disable(0);
        }
        graph.configure_pipeline(config)?;
        Ok(graph)
    }

    fn stop_pipeline(&mut self) {
        unsafe {
            if self.depth_enabled {
                MI_SYS_SetChnOutputPortDepth(0, &mut self.port, 0, 2);
                self.depth_enabled = false;
            }
            if self.output_fd >= 0 {
                MI_SYS_CloseFd(self.output_fd);
                self.output_fd = -1;
            }
            if self.port_enabled {
                MI_VIF_DisableOutputPort(0, 0);
                self.port_enabled = false;
            }
            if self.device_enabled {
                MI_VIF_DisableDev(0);
                self.device_enabled = false;
            }
            if self.group_created {
                MI_VIF_DestroyDevGroup(0);
                self.group_created = false;
            }
            if self.sensor_enabled {
                MI_SNR_Disable(0);
                self.sensor_enabled = false;
            }
        }
        self.sensor = None;
    }

    fn reconfigure(&mut self, config: &Config) -> Result<(), String> {
        self.stop_pipeline();
        thread::sleep(PIPELINE_STOP_SETTLE);
        self.configure_pipeline(config)
    }

    fn reconfigure_geometry_live(&mut self, config: &Config) -> Result<(), String> {
        if !self.sensor_enabled || !self.group_created {
            return Err("sensor/VIF graph is not live".to_string());
        }

        // Quiesce only the DMA-facing VIF endpoint while frame boundaries are
        // still arriving. Disabling VIF after putting the IMX582 in standby
        // makes this SigmaStar generation wait for its roughly two-second
        // missing-frame timeout. The MI sensor device and MIPI group remain
        // resident throughout.
        let vif_disable_started = Instant::now();
        if self.depth_enabled {
            mi("MI_SYS_SetChnOutputPortDepth(live disable)", unsafe {
                MI_SYS_SetChnOutputPortDepth(0, &mut self.port, 0, 2)
            })?;
            self.depth_enabled = false;
        }
        if self.output_fd >= 0 {
            mi("MI_SYS_CloseFd(live geometry)", unsafe {
                MI_SYS_CloseFd(self.output_fd)
            })?;
            self.output_fd = -1;
        }
        if self.port_enabled {
            mi("MI_VIF_DisableOutputPort(live geometry)", unsafe {
                MI_VIF_DisableOutputPort(0, 0)
            })?;
            self.port_enabled = false;
        }
        if self.device_enabled {
            mi("MI_VIF_DisableDev(live geometry)", unsafe {
                MI_VIF_DisableDev(0)
            })?;
            self.device_enabled = false;
        }
        let vif_disable_elapsed = vif_disable_started.elapsed();

        // Now stop and retarget the IMX582. Restarting only after the resident
        // VIF endpoint is ready gives the receiver a clean new frame-start.
        let sensor_started = Instant::now();
        self.sensor
            .as_mut()
            .ok_or_else(|| "sensor is not open".to_string())?
            .prepare_geometry_live(config)?;
        let sensor_elapsed = sensor_started.elapsed();

        self.width = config.tile_width;
        self.height = config.tile_height;
        self.frame_timeout = config.frame_timeout;
        self.live_origin = config.direct_full_raw && config.sensor_resolution != 1;
        self.pixel_format = config.pixel_format;
        self.metadata_logged = false;
        self.origin_x = config.initial_x;
        self.origin_y = config.initial_y;

        let vif_enable_started = Instant::now();
        let geometry = (config.tile_height << 16) | config.tile_width;
        let mut device = VifDevAttr {
            words: [
                config.pixel_format,
                0,
                geometry,
                0,
                0,
                0,
                geometry,
                geometry,
                config.pixel_format,
                0,
                0,
                4,
            ],
        };
        mi("MI_VIF_SetDevAttr(live geometry)", unsafe {
            MI_VIF_SetDevAttr(0, &mut device)
        })?;
        mi("MI_VIF_EnableDev(live geometry)", unsafe {
            MI_VIF_EnableDev(0)
        })?;
        self.device_enabled = true;

        let mut output = VifOutputPortAttr {
            words: [
                0,
                geometry,
                geometry,
                config.pixel_format,
                0,
                0,
                4,
                0,
                0,
                2,
                0,
                0,
            ],
        };
        mi("MI_VIF_SetOutputPortAttr(live geometry)", unsafe {
            MI_VIF_SetOutputPortAttr(0, 0, &mut output)
        })?;
        mi("MI_VIF_EnableOutputPort(live geometry)", unsafe {
            MI_VIF_EnableOutputPort(0, 0)
        })?;
        self.port_enabled = true;
        mi("MI_SYS_SetChnOutputPortDepth(live enable)", unsafe {
            MI_SYS_SetChnOutputPortDepth(0, &mut self.port, 1, 2)
        })?;
        self.depth_enabled = true;
        mi("MI_SYS_GetFd(live geometry)", unsafe {
            MI_SYS_GetFd(&mut self.port, &mut self.output_fd)
        })?;
        if self.output_fd < 0 {
            return Err("MI_SYS_GetFd returned a negative descriptor".to_string());
        }
        let vif_enable_elapsed = vif_enable_started.elapsed();

        // Restart last so the first new frame start is presented to a fully
        // configured resident VIF endpoint.
        let restart_started = Instant::now();
        self.sensor
            .as_mut()
            .ok_or_else(|| "sensor is not open".to_string())?
            .restart_geometry_live()?;
        eprintln!(
            "resident IMX582/VIF retune: VIF-disable={} us sensor-registers={} us VIF-enable={} us sensor-restart={} us (MIPI group retained)",
            vif_disable_elapsed.as_micros(),
            sensor_elapsed.as_micros(),
            vif_enable_elapsed.as_micros(),
            restart_started.elapsed().as_micros(),
        );
        Ok(())
    }

    fn rearm_vif_output_live(&mut self) -> Result<(), String> {
        if !self.device_enabled || !self.port_enabled {
            return Err("VIF device/output port is not live".to_string());
        }
        let started = Instant::now();
        if self.depth_enabled {
            mi("MI_SYS_SetChnOutputPortDepth(re-arm disable)", unsafe {
                MI_SYS_SetChnOutputPortDepth(0, &mut self.port, 0, 2)
            })?;
            self.depth_enabled = false;
        }
        if self.output_fd >= 0 {
            mi("MI_SYS_CloseFd(re-arm)", unsafe {
                MI_SYS_CloseFd(self.output_fd)
            })?;
            self.output_fd = -1;
        }
        mi("MI_VIF_DisableOutputPort(re-arm)", unsafe {
            MI_VIF_DisableOutputPort(0, 0)
        })?;
        self.port_enabled = false;
        mi("MI_VIF_EnableOutputPort(re-arm)", unsafe {
            MI_VIF_EnableOutputPort(0, 0)
        })?;
        self.port_enabled = true;
        mi("MI_SYS_SetChnOutputPortDepth(re-arm enable)", unsafe {
            MI_SYS_SetChnOutputPortDepth(0, &mut self.port, 1, 2)
        })?;
        self.depth_enabled = true;
        mi("MI_SYS_GetFd(re-arm)", unsafe {
            MI_SYS_GetFd(&mut self.port, &mut self.output_fd)
        })?;
        if self.output_fd < 0 {
            return Err("MI_SYS_GetFd returned a negative descriptor after re-arm".to_string());
        }
        eprintln!(
            "resident VIF output-only re-arm completed in {} us",
            started.elapsed().as_micros(),
        );
        Ok(())
    }

    fn configure_pipeline(&mut self, config: &Config) -> Result<(), String> {
        self.width = config.tile_width;
        self.height = config.tile_height;
        self.frame_timeout = config.frame_timeout;
        self.live_origin = config.direct_full_raw && config.sensor_resolution != 1;
        self.pixel_format = config.pixel_format;
        self.metadata_logged = false;
        self.origin_x = config.initial_x;
        self.origin_y = config.initial_y;

        mi("MI_SNR_SetPlaneMode", unsafe { MI_SNR_SetPlaneMode(0, 0) })?;
        mi("MI_SNR_SetOrien", unsafe { MI_SNR_SetOrien(0, 0, 0) })?;
        let mut resolution_info = [0u32; 32];
        mi("MI_SNR_GetRes", unsafe {
            MI_SNR_GetRes(0, 0, resolution_info.as_mut_ptr().cast::<c_void>())
        })?;
        mi("MI_SNR_SetRes", unsafe {
            MI_SNR_SetRes(0, config.sensor_resolution)
        })?;
        if config.sensor_fps != 0 {
            mi("MI_SNR_SetFps", unsafe {
                MI_SNR_SetFps(0, config.sensor_fps)
            })?;
        }
        mi("MI_SNR_Enable", unsafe { MI_SNR_Enable(0) })?;
        self.sensor_enabled = true;
        thread::sleep(SENSOR_ENABLE_SETTLE);
        let mut plane = SensorPlaneInfo::default();
        let plane_result = unsafe { MI_SNR_GetPlaneInfo(0, 0, &mut plane) };
        eprintln!(
            "sensor plane ret={} capture={},{} {}x{} bayer={} precision={} configured-pixel={}",
            plane_result,
            plane.capture.x,
            plane.capture.y,
            plane.capture.width,
            plane.capture.height,
            plane.bayer_id,
            plane.pixel_precision,
            config.pixel_format,
        );

        let mut sensor = Sensor::open(
            config.tile_width,
            config.tile_height,
            config.sensor_binning,
            config.sensor_scale,
        )?;
        if config.direct_full_raw {
            if config.sensor_resolution == 1 {
                if config.initial_x != PREVIEW_SENSOR_X
                    || config.initial_y != PREVIEW_SENSOR_Y
                    || config.tile_width != PREVIEW_WIDTH
                    || config.tile_height != PREVIEW_HEIGHT
                    || config.sensor_binning != 4
                {
                    return Err(
                        "stock scaler acquisition mode requires physical 160,840 7680x4320 and output 1920x1080".to_string(),
                    );
                }
                sensor.start_configured_module_mode()?;
            } else {
                match config.sensor_binning {
                    1 => sensor.apply_full_raw_mode(config.initial_x, config.initial_y)?,
                    4 => sensor.apply_binned_raw_mode(config.initial_x, config.initial_y)?,
                    binning => {
                        return Err(format!(
                            "direct sensor programming does not support {binning}x{binning} binning"
                        ));
                    }
                }
                sensor.set_exposure(config.frame_length, config.coarse, config.gain)?;
                if config.sensor_binning == 1 {
                    sensor.verify_full_raw_mode(config.initial_x, config.initial_y)?;
                    eprintln!(
                        "verified physical 1x1 RAW10 sensor path: origin={},{} output={}x{}; binning/scaling/subsampling disabled",
                        config.initial_x, config.initial_y, config.tile_width, config.tile_height,
                    );
                }
            }
        }
        self.sensor = Some(sensor);

        let mut group = VifGroupAttr {
            words: [4, 0, 0, 2, 0, 0, 0, 1],
        };
        mi("MI_VIF_CreateDevGroup", unsafe {
            MI_VIF_CreateDevGroup(0, &mut group)
        })?;
        self.group_created = true;

        let geometry = (config.tile_height << 16) | config.tile_width;
        let mut device = VifDevAttr {
            words: [
                config.pixel_format,
                0,
                geometry,
                0,
                0,
                0,
                geometry,
                geometry,
                config.pixel_format,
                0,
                0,
                4,
            ],
        };
        mi("MI_VIF_SetDevAttr", unsafe {
            MI_VIF_SetDevAttr(0, &mut device)
        })?;
        mi("MI_VIF_EnableDev", unsafe { MI_VIF_EnableDev(0) })?;
        self.device_enabled = true;

        let mut output = VifOutputPortAttr {
            words: [
                0,
                geometry,
                geometry,
                config.pixel_format,
                0,
                0,
                4,
                0,
                0,
                2,
                0,
                0,
            ],
        };
        mi("MI_VIF_SetOutputPortAttr", unsafe {
            MI_VIF_SetOutputPortAttr(0, 0, &mut output)
        })?;
        mi("MI_VIF_EnableOutputPort", unsafe {
            MI_VIF_EnableOutputPort(0, 0)
        })?;
        self.port_enabled = true;

        mi("MI_SYS_SetChnOutputPortDepth", unsafe {
            MI_SYS_SetChnOutputPortDepth(0, &mut self.port, 1, 2)
        })?;
        self.depth_enabled = true;
        mi("MI_SYS_GetFd", unsafe {
            MI_SYS_GetFd(&mut self.port, &mut self.output_fd)
        })?;
        if self.output_fd < 0 {
            return Err("MI_SYS_GetFd returned a negative descriptor".to_string());
        }
        thread::sleep(VIF_ENABLE_SETTLE);
        Ok(())
    }

    fn capture_tile(&mut self, x: u32, y: u32, settle_frames: u32) -> Result<RawFrame, String> {
        let origin_changed = x != self.origin_x || y != self.origin_y;
        if origin_changed {
            if self.live_origin {
                self.sensor
                    .as_mut()
                    .ok_or_else(|| "sensor is not open".to_string())?
                    .set_origin_live(x, y)?;
            } else {
                return Err(format!(
                    "fixed sensor mode only accepts origin {},{}",
                    self.origin_x, self.origin_y
                ));
            }
            // VIF is configured with a two-buffer queue.  Frames completed
            // before the group-held crop reached a sensor boundary still
            // carry the previous physical origin, so remove every already
            // queued buffer before counting settle exposures for the new one.
            self.discard_queued_frames(2)?;
        }
        let mut result = None;
        for _ in 0..=settle_frames {
            result = Some(self.acquire_frame()?);
        }
        self.origin_x = x;
        self.origin_y = y;
        result.ok_or_else(|| "no raw frame acquired".to_string())
    }

    fn acquire_frame(&mut self) -> Result<RawFrame, String> {
        self.acquire_frame_with_timeout(self.frame_timeout)
    }

    fn acquire_frame_with_timeout(&mut self, timeout: Duration) -> Result<RawFrame, String> {
        let start = Instant::now();
        let mut info = [0u32; 512];
        let mut handle = 0usize;
        let ret = loop {
            info.fill(0);
            let ret = unsafe {
                MI_SYS_ChnOutputPortGetBuf(
                    &mut self.port,
                    info.as_mut_ptr().cast::<c_void>(),
                    &mut handle,
                )
            };
            if ret == 0 {
                break ret;
            }
            if start.elapsed() >= timeout {
                return Err(format!("raw frame timeout, last MI result {ret}"));
            }
            thread::sleep(Duration::from_millis(2));
        };
        if ret != 0 {
            return Err(format!("MI_SYS_ChnOutputPortGetBuf failed: {ret}"));
        }

        let info_bytes = unsafe {
            std::slice::from_raw_parts(info.as_ptr().cast::<u8>(), std::mem::size_of_val(&info))
        };
        let tile_mode = read_u32(info_bytes, 0x20);
        let pixel = read_u32(info_bytes, 0x24);
        let compression = read_u32(info_bytes, 0x28);
        let physical_layout = read_u32(info_bytes, 0x34);
        let width = read_u16(info_bytes, 0x38) as u32;
        let height = read_u16(info_bytes, 0x3a) as u32;
        let address = read_u32(info_bytes, 0x3c) as usize;
        let source_stride = read_u32(info_bytes, 0x60) as usize;
        let buffer_size = read_u32(info_bytes, 0x6c) as usize;
        let packed_stride = self.width.div_ceil(4) as usize * 5;

        if !self.metadata_logged {
            eprintln!(
                "RAW frame metadata tile={tile_mode} physical-layout={physical_layout} stride={source_stride} buffer-size={buffer_size} packed-line={packed_stride}"
            );
            self.metadata_logged = true;
        }

        let validation = if pixel != self.pixel_format {
            Err(format!(
                "expected RAW10 {}, got pixel format {pixel}",
                self.pixel_format
            ))
        } else if compression != 0 {
            Err(format!("refusing non-raw compression mode {compression}"))
        } else if width != self.width || height != self.height {
            Err(format!(
                "frame geometry changed: expected {}x{}, got {width}x{height}",
                self.width, self.height
            ))
        } else if address == 0 || source_stride < packed_stride {
            Err(format!(
                "invalid frame buffer address=0x{address:x} stride={source_stride}, need {packed_stride}"
            ))
        } else {
            Ok(())
        };

        let bytes = validation.and_then(|()| {
            // VIF DMA reuses cached userspace mappings across sensor modes.
            // Without invalidation, old scan-band cache lines survive inside
            // a later full-frame buffer and appear as horizontally displaced
            // temporal strips even though the metadata says linear/normal.
            mi("MI_SYS_FlushInvCache", unsafe {
                MI_SYS_FlushInvCache(address as *mut c_void, buffer_size as u32)
            })?;
            let mut raw = vec![0u8; packed_stride * self.height as usize];
            for row in 0..self.height as usize {
                let source = unsafe {
                    std::slice::from_raw_parts(
                        (address + row * source_stride) as *const u8,
                        packed_stride,
                    )
                };
                raw[row * packed_stride..(row + 1) * packed_stride].copy_from_slice(source);
            }
            Ok(raw)
        });
        let put_ret = unsafe { MI_SYS_ChnOutputPortPutBuf(handle) };
        if put_ret != 0 {
            return Err(format!("MI_SYS_ChnOutputPortPutBuf failed: {put_ret}"));
        }

        Ok(RawFrame {
            bytes: bytes?,
            stride: packed_stride as u32,
            timestamp_ns: SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64,
        })
    }

    fn acquire_eye_set(
        &mut self,
        eyes: [(u32, u32); 2],
        eye_width: u32,
        eye_height: u32,
        context: Option<ContextStream>,
    ) -> Result<RawEyeSet, String> {
        let started = Instant::now();
        let mut info = [0u32; 512];
        let mut handle = 0usize;
        let ret = loop {
            info.fill(0);
            let ret = unsafe {
                MI_SYS_ChnOutputPortGetBuf(
                    &mut self.port,
                    info.as_mut_ptr().cast::<c_void>(),
                    &mut handle,
                )
            };
            if ret == 0 {
                break ret;
            }
            if started.elapsed() >= self.frame_timeout {
                return Err(format!("raw eye frame timeout, last MI result {ret}"));
            }
            thread::sleep(Duration::from_millis(2));
        };
        if ret != 0 {
            return Err(format!("MI_SYS_ChnOutputPortGetBuf failed: {ret}"));
        }
        let sensor_wait = started.elapsed();

        let info_bytes = unsafe {
            std::slice::from_raw_parts(info.as_ptr().cast::<u8>(), std::mem::size_of_val(&info))
        };
        let pixel = read_u32(info_bytes, 0x24);
        let compression = read_u32(info_bytes, 0x28);
        let width = read_u16(info_bytes, 0x38) as u32;
        let height = read_u16(info_bytes, 0x3a) as u32;
        let address = read_u32(info_bytes, 0x3c) as usize;
        let source_stride = read_u32(info_bytes, 0x60) as usize;
        let buffer_size = read_u32(info_bytes, 0x6c) as usize;
        let packed_stride = self.width.div_ceil(4) as usize * 5;
        let required_buffer = source_stride.saturating_mul(self.height as usize);

        let validation = if pixel != self.pixel_format {
            Err(format!(
                "expected RAW10 {}, got pixel format {pixel}",
                self.pixel_format
            ))
        } else if compression != 0 {
            Err(format!("refusing non-raw compression mode {compression}"))
        } else if width != self.width || height != self.height {
            Err(format!(
                "eye frame geometry changed: expected {}x{}, got {width}x{height}",
                self.width, self.height
            ))
        } else if address == 0 || source_stride < packed_stride || buffer_size < required_buffer {
            Err(format!(
                "invalid eye frame buffer address=0x{address:x} stride={source_stride} size={buffer_size}, need stride>={packed_stride} size>={required_buffer}"
            ))
        } else {
            Ok(())
        };

        let extraction = validation.and_then(|()| {
            let relative =
                eyes.map(|(x, y)| (x.checked_sub(self.origin_x), y.checked_sub(self.origin_y)));
            let relative = relative
                .map(|(x, y)| x.zip(y))
                .into_iter()
                .collect::<Option<Vec<_>>>()
                .ok_or_else(|| "absolute eye ROI precedes the active sensor band".to_string())?;
            if relative.iter().any(|&(x, y)| {
                x.saturating_add(eye_width) > self.width
                    || y.saturating_add(eye_height) > self.height
            }) {
                return Err("absolute eye ROI exceeds the active sensor band".to_string());
            }

            // Context samples the whole sensor band, so invalidate the whole
            // DMA mapping only on those frames.  With context disabled, bound
            // cache maintenance to the row span containing the two requested
            // RAW eye rectangles.  No 8000xH intermediate image is copied.
            let (flush_address, flush_size) = if context.is_some() {
                (address, buffer_size)
            } else {
                let first_row = relative.iter().map(|&(_, y)| y).min().unwrap() as usize;
                let final_row =
                    relative.iter().map(|&(_, y)| y + eye_height).max().unwrap() as usize;
                let first = address + first_row * source_stride;
                let final_byte = address + final_row * source_stride;
                let aligned_first = first & !63usize;
                let aligned_final = final_byte.saturating_add(63) & !63usize;
                (aligned_first, aligned_final.saturating_sub(aligned_first))
            };
            mi("MI_SYS_FlushInvCache(eye source)", unsafe {
                MI_SYS_FlushInvCache(flush_address as *mut c_void, flush_size as u32)
            })?;
            let source = unsafe { std::slice::from_raw_parts(address as *const u8, buffer_size) };

            let context_started = Instant::now();
            let context_payload = context
                .map(|context| {
                    downsample_raw10_tracking_context_bytes(
                        source,
                        source_stride,
                        self.width,
                        self.height,
                        context.width,
                        context.height,
                    )
                })
                .transpose()?;
            let context_build = context_started.elapsed();

            let crop_started = Instant::now();
            let payloads = [
                crop_raw10_bytes(
                    source,
                    source_stride,
                    relative[0].0,
                    relative[0].1,
                    eye_width,
                    eye_height,
                )?,
                crop_raw10_bytes(
                    source,
                    source_stride,
                    relative[1].0,
                    relative[1].1,
                    eye_width,
                    eye_height,
                )?,
            ];
            let roi_slice = crop_started.elapsed();
            Ok((payloads, context_payload, roi_slice, context_build))
        });
        let put_ret = unsafe { MI_SYS_ChnOutputPortPutBuf(handle) };
        if put_ret != 0 {
            return Err(format!("MI_SYS_ChnOutputPortPutBuf failed: {put_ret}"));
        }
        let (payloads, context, roi_slice, context_build) = extraction?;
        Ok(RawEyeSet {
            payloads,
            context,
            timestamp_ns: SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64,
            sensor_wait,
            roi_slice,
            context_build,
        })
    }

    fn discard_queued_frames(&mut self, maximum: usize) -> Result<usize, String> {
        let mut discarded = 0usize;
        for _ in 0..maximum {
            let mut info = [0u32; 512];
            let mut handle = 0usize;
            let ret = unsafe {
                MI_SYS_ChnOutputPortGetBuf(
                    &mut self.port,
                    info.as_mut_ptr().cast::<c_void>(),
                    &mut handle,
                )
            };
            if ret != 0 {
                break;
            }
            mi("MI_SYS_ChnOutputPortPutBuf(discard queued)", unsafe {
                MI_SYS_ChnOutputPortPutBuf(handle)
            })?;
            discarded += 1;
        }
        Ok(discarded)
    }

    fn discard_next_frames(&mut self, count: usize) -> Result<(), String> {
        for index in 0..count {
            let started = Instant::now();
            let mut info = [0u32; 512];
            let mut handle = 0usize;
            loop {
                info.fill(0);
                let ret = unsafe {
                    MI_SYS_ChnOutputPortGetBuf(
                        &mut self.port,
                        info.as_mut_ptr().cast::<c_void>(),
                        &mut handle,
                    )
                };
                if ret == 0 {
                    break;
                }
                if started.elapsed() >= self.frame_timeout {
                    return Err(format!(
                        "timed out waiting for live-origin settle frame {}/{count}, last MI result {ret}",
                        index + 1,
                    ));
                }
                thread::sleep(Duration::from_millis(2));
            }
            mi("MI_SYS_ChnOutputPortPutBuf(discard settle)", unsafe {
                MI_SYS_ChnOutputPortPutBuf(handle)
            })?;
        }
        Ok(())
    }
}

impl Drop for Graph {
    fn drop(&mut self) {
        self.stop_pipeline();
        unsafe {
            if self.snr_initialized {
                MI_SNR_DeInitDev();
            }
            if self.sys_initialized {
                MI_SYS_Exit(0);
            }
        }
    }
}

fn apply_sensor_config(graph: &mut Graph, config: &Config) -> Result<(), String> {
    let mut last_error = String::new();
    for attempt in 1..=2 {
        let result = graph.reconfigure(config).and_then(|()| {
            graph
                .capture_tile(config.initial_x, config.initial_y, 0)
                .map(|_| ())
        });
        match result {
            Ok(()) => return Ok(()),
            Err(error) => {
                last_error = error;
                eprintln!(
                    "sensor/VIF apply attempt {attempt}/2 failed for {},{} {}x{} {}x binning: {last_error}",
                    config.initial_x,
                    config.initial_y,
                    config.tile_width * config.sensor_binning * config.sensor_scale,
                    config.tile_height * config.sensor_binning * config.sensor_scale,
                    config.sensor_binning * config.sensor_scale,
                );
            }
        }
    }
    Err(format!(
        "sensor/VIF did not produce a frame after two in-process apply attempts: {last_error}"
    ))
}

fn apply_sensor_transition(
    graph: &mut Graph,
    current: &Config,
    next: &Config,
) -> Result<SensorTransition, String> {
    let transition = sensor_transition(current, next);
    match transition {
        SensorTransition::Unchanged => Ok(transition),
        SensorTransition::LiveOrigin => {
            let started = Instant::now();
            // Commit under group hold, drain already queued buffers, then
            // consume the two possibly stale in-flight exposures without a
            // full-band copy.  The first frame visible to STREAM is therefore
            // the independently validated third post-command exposure.
            let result = graph
                .sensor
                .as_mut()
                .ok_or_else(|| "sensor is not open".to_string())
                .and_then(|sensor| sensor.set_origin_live(next.initial_x, next.initial_y))
                .and_then(|()| {
                    graph
                        .sensor
                        .as_mut()
                        .ok_or_else(|| "sensor is not open".to_string())?
                        .verify_full_raw_mode(next.initial_x, next.initial_y)
                })
                .and_then(|()| {
                    graph.origin_x = next.initial_x;
                    graph.origin_y = next.initial_y;
                    graph.discard_queued_frames(2).map(|_| ())
                })
                .and_then(|()| graph.discard_next_frames(LIVE_ORIGIN_DISCARD_FRAMES));
            match result {
                Ok(()) => {
                    eprintln!(
                        "live IMX582 origin transition settled in {} us; first validated 1x1 frame remains on the data path",
                        started.elapsed().as_micros(),
                    );
                    Ok(transition)
                }
                Err(error) => {
                    eprintln!(
                        "live IMX582 origin move failed ({error}); falling back to one full sensor/VIF rebuild"
                    );
                    apply_sensor_config(graph, next)?;
                    Ok(SensorTransition::Rebuild)
                }
            }
        }
        SensorTransition::LiveGeometry => {
            let started = Instant::now();
            let reconfigure = graph.reconfigure_geometry_live(next);
            let reconfigure_elapsed = started.elapsed();
            let frame_started = Instant::now();
            let result = reconfigure.and_then(|()| graph.acquire_frame().map(|_| ()));
            let frame_elapsed = frame_started.elapsed();
            match result {
                Ok(()) => {
                    eprintln!(
                        "live IMX582/VIF geometry transition produced a validated frame in {} us (reconfigure={} us frame={} us)",
                        started.elapsed().as_micros(),
                        reconfigure_elapsed.as_micros(),
                        frame_elapsed.as_micros(),
                    );
                    Ok(transition)
                }
                Err(error) => {
                    eprintln!(
                        "live IMX582/VIF geometry transition failed ({error}); falling back to a full sensor/VIF rebuild"
                    );
                    apply_sensor_config(graph, next)?;
                    Ok(SensorTransition::Rebuild)
                }
            }
        }
        SensorTransition::Rebuild => {
            apply_sensor_config(graph, next)?;
            Ok(transition)
        }
    }
}

fn main() {
    if let Err(error) = run() {
        eprintln!("pw203_camera_service: {error}");
        std::process::exit(1);
    }
}

fn run() -> Result<(), String> {
    let config = parse_args(env::args().skip(1).collect())?;
    validate_config(&config)?;
    eprintln!(
        "opening standalone IMX582 RAW10 graph: output={}x{} binning={}x{} physical={}x{} origin={},{} listen={} settle={}",
        config.tile_width,
        config.tile_height,
        config.sensor_binning,
        config.sensor_binning,
        config.tile_width * config.sensor_binning * config.sensor_scale,
        config.tile_height * config.sensor_binning * config.sensor_scale,
        config.initial_x,
        config.initial_y,
        config.listen,
        config.settle_frames
    );
    let mut base_config = config.clone();
    let mut graph = Some(Graph::open(&config)?);
    let live_eye_roi = Arc::new(Mutex::new(None::<LiveEyeRoi>));
    let sensor_mode = Arc::new(Mutex::new(SensorMode::from_config(&config)));
    let exposure_timing = Arc::new(AtomicU32::new(ExposureTiming::from_config(&config).pack()));
    let telemetry = Arc::new(Mutex::new(CameraTelemetry::new()));
    let vcm_control = config.vcm_control.clone();
    let control_roi = Arc::clone(&live_eye_roi);
    let control_mode = Arc::clone(&sensor_mode);
    let control_telemetry = Arc::clone(&telemetry);
    let control_exposure_timing = Arc::clone(&exposure_timing);
    thread::Builder::new()
        .name("pw203-vcm-control".to_string())
        .spawn(move || {
            if let Err(error) = serve_vcm_control(
                &vcm_control,
                control_roi,
                control_mode,
                control_telemetry,
                control_exposure_timing,
            ) {
                eprintln!("VCM control stopped: {error}");
            }
        })
        .map_err(|error| format!("spawn VCM control: {error}"))?;
    let listener =
        TcpListener::bind(&config.listen).map_err(|e| format!("bind {}: {e}", config.listen))?;
    eprintln!("standalone raw tile service listening on {}", config.listen);
    for connection in listener.incoming() {
        match connection {
            Ok(stream) => {
                let action = match serve_client(
                    graph.as_mut().ok_or("camera graph unavailable")?,
                    stream,
                    &base_config,
                    Arc::clone(&live_eye_roi),
                    Arc::clone(&sensor_mode),
                    Arc::clone(&telemetry),
                    Arc::clone(&exposure_timing),
                ) {
                    Ok(action) => action,
                    Err(error) => {
                        eprintln!("raw tile client ended: {error}");
                        continue;
                    }
                };
                match action {
                    ClientAction::Continue => {}
                    ClientAction::Shutdown => {
                        eprintln!("graceful custom RAW camera shutdown requested");
                        return Ok(());
                    }
                    ClientAction::SensorMode {
                        mut stream,
                        config: next,
                    } => {
                        let previous = base_config.clone();
                        let transition = apply_sensor_transition(
                            graph.as_mut().ok_or("camera graph unavailable")?,
                            &previous,
                            &next,
                        );
                        match transition {
                            Ok(transition) => {
                                let applied = SensorMode::from_config(&next);
                                base_config = next;
                                if let Ok(mut mode) = sensor_mode.lock() {
                                    *mode = applied;
                                }
                                if let Ok(mut roi) = live_eye_roi.lock() {
                                    *roi = None;
                                }
                                stream.write_all(applied.response().as_bytes()).map_err(
                                    |error| format!("write sensor-mode response: {error}"),
                                )?;
                                eprintln!(
                                    "host sensor crop applied via {transition:?}: physical={},{} {}x{} binning={}x{} output={}x{}",
                                    applied.sensor_x,
                                    applied.sensor_y,
                                    applied.physical_width,
                                    applied.physical_height,
                                    applied.binning,
                                    applied.binning,
                                    applied.output_width,
                                    applied.output_height,
                                );
                            }
                            Err(error) => {
                                let restore = apply_sensor_config(
                                    graph.as_mut().ok_or("camera graph unavailable")?,
                                    &previous,
                                );
                                match restore {
                                    Ok(()) => {
                                        let restored = SensorMode::from_config(&previous);
                                        if let Ok(mut mode) = sensor_mode.lock() {
                                            *mode = restored;
                                        }
                                        let response = format!(
                                            "ERR sensor crop reconfiguration failed: {error}\n"
                                        );
                                        stream.write_all(response.as_bytes()).ok();
                                    }
                                    Err(restore_error) => {
                                        let response = format!(
                                            "ERR sensor crop reconfiguration failed: {error}; previous mode restore failed: {restore_error}\n"
                                        );
                                        stream.write_all(response.as_bytes()).ok();
                                        return Err(format!(
                                            "sensor mode and rollback failed: {error}; {restore_error}"
                                        ));
                                    }
                                }
                            }
                        }
                    }
                    ClientAction::CoarseCapture {
                        mut stream,
                        sequence,
                        width,
                        height,
                        sensor_scaled,
                        format,
                    } => {
                        let live_timing =
                            ExposureTiming::unpack(exposure_timing.load(Ordering::Relaxed));
                        let mut live_fine_config = base_config.clone();
                        live_fine_config.frame_length = live_timing.frame_length;
                        live_fine_config.coarse = live_timing.coarse_lines;
                        match capture_full_sensor_coarse_thumbnail(
                            graph.as_mut().ok_or("camera graph unavailable")?,
                            &live_fine_config,
                            width,
                            height,
                            sensor_scaled,
                            format,
                        ) {
                            Ok((
                                payload,
                                timestamp_ns,
                                stride,
                                switch,
                                capture,
                                reduce,
                                restore,
                            )) => {
                                let header = match format {
                                    CaptureFormat::Gray16 => thumbnail_packet_header(
                                        0,
                                        sequence,
                                        timestamp_ns,
                                        0,
                                        0,
                                        width,
                                        height,
                                        payload.len() as u32,
                                        0,
                                    ),
                                    CaptureFormat::Raw10 => packet_header(
                                        0,
                                        sequence,
                                        timestamp_ns,
                                        0,
                                        0,
                                        width,
                                        height,
                                        stride,
                                        payload.len() as u32,
                                        0,
                                    ),
                                };
                                stream
                                    .write_all(&header)
                                    .and_then(|()| stream.write_all(&payload))
                                    .map_err(|error| {
                                        format!("write coarse sensor thumbnail: {error}")
                                    })?;
                                eprintln!(
                                    "full-sensor 4x4 one-shot seq={sequence} sensor-scale={}x format={} output={}x{} switch={} us capture={} us reduction={} us fine-restore={} us",
                                    if sensor_scaled { 2 } else { 1 },
                                    format.label(),
                                    width,
                                    height,
                                    switch.as_micros(),
                                    capture.as_micros(),
                                    reduce.as_micros(),
                                    restore.as_micros(),
                                );
                            }
                            Err(error) => {
                                eprintln!(
                                    "full-sensor 4x4 one-shot seq={sequence} sensor-scale={}x format={} failed: {error}",
                                    if sensor_scaled { 2 } else { 1 },
                                    format.label(),
                                );
                                let header = match format {
                                    CaptureFormat::Gray16 => {
                                        thumbnail_packet_header(-1, sequence, 0, 0, 0, 0, 0, 0, 0)
                                    }
                                    CaptureFormat::Raw10 => {
                                        packet_header(-1, sequence, 0, 0, 0, 0, 0, 0, 0, 0)
                                    }
                                };
                                stream
                                    .write_all(&header)
                                    .map_err(|write_error| write_error.to_string())?;
                            }
                        }
                    }
                }
            }
            Err(error) => eprintln!("accept failed: {error}"),
        }
    }
    Ok(())
}

fn validate_absolute_eye_rois(
    mode: SensorMode,
    eyes: [(u32, u32); 2],
    width: u32,
    height: u32,
) -> Result<(), String> {
    if mode.binning != 1 {
        return Err("eye ROIs require 1x1 sensor readout".to_string());
    }
    if width == 0 || width & 3 != 0 || height == 0 || height & 1 != 0 {
        return Err("RAW10 eye width must be x/4 aligned and height must be even".to_string());
    }
    let mode_right = mode.sensor_x + mode.physical_width;
    let mode_bottom = mode.sensor_y + mode.physical_height;
    for &(x, y) in &eyes {
        if x & 3 != 0
            || y & 1 != 0
            || x < mode.sensor_x
            || y < mode.sensor_y
            || x.checked_add(width).is_none_or(|right| right > mode_right)
            || y.checked_add(height)
                .is_none_or(|bottom| bottom > mode_bottom)
        {
            return Err(format!(
                "absolute eye ROI {x},{y} {width}x{height} exceeds sensor crop {},{} {}x{}",
                mode.sensor_x, mode.sensor_y, mode.physical_width, mode.physical_height,
            ));
        }
    }
    Ok(())
}

fn serve_vcm_control(
    address: &str,
    live_eye_roi: Arc<Mutex<Option<LiveEyeRoi>>>,
    sensor_mode: Arc<Mutex<SensorMode>>,
    telemetry: Arc<Mutex<CameraTelemetry>>,
    exposure_timing: Arc<AtomicU32>,
) -> Result<(), String> {
    let listener = TcpListener::bind(address)
        .map_err(|error| format!("bind VCM control {address}: {error}"))?;
    let mut vcm = Vcm::open()?;
    let mut sensor = Sensor::open(4, 4, 1, 1)?;
    let mut last_mode = *sensor_mode.lock().map_err(|_| "sensor mode poisoned")?;
    let mut frame_length = if last_mode.binning == 1 {
        FINE_FRAME_LENGTH
    } else {
        COARSE_FRAME_LENGTH
    };
    let mut exposure = if last_mode.binning == 1 {
        FINE_EXPOSURE
    } else {
        1200
    };
    eprintln!("direct DW9738/GT9778 VCM control listening on {address}");
    for connection in listener.incoming() {
        let mut stream = match connection {
            Ok(stream) => stream,
            Err(error) => {
                eprintln!("VCM accept failed: {error}");
                continue;
            }
        };
        stream.set_nodelay(true).ok();
        let reader_stream = match stream.try_clone() {
            Ok(stream) => stream,
            Err(error) => {
                eprintln!("VCM client clone failed: {error}");
                continue;
            }
        };
        let mut reader = BufReader::new(reader_stream);
        loop {
            let mut line = String::new();
            let read = match reader.read_line(&mut line) {
                Ok(read) => read,
                Err(error) => {
                    eprintln!("VCM client read ended: {error}");
                    break;
                }
            };
            if read == 0 {
                break;
            }
            let command_started = Instant::now();
            let current_mode = *sensor_mode.lock().map_err(|_| "sensor mode poisoned")?;
            if current_mode != last_mode {
                last_mode = current_mode;
                frame_length = if last_mode.binning == 1 {
                    FINE_FRAME_LENGTH
                } else {
                    2800
                };
                exposure = if last_mode.binning == 1 {
                    FINE_EXPOSURE
                } else {
                    1200
                };
                exposure_timing.store(
                    ExposureTiming {
                        frame_length,
                        coarse_lines: exposure,
                    }
                    .pack(),
                    Ordering::Relaxed,
                );
            }
            let minimum_frame_length = if current_mode.binning == 1 {
                FINE_FRAME_LENGTH
            } else {
                2800
            };
            let gain = if current_mode.binning == 1 {
                FINE_GAIN
            } else {
                256
            };
            let fields = line.split_whitespace().collect::<Vec<_>>();
            let acquisition_mutation = current_mode.binning != 1
                && match fields.as_slice() {
                    [focus, action, ..]
                        if focus.eq_ignore_ascii_case("FOCUS")
                            && (action.eq_ignore_ascii_case("SET")
                                || action.eq_ignore_ascii_case("STEP")) =>
                    {
                        true
                    }
                    [roi, set, ..]
                        if roi.eq_ignore_ascii_case("ROI") && set.eq_ignore_ascii_case("SET") =>
                    {
                        true
                    }
                    _ => false,
                };
            let response = if acquisition_mutation {
                "ERR focus and eye-ROI changes are disabled in binned acquisition mode\n"
                    .to_string()
            } else {
                match fields.as_slice() {
                [command] if command.eq_ignore_ascii_case("PING") => "OK VCM\n".to_string(),
                [focus, get]
                    if focus.eq_ignore_ascii_case("FOCUS") && get.eq_ignore_ascii_case("GET") =>
                {
                    match vcm.read_position().or_else(|error| vcm.last_position.ok_or(error)) {
                        Ok(position) => format!("FOCUS {position}\n"),
                        Err(error) => format!("ERR {error}\n"),
                    }
                }
                [focus, status]
                    if focus.eq_ignore_ascii_case("FOCUS")
                        && status.eq_ignore_ascii_case("STATUS") =>
                {
                    match vcm.focus_status() {
                        Ok(status) => format!(
                            "FOCUS STATUS TARGET {} READBACK {} SETTLED {} REMAINING_MS {} GENERATION {} SOURCE ESTIMATED\n",
                            status.target,
                            status.readback,
                            u8::from(status.settled),
                            status.remaining_ms,
                            status.generation,
                        ),
                        Err(error) => format!("ERR {error}\n"),
                    }
                }
                [focus, set, value]
                    if focus.eq_ignore_ascii_case("FOCUS") && set.eq_ignore_ascii_case("SET") =>
                {
                    match value.parse::<u16>() {
                        Ok(position) => match vcm.set_position(position) {
                            Ok(readback) => format!("OK FOCUS {position} READBACK {readback}\n"),
                            Err(error) => format!("ERR {error}\n"),
                        },
                        Err(error) => format!("ERR invalid VCM position: {error}\n"),
                    }
                }
                [focus, step, value]
                    if focus.eq_ignore_ascii_case("FOCUS") && step.eq_ignore_ascii_case("STEP") =>
                {
                    match value.parse::<i16>() {
                        Ok(delta) => match vcm.read_position().or_else(|error| vcm.last_position.ok_or(error)) {
                            Ok(current) => {
                                let position = (current as i32 + delta as i32).clamp(0, 1023) as u16;
                                match vcm.set_position(position) {
                                    Ok(readback) => format!("OK FOCUS {position} READBACK {readback}\n"),
                                    Err(error) => format!("ERR {error}\n"),
                                }
                            }
                            Err(error) => format!("ERR {error}\n"),
                        },
                        Err(error) => format!("ERR invalid VCM step: {error}\n"),
                    }
                }
                [exposure_command, get]
                    if exposure_command.eq_ignore_ascii_case("EXPOSURE")
                        && get.eq_ignore_ascii_case("GET") =>
                {
                    format!("EXPOSURE {exposure} FRAME_LENGTH {frame_length}\n")
                }
                [exposure_command, set, value]
                    if exposure_command.eq_ignore_ascii_case("EXPOSURE")
                        && set.eq_ignore_ascii_case("SET") =>
                {
                    match value.parse::<u16>() {
                        Ok(requested) => {
                            let next = requested.clamp(1, u16::MAX - 1);
                            let next_frame_length = minimum_frame_length.max(next.saturating_add(1));
                            match sensor.set_exposure(next_frame_length, next, gain) {
                                Ok(()) => {
                                    exposure = next;
                                    frame_length = next_frame_length;
                                    exposure_timing.store(
                                        ExposureTiming {
                                            frame_length,
                                            coarse_lines: exposure,
                                        }
                                        .pack(),
                                        Ordering::Relaxed,
                                    );
                                    format!("OK EXPOSURE {exposure} FRAME_LENGTH {frame_length}\n")
                                }
                                Err(error) => format!("ERR {error}\n"),
                            }
                        }
                        Err(error) => format!("ERR invalid exposure: {error}\n"),
                    }
                }
                [exposure_command, step, value]
                    if exposure_command.eq_ignore_ascii_case("EXPOSURE")
                        && step.eq_ignore_ascii_case("STEP") =>
                {
                    match value.parse::<i32>() {
                        Ok(delta) => {
                            let next = (exposure as i64 + delta as i64)
                                .clamp(1, (u16::MAX - 1) as i64) as u16;
                            let next_frame_length = minimum_frame_length.max(next.saturating_add(1));
                            match sensor.set_exposure(next_frame_length, next, gain) {
                                Ok(()) => {
                                    exposure = next;
                                    frame_length = next_frame_length;
                                    exposure_timing.store(
                                        ExposureTiming {
                                            frame_length,
                                            coarse_lines: exposure,
                                        }
                                        .pack(),
                                        Ordering::Relaxed,
                                    );
                                    format!("OK EXPOSURE {exposure} FRAME_LENGTH {frame_length}\n")
                                }
                                Err(error) => format!("ERR {error}\n"),
                            }
                        }
                        Err(error) => format!("ERR invalid exposure step: {error}\n"),
                    }
                }
                [roi, set, left_x, left_y, right_x, right_y, eye_width, eye_height]
                    if roi.eq_ignore_ascii_case("ROI") && set.eq_ignore_ascii_case("SET") =>
                {
                    let parsed = [left_x, left_y, right_x, right_y, eye_width, eye_height]
                        .iter()
                        .map(|value| value.parse::<u32>())
                        .collect::<Result<Vec<_>, _>>();
                    match parsed {
                        Ok(values) => {
                            let eyes = [(values[0], values[1]), (values[2], values[3])];
                            if validate_absolute_eye_rois(
                                current_mode,
                                eyes,
                                values[4],
                                values[5],
                            )
                            .is_ok()
                            {
                                let mut state = live_eye_roi.lock().map_err(|_| "ROI state poisoned")?;
                                let generation = state.map(|current| current.generation + 1).unwrap_or(1);
                                *state = Some(LiveEyeRoi {
                                    eyes,
                                    eye_width: values[4],
                                    eye_height: values[5],
                                    generation,
                                });
                                format!("OK ROI {generation}\n")
                            } else {
                                "ERR ROI geometry exceeds physical sensor window\n".to_string()
                            }
                        }
                        Err(error) => format!("ERR invalid ROI coordinate: {error}\n"),
                    }
                }
                    _ => "ERR commands: PING | FOCUS GET|STATUS|SET|STEP | EXPOSURE GET|SET|STEP | ROI SET left_abs_x left_abs_y right_abs_x right_abs_y eye_width eye_height\n".to_string(),
                }
            };
            if fields
                .first()
                .is_some_and(|field| field.eq_ignore_ascii_case("FOCUS"))
            {
                record_camera_timing(
                    &telemetry,
                    CAMERA_STAGE_VCM_COMMAND,
                    command_started.elapsed(),
                );
                if fields
                    .get(1)
                    .is_some_and(|field| field.eq_ignore_ascii_case("STATUS"))
                {
                    let response_fields = response.split_whitespace().collect::<Vec<_>>();
                    if let Some(remaining_ms) = response_fields
                        .windows(2)
                        .find(|pair| pair[0] == "REMAINING_MS")
                        .and_then(|pair| pair[1].parse::<u64>().ok())
                    {
                        record_camera_sample_us(
                            &telemetry,
                            CAMERA_STAGE_VCM_REMAINING,
                            remaining_ms.saturating_mul(1_000).min(u32::MAX as u64) as u32,
                        );
                    }
                }
            }
            if let Err(error) = stream.write_all(response.as_bytes()) {
                eprintln!("VCM client write ended: {error}");
            }
            // Every host helper already opens a fresh control connection for
            // one command. Close after its response so an idle diagnostic or
            // abandoned client cannot monopolize the serial VCM/ROI owner and
            // block the live tracker from restoring sensor geometry.
            break;
        }
    }
    Ok(())
}

fn serve_client(
    graph: &mut Graph,
    mut stream: TcpStream,
    config: &Config,
    live_eye_roi: Arc<Mutex<Option<LiveEyeRoi>>>,
    sensor_mode: Arc<Mutex<SensorMode>>,
    telemetry: Arc<Mutex<CameraTelemetry>>,
    exposure_timing: Arc<AtomicU32>,
) -> Result<ClientAction, String> {
    let peer = stream.peer_addr().ok();
    stream
        .set_nodelay(true)
        .map_err(|e| format!("set TCP_NODELAY: {e}"))?;
    let reader_stream = stream
        .try_clone()
        .map_err(|e| format!("clone client stream: {e}"))?;
    let mut reader = BufReader::new(reader_stream);
    eprintln!("raw tile client connected: {peer:?}");
    loop {
        let mut line = String::new();
        let read = reader
            .read_line(&mut line)
            .map_err(|e| format!("read command: {e}"))?;
        if read == 0 {
            break;
        }
        let fields: Vec<&str> = line.split_whitespace().collect();
        if fields.is_empty() {
            continue;
        }
        if fields[0].eq_ignore_ascii_case("QUIT") {
            break;
        }
        if fields[0].eq_ignore_ascii_case("VERSION") {
            stream
                .write_all(format!("OK CUSTOM_CAMERA {PROTOCOL_VERSION}\n").as_bytes())
                .map_err(|error| error.to_string())?;
            continue;
        }
        if fields.len() == 2
            && fields[0].eq_ignore_ascii_case("SENSOR")
            && fields[1].eq_ignore_ascii_case("GET")
        {
            let mode = *sensor_mode.lock().map_err(|_| "sensor mode poisoned")?;
            stream
                .write_all(mode.response().as_bytes())
                .map_err(|error| error.to_string())?;
            continue;
        }
        if fields.len() == 2
            && fields[0].eq_ignore_ascii_case("SENSOR")
            && fields[1].eq_ignore_ascii_case("PREVIEW")
        {
            let next = scaled_acquisition_config(config)?;
            return Ok(ClientAction::SensorMode {
                stream,
                config: next,
            });
        }
        if fields.len() == 7
            && fields[0].eq_ignore_ascii_case("SENSOR")
            && fields[1].eq_ignore_ascii_case("SET")
        {
            let sensor_x = parse::<u32>(fields[2], "sensor crop x")?;
            let sensor_y = parse::<u32>(fields[3], "sensor crop y")?;
            let physical_width = parse::<u32>(fields[4], "sensor crop width")?;
            let physical_height = parse::<u32>(fields[5], "sensor crop height")?;
            let binning = parse::<u32>(fields[6], "sensor crop binning")?;
            let next = sensor_crop_config(
                config,
                sensor_x,
                sensor_y,
                physical_width,
                physical_height,
                binning,
            )?;
            return Ok(ClientAction::SensorMode {
                stream,
                config: next,
            });
        }
        if fields[0].eq_ignore_ascii_case("PROBE") && fields.len() == 1 {
            let origin_x = graph.origin_x;
            let origin_y = graph.origin_y;
            let response = match graph.capture_tile(origin_x, origin_y, 1) {
                Ok(frame) => format!("OK FRAME {}\n", frame.timestamp_ns),
                Err(error) => format!("ERR FRAME {error}\n"),
            };
            stream
                .write_all(response.as_bytes())
                .map_err(|error| error.to_string())?;
            continue;
        }
        if fields[0].eq_ignore_ascii_case("SHUTDOWN") {
            stream.write_all(b"OK SHUTDOWN\n").ok();
            return Ok(ClientAction::Shutdown);
        }
        if fields[0].eq_ignore_ascii_case("CAPTURE_SCAN") && fields.len() == 4 {
            let sequence = parse::<u64>(fields[1], "sequence")?;
            let output_width = parse::<u32>(fields[2], "scan thumbnail width")?;
            let output_height = parse::<u32>(fields[3], "scan thumbnail height")?;
            match capture_full_sensor_scan_thumbnail(graph, config, output_width, output_height) {
                Ok((payload, timestamp_ns, bands, scan_elapsed, restore_elapsed)) => {
                    let header = thumbnail_packet_header(
                        0,
                        sequence,
                        timestamp_ns,
                        0,
                        0,
                        output_width,
                        output_height,
                        payload.len() as u32,
                        bands as u32,
                    );
                    stream
                        .write_all(&header)
                        .and_then(|()| stream.write_all(&payload))
                        .map_err(|error| format!("write full-sensor scan thumbnail: {error}"))?;
                    eprintln!(
                        "full-sensor 1x1 scan seq={sequence} bands={bands} output={}x{} capture={} us fine-restore={} us",
                        output_width,
                        output_height,
                        scan_elapsed.as_micros(),
                        restore_elapsed.as_micros(),
                    );
                }
                Err(error) => {
                    eprintln!("full-sensor 1x1 scan seq={sequence} failed: {error}");
                    let header = thumbnail_packet_header(-1, sequence, 0, 0, 0, 0, 0, 0, 0);
                    stream
                        .write_all(&header)
                        .map_err(|write_error| write_error.to_string())?;
                }
            }
            continue;
        }
        if fields[0].eq_ignore_ascii_case("CAPTURE_COARSE") && matches!(fields.len(), 4 | 5) {
            let sequence = parse::<u64>(fields[1], "sequence")?;
            let output_width = parse::<u32>(fields[2], "coarse thumbnail width")?;
            let output_height = parse::<u32>(fields[3], "coarse thumbnail height")?;
            let format = CaptureFormat::parse(fields.get(4).copied())?;
            return Ok(ClientAction::CoarseCapture {
                stream,
                sequence,
                width: output_width,
                height: output_height,
                sensor_scaled: false,
                format,
            });
        }
        if fields[0].eq_ignore_ascii_case("CAPTURE_GLOBAL") && matches!(fields.len(), 4 | 5) {
            let sequence = parse::<u64>(fields[1], "sequence")?;
            let output_width = parse::<u32>(fields[2], "global thumbnail width")?;
            let output_height = parse::<u32>(fields[3], "global thumbnail height")?;
            let format = CaptureFormat::parse(fields.get(4).copied())?;
            return Ok(ClientAction::CoarseCapture {
                stream,
                sequence,
                width: output_width,
                height: output_height,
                sensor_scaled: true,
                format,
            });
        }
        if fields[0].eq_ignore_ascii_case("CAPTURE_THUMB") && fields.len() == 7 {
            let sequence = parse::<u64>(fields[1], "sequence")?;
            let x = parse::<u32>(fields[2], "thumbnail x")?;
            let y = parse::<u32>(fields[3], "thumbnail y")?;
            let settle = parse::<u32>(fields[4], "thumbnail settle")?;
            let output_width = parse::<u32>(fields[5], "thumbnail width")?;
            let output_height = parse::<u32>(fields[6], "thumbnail height")?;
            let capture = if x == graph.origin_x && y == graph.origin_y {
                graph.capture_tile(x, y, settle)
            } else {
                Err(format!(
                    "thumbnail origin {x},{y} differs from applied host sensor crop {},{}; use SENSOR SET",
                    graph.origin_x, graph.origin_y
                ))
            };
            match capture {
                Ok(frame) => {
                    let payload = downsample_raw10_gray(
                        &frame,
                        graph.width,
                        graph.height,
                        output_width,
                        output_height,
                    )?;
                    let header = thumbnail_packet_header(
                        0,
                        sequence,
                        frame.timestamp_ns,
                        x,
                        y,
                        output_width,
                        output_height,
                        payload.len() as u32,
                        settle,
                    );
                    stream
                        .write_all(&header)
                        .and_then(|()| stream.write_all(&payload))
                        .map_err(|error| format!("write tracking thumbnail: {error}"))?;
                }
                Err(error) => {
                    eprintln!("tracking thumbnail seq={sequence} roi={x},{y} failed: {error}");
                    let header = thumbnail_packet_header(-1, sequence, 0, x, y, 0, 0, 0, settle);
                    stream
                        .write_all(&header)
                        .map_err(|error| error.to_string())?;
                }
            }
            continue;
        }
        if fields[0].eq_ignore_ascii_case("CAPTURE_CURRENT") && (2..=3).contains(&fields.len()) {
            let sequence = parse::<u64>(fields[1], "sequence")?;
            let settle = if fields.len() == 3 {
                parse::<u32>(fields[2], "settle")?
            } else {
                config.settle_frames
            };
            match graph.capture_tile(graph.origin_x, graph.origin_y, settle) {
                Ok(frame) => {
                    let header = packet_header(
                        0,
                        sequence,
                        frame.timestamp_ns,
                        graph.origin_x,
                        graph.origin_y,
                        graph.width,
                        graph.height,
                        frame.stride,
                        frame.bytes.len() as u32,
                        settle,
                    );
                    stream
                        .write_all(&header)
                        .and_then(|()| stream.write_all(&frame.bytes))
                        .map_err(|error| format!("write current raw tile: {error}"))?;
                }
                Err(error) => {
                    eprintln!("capture current seq={sequence} failed: {error}");
                    let header = packet_header(
                        -1,
                        sequence,
                        0,
                        graph.origin_x,
                        graph.origin_y,
                        graph.width,
                        graph.height,
                        0,
                        0,
                        settle,
                    );
                    stream
                        .write_all(&header)
                        .map_err(|error| error.to_string())?;
                }
            }
            continue;
        }
        if fields.len() == 2
            && fields[0].eq_ignore_ascii_case("MODE")
            && fields[1].eq_ignore_ascii_case("TRACKING")
        {
            stream
                .write_all(
                    format!(
                        "OK MODE TRACKING {} {} {} {} {} {} {}\n",
                        config.initial_x,
                        config.initial_y,
                        graph.width,
                        graph.height,
                        config.sensor_binning,
                        graph.width * config.sensor_binning * config.sensor_scale,
                        graph.height * config.sensor_binning * config.sensor_scale,
                    )
                    .as_bytes(),
                )
                .map_err(|error| error.to_string())?;
            continue;
        }
        if fields.len() == 4
            && fields[0].eq_ignore_ascii_case("MODE")
            && fields[1].eq_ignore_ascii_case("EYES")
        {
            stream
                .write_all(b"ERR MODE EYES was replaced by host-driven SENSOR SET\n")
                .map_err(|error| error.to_string())?;
            continue;
        }
        if fields[0].eq_ignore_ascii_case("STREAM_EYES") && matches!(fields.len(), 8 | 9 | 12) {
            if config.sensor_binning != 1 {
                stream
                    .write_all(b"ERR eye streaming requires 1x1 sensor readout\n")
                    .map_err(|error| error.to_string())?;
                continue;
            }
            let sequence = parse::<u64>(fields[1], "sequence")?;
            let left_x = parse::<u32>(fields[2], "absolute left x")?;
            let left_y = parse::<u32>(fields[3], "absolute left y")?;
            let right_x = parse::<u32>(fields[4], "absolute right x")?;
            let right_y = parse::<u32>(fields[5], "absolute right y")?;
            let eye_width = parse::<u32>(fields[6], "eye width")?;
            let eye_height = parse::<u32>(fields[7], "eye height")?;
            let frames = if fields.len() >= 9 {
                parse::<u32>(fields[8], "frames")?
            } else {
                0
            };
            let context = if fields.len() == 12 {
                let every = parse::<u32>(fields[9], "context interval")?;
                let width = parse::<u32>(fields[10], "context width")?;
                let height = parse::<u32>(fields[11], "context height")?;
                parse_context_stream(every, width, height, graph.width, graph.height)?
            } else {
                None
            };
            let eyes = [(left_x, left_y), (right_x, right_y)];
            validate_absolute_eye_rois(
                SensorMode::from_config(config),
                eyes,
                eye_width,
                eye_height,
            )?;
            {
                let mut state = live_eye_roi.lock().map_err(|_| "ROI state poisoned")?;
                let generation = state.map(|current| current.generation + 1).unwrap_or(1);
                *state = Some(LiveEyeRoi {
                    eyes,
                    eye_width,
                    eye_height,
                    generation,
                });
            }
            stream_eye_rois(
                graph,
                &mut stream,
                config,
                sequence,
                eyes,
                eye_width,
                eye_height,
                frames,
                context,
                live_eye_roi,
                telemetry,
                exposure_timing,
            )?;
            break;
        }
        if !fields[0].eq_ignore_ascii_case("CAPTURE") || !(4..=5).contains(&fields.len()) {
            eprintln!("ignored command: {}", line.trim());
            continue;
        }
        let sequence = parse::<u64>(fields[1], "sequence")?;
        let x = parse::<u32>(fields[2], "x")?;
        let y = parse::<u32>(fields[3], "y")?;
        let settle = if fields.len() == 5 {
            parse::<u32>(fields[4], "settle")?
        } else {
            config.settle_frames
        };

        let capture = if x == graph.origin_x && y == graph.origin_y {
            graph.capture_tile(x, y, settle)
        } else {
            Err(format!(
                "capture origin {x},{y} differs from applied host sensor crop {},{}; use SENSOR SET",
                graph.origin_x, graph.origin_y,
            ))
        };
        match capture {
            Ok(frame) => {
                let header = packet_header(
                    0,
                    sequence,
                    frame.timestamp_ns,
                    x,
                    y,
                    graph.width,
                    graph.height,
                    frame.stride,
                    frame.bytes.len() as u32,
                    settle,
                );
                stream
                    .write_all(&header)
                    .and_then(|()| stream.write_all(&frame.bytes))
                    .map_err(|e| format!("write raw tile: {e}"))?;
            }
            Err(error) => {
                eprintln!("capture seq={sequence} roi={x},{y} failed: {error}");
                let header = packet_header(
                    -1,
                    sequence,
                    0,
                    x,
                    y,
                    graph.width,
                    graph.height,
                    0,
                    0,
                    settle,
                );
                stream
                    .write_all(&header)
                    .map_err(|e| format!("write error response: {e}"))?;
            }
        }
    }
    eprintln!("raw tile client disconnected: {peer:?}");
    Ok(ClientAction::Continue)
}

#[allow(clippy::too_many_arguments)]
fn stream_eye_rois(
    graph: &mut Graph,
    stream: &mut TcpStream,
    config: &Config,
    mut sequence: u64,
    eyes: [(u32, u32); 2],
    eye_width: u32,
    eye_height: u32,
    frames: u32,
    context: Option<ContextStream>,
    live_eye_roi: Arc<Mutex<Option<LiveEyeRoi>>>,
    telemetry: Arc<Mutex<CameraTelemetry>>,
    exposure_timing: Arc<AtomicU32>,
) -> Result<(), String> {
    let mode = SensorMode::from_config(config);
    validate_absolute_eye_rois(mode, eyes, eye_width, eye_height)?;
    graph
        .sensor
        .as_mut()
        .ok_or_else(|| "sensor is not open".to_string())?
        .verify_full_raw_mode(graph.origin_x, graph.origin_y)?;
    let started = Instant::now();
    let mut sent = 0u32;
    let mut last_frame_ready = None;
    let mut completed_sets = VecDeque::with_capacity(MIN_RAW_ROI_SETS_PER_SECOND + 1);
    let mut active = LiveEyeRoi {
        eyes,
        eye_width,
        eye_height,
        generation: live_eye_roi
            .lock()
            .ok()
            .and_then(|state| *state)
            .map(|state| state.generation)
            .unwrap_or(0),
    };
    if let Ok(mut telemetry) = telemetry.lock() {
        telemetry.reset(Instant::now());
    }
    let context_description = context
        .map(|stream| {
            format!(
                "{}x{} every {} frames",
                stream.width, stream.height, stream.every
            )
        })
        .unwrap_or_else(|| "disabled".to_string());
    eprintln!(
        "streaming paired RAW10 eyes sensor-crop={},{} {}x{} eye={}x{} absolute-left={},{} absolute-right={},{} frames={} context={}",
        graph.origin_x,
        graph.origin_y,
        graph.width,
        graph.height,
        eye_width,
        eye_height,
        eyes[0].0,
        eyes[0].1,
        eyes[1].0,
        eyes[1].1,
        frames,
        context_description,
    );
    while frames == 0 || sent < frames {
        let requested = live_eye_roi
            .lock()
            .map_err(|_| "ROI state poisoned")?
            .as_ref()
            .copied();
        if let Some(requested) =
            requested.filter(|requested| requested.generation != active.generation)
        {
            validate_absolute_eye_rois(
                mode,
                requested.eyes,
                requested.eye_width,
                requested.eye_height,
            )?;
            eprintln!(
                "tracking update generation={} absolute-left={},{} absolute-right={},{}",
                requested.generation,
                requested.eyes[0].0,
                requested.eyes[0].1,
                requested.eyes[1].0,
                requested.eyes[1].1,
            );
            active = requested;
        }
        let active_context = context.filter(|context| sent % context.every == 0);
        let frame = graph.acquire_eye_set(
            active.eyes,
            active.eye_width,
            active.eye_height,
            active_context,
        )?;
        let frame_ready = Instant::now();
        record_camera_timing(&telemetry, CAMERA_STAGE_SENSOR_ACQUIRE, frame.sensor_wait);
        record_camera_timing(&telemetry, CAMERA_STAGE_ROI_SLICE, frame.roi_slice);
        if active_context.is_some() {
            record_camera_timing(&telemetry, CAMERA_STAGE_CONTEXT_BUILD, frame.context_build);
        }
        if let Some(previous_ready) = last_frame_ready {
            record_camera_timing(
                &telemetry,
                CAMERA_STAGE_SENSOR_INTERVAL,
                frame_ready.duration_since(previous_ready),
            );
        }
        last_frame_ready = Some(frame_ready);
        let timestamp_ns = frame.timestamp_ns;
        if let (Some(context), Some(payload)) = (active_context, frame.context.as_ref()) {
            let header = thumbnail_packet_header(
                0,
                sequence,
                timestamp_ns,
                graph.origin_x,
                graph.origin_y,
                context.width,
                context.height,
                payload.len() as u32,
                0,
            );
            let write_started = Instant::now();
            stream
                .write_all(&header)
                .and_then(|()| stream.write_all(&payload))
                .map_err(|error| format!("write RAW10 context thumbnail: {error}"))?;
            record_camera_timing(
                &telemetry,
                CAMERA_STAGE_STREAM_WRITE,
                write_started.elapsed(),
            );
        }
        for (index, (&(absolute_x, absolute_y), payload)) in
            active.eyes.iter().zip(frame.payloads.iter()).enumerate()
        {
            let eye_id = index as u32 + 1;
            let header = eye_packet_header(
                eye_id,
                sequence,
                timestamp_ns,
                absolute_x,
                absolute_y,
                active.eye_width,
                active.eye_height,
                active.eye_width / 4 * 5,
                payload.len() as u32,
            );
            let write_started = Instant::now();
            stream
                .write_all(&header)
                .and_then(|()| stream.write_all(&payload))
                .map_err(|error| format!("write RAW10 eye {eye_id}: {error}"))?;
            record_camera_timing(
                &telemetry,
                CAMERA_STAGE_STREAM_WRITE,
                write_started.elapsed(),
            );
        }
        sent = sent.wrapping_add(1);
        sequence = sequence.wrapping_add(1);
        let completed = Instant::now();
        completed_sets.push_back(completed);
        let frame_length = usize::from(
            ExposureTiming::unpack(exposure_timing.load(Ordering::Relaxed))
                .frame_length
                .max(FINE_FRAME_LENGTH),
        );
        let required_sets = (MIN_RAW_ROI_SETS_PER_SECOND * usize::from(FINE_FRAME_LENGTH))
            .div_ceil(frame_length)
            .clamp(
                MIN_LONG_EXPOSURE_RAW_ROI_SETS_PER_SECOND,
                MIN_RAW_ROI_SETS_PER_SECOND,
            );
        let cadence_window = required_sets + 1;
        while completed_sets.len() > cadence_window {
            completed_sets.pop_front();
        }
        if completed_sets.len() == cadence_window {
            let span = completed.duration_since(*completed_sets.front().unwrap());
            if span > Duration::from_secs(1) {
                return Err(format!(
                    "FATAL RAW ROI cadence: {} shared-exposure intervals took {:.3}s ({:.2} sets/s), below the exposure-dependent requirement of {} sets/s at frame length {}",
                    required_sets,
                    span.as_secs_f64(),
                    required_sets as f64 / span.as_secs_f64(),
                    required_sets,
                    frame_length,
                ));
            }
        }
        if sent % TELEMETRY_EVERY_RAW_SETS == 0 {
            let telemetry_now = Instant::now();
            let payload = telemetry
                .lock()
                .map_err(|_| "camera telemetry state poisoned")?
                .payload(telemetry_now);
            let header = telemetry_packet_header(
                sequence.wrapping_sub(1),
                timestamp_ns,
                payload.len() as u32,
            );
            let write_started = Instant::now();
            stream
                .write_all(&header)
                .and_then(|()| stream.write_all(&payload))
                .map_err(|error| format!("write bounded timing telemetry: {error}"))?;
            record_camera_timing(
                &telemetry,
                CAMERA_STAGE_STREAM_WRITE,
                write_started.elapsed(),
            );
        }
        if sent <= 3 || sent % 30 == 0 {
            let fps = sent as f64 / started.elapsed().as_secs_f64().max(0.001);
            eprintln!("paired RAW10 eye frames={sent} rate={fps:.2}fps");
        }
    }
    Ok(())
}

fn crop_raw10(
    frame: &RawFrame,
    x: u32,
    y: u32,
    width: u32,
    height: u32,
) -> Result<Vec<u8>, String> {
    crop_raw10_bytes(&frame.bytes, frame.stride as usize, x, y, width, height)
}

fn crop_raw10_bytes(
    source_bytes: &[u8],
    source_stride: usize,
    x: u32,
    y: u32,
    width: u32,
    height: u32,
) -> Result<Vec<u8>, String> {
    let byte_x = (x / 4 * 5) as usize;
    let output_stride = (width / 4 * 5) as usize;
    let mut output = vec![0u8; output_stride * height as usize];
    for row in 0..height as usize {
        let source = (y as usize + row) * source_stride + byte_x;
        let destination = row * output_stride;
        let source_end = source + output_stride;
        if source_end > source_bytes.len() {
            return Err("RAW10 eye crop exceeds captured payload".to_string());
        }
        output[destination..destination + output_stride]
            .copy_from_slice(&source_bytes[source..source_end]);
    }
    Ok(output)
}

fn packed_raw10_pixel(frame: &RawFrame, x: usize, y: usize) -> u16 {
    packed_raw10_pixel_bytes(&frame.bytes, frame.stride as usize, x, y)
}

fn packed_raw10_pixel_bytes(bytes: &[u8], stride: usize, x: usize, y: usize) -> u16 {
    let row = &bytes[y * stride..(y + 1) * stride];
    let offset = x / 4 * 5;
    let word = (row[offset] as u64)
        | ((row[offset + 1] as u64) << 8)
        | ((row[offset + 2] as u64) << 16)
        | ((row[offset + 3] as u64) << 24)
        | ((row[offset + 4] as u64) << 32);
    ((word >> ((x & 3) * 10)) & 0x3ff) as u16
}

fn capture_full_sensor_coarse_thumbnail(
    graph: &mut Graph,
    fine_config: &Config,
    output_width: u32,
    output_height: u32,
    sensor_scaled: bool,
    format: CaptureFormat,
) -> Result<(Vec<u8>, u64, u32, Duration, Duration, Duration, Duration), String> {
    if fine_config.sensor_binning != 1
        || !fine_config.direct_full_raw
        || fine_config.initial_x != 0
        || fine_config.tile_width != SENSOR_WIDTH
    {
        return Err(format!(
            "one-shot coarse acquisition requires a full-width physical 1x1 fine band, got origin={},{} output={}x{} binning={}x",
            fine_config.initial_x,
            fine_config.initial_y,
            fine_config.tile_width,
            fine_config.tile_height,
            fine_config.sensor_binning,
        ));
    }
    let mut coarse = if sensor_scaled {
        full_sensor_scaled_config(fine_config)?
    } else {
        sensor_crop_config(fine_config, 0, 0, SENSOR_WIDTH, SENSOR_HEIGHT, 4)?
    };
    let fine_timing = ExposureTiming::from_config(fine_config);
    let coarse_timing = matching_coarse_exposure(fine_timing);
    coarse.frame_length = coarse_timing.frame_length;
    coarse.coarse = coarse_timing.coarse_lines;
    coarse.gain = fine_config.gain;
    // The warm provider's FPS value is framework timing metadata. Keep it in
    // step with the inherited fine exposure instead of advertising the old
    // short-exposure 15 fps cadence while the sensor is producing ~10 fps.
    coarse.sensor_fps = fine_config.sensor_fps;
    validate_config(&coarse)?;
    let (maximum_width, maximum_height) = match (format, sensor_scaled) {
        (CaptureFormat::Raw10, _) | (CaptureFormat::Gray16, true) => {
            (coarse.tile_width, coarse.tile_height)
        }
        (CaptureFormat::Gray16, false) => (coarse.tile_width / 2, coarse.tile_height / 2),
    };
    if output_width == 0
        || output_height == 0
        || output_width > maximum_width
        || output_height > maximum_height
    {
        return Err(format!(
            "coarse {} thumbnail {}x{} must be at most {}x{}",
            format.label(),
            output_width,
            output_height,
            maximum_width,
            maximum_height,
        ));
    }
    if format == CaptureFormat::Raw10 && (output_width & 3 != 0 || output_height & 1 != 0) {
        return Err(format!(
            "packed RAW10 thumbnail {}x{} requires width divisible by 4 and even height",
            output_width, output_height,
        ));
    }

    let switch_started = Instant::now();
    eprintln!(
        "matching global integration: fine={} lines x {} clocks, global={} lines x {} clocks, gain={}",
        fine_timing.coarse_lines,
        FINE_LINE_LENGTH,
        coarse_timing.coarse_lines,
        COARSE_LINE_LENGTH,
        coarse.gain,
    );
    // A changed SigmaStar VIF line geometry is accepted on the first resident
    // retune, but its output port remains working-inactive for that first
    // coarse exposure. Let the measured 124-137 ms exposure finish, then
    // re-arm the same already-programmed geometry. This replaces the previous
    // 1.5-second timeout without discarding a usable frame.
    let rearm_dwell_ms = std::env::var("PW203_COARSE_REARM_DWELL_MS")
        .ok()
        .and_then(|value| value.parse::<u64>().ok())
        .filter(|value| (120..=300).contains(value))
        .unwrap_or(COARSE_REARM_DWELL_MS);
    let coarse_switch = graph
        .reconfigure_geometry_live(&coarse)
        .and_then(|()| {
            if sensor_scaled {
                graph
                    .sensor
                    .as_mut()
                    .ok_or_else(|| "sensor is not open".to_string())?
                    .verify_scaled_global_mode()?;
            }
            Ok(())
        })
        .and_then(|()| {
            thread::sleep(Duration::from_millis(rearm_dwell_ms));
            graph.rearm_vif_output_live()
        });
    if let Err(error) = coarse_switch {
        let restore = apply_sensor_config(graph, fine_config);
        return match restore {
            Ok(()) => Err(format!("two-phase resident coarse retune failed: {error}")),
            Err(restore_error) => Err(format!(
                "two-phase resident coarse retune failed: {error}; fine rollback failed: {restore_error}"
            )),
        };
    }
    let switch_elapsed = switch_started.elapsed();

    let capture_started = Instant::now();
    let frame = match graph.acquire_frame_with_timeout(COARSE_FRAME_WAIT) {
        Ok(frame) => Ok(frame),
        Err(first_error) => {
            // The sensor continues producing frame starts during the observed
            // SigmaStar miss, but the newly sized output port occasionally
            // remains working-inactive. Re-arm the same resident geometry once
            // and keep both waits inside a bounded acquisition budget.
            eprintln!(
                "coarse VIF output missed its {} ms frame budget ({first_error}); re-arming the same resident geometry once",
                COARSE_FRAME_WAIT.as_millis(),
            );
            graph
                .rearm_vif_output_live()
                .map_err(|error| {
                    format!("coarse output re-arm failed after {first_error}: {error}")
                })
                .and_then(|()| {
                    graph
                        .acquire_frame_with_timeout(COARSE_FRAME_WAIT)
                        .map_err(|error| {
                            format!(
                                "coarse output missed both bounded frame attempts: first={first_error}; second={error}"
                            )
                        })
                })
        }
    };
    let capture_elapsed = capture_started.elapsed();

    // acquire_frame owns a complete copy and has already returned the VIF
    // buffer. Restore fine geometry before the roughly 68 ms Bayer reduction
    // so fine 1x1 exposures resume as early as possible.
    let restore_started = Instant::now();
    let resident_restore = graph.reconfigure_geometry_live(fine_config).and_then(|()| {
        graph
            .sensor
            .as_mut()
            .ok_or_else(|| "sensor is not open".to_string())?
            .verify_full_raw_mode(fine_config.initial_x, fine_config.initial_y)
    });
    let restore = match resident_restore {
        Ok(()) => Ok(()),
        Err(error) => {
            eprintln!(
                "resident fine restore after coarse one-shot failed ({error}); using full custom-graph recovery"
            );
            apply_sensor_config(graph, fine_config)
        }
    };
    let restore_elapsed = restore_started.elapsed();

    let reduce_started = Instant::now();
    let reduced = frame.and_then(|frame| {
        let timestamp_ns = frame.timestamp_ns;
        let reduction = match (format, sensor_scaled) {
            (CaptureFormat::Raw10, _) => resample_raw10_bayer(
                &frame,
                coarse.tile_width,
                coarse.tile_height,
                output_width,
                output_height,
            ),
            (CaptureFormat::Gray16, true) => downsample_raw10_gray_strobe(
                &frame,
                coarse.tile_width,
                coarse.tile_height,
                output_width,
                output_height,
            ),
            (CaptureFormat::Gray16, false) => downsample_raw10_gray(
                &frame,
                coarse.tile_width,
                coarse.tile_height,
                output_width,
                output_height,
            ),
        };
        reduction.map(|payload| (payload, timestamp_ns))
    });
    let reduce_elapsed = reduce_started.elapsed();

    match (reduced, restore) {
        (Ok((payload, timestamp_ns)), Ok(())) => Ok((
            payload,
            timestamp_ns,
            match format {
                CaptureFormat::Gray16 => output_width * 2,
                CaptureFormat::Raw10 => output_width / 4 * 5,
            },
            switch_elapsed,
            capture_elapsed,
            reduce_elapsed,
            restore_elapsed,
        )),
        (Err(capture_error), Ok(())) => Err(capture_error),
        (Ok(_), Err(restore_error)) => Err(format!(
            "coarse thumbnail completed but fine restore failed: {restore_error}"
        )),
        (Err(capture_error), Err(restore_error)) => Err(format!(
            "coarse thumbnail failed: {capture_error}; fine restore failed: {restore_error}"
        )),
    }
}

fn capture_full_sensor_scan_thumbnail(
    graph: &mut Graph,
    config: &Config,
    output_width: u32,
    output_height: u32,
) -> Result<(Vec<u8>, u64, usize, Duration, Duration), String> {
    if config.sensor_binning != 1
        || graph.width != SENSOR_WIDTH
        || graph.height == 0
        || graph.height > SENSOR_HEIGHT
        || graph.origin_x != 0
    {
        return Err(format!(
            "full-sensor scan requires a full-width 1x1 RAW band, got origin={},{} output={}x{} binning={}x",
            graph.origin_x, graph.origin_y, graph.width, graph.height, config.sensor_binning,
        ));
    }
    if output_width == 0
        || output_height == 0
        || SENSOR_WIDTH % output_width != 0
        || SENSOR_HEIGHT % output_height != 0
    {
        return Err(format!(
            "scan thumbnail {output_width}x{output_height} must evenly divide {}x{}",
            SENSOR_WIDTH, SENSOR_HEIGHT,
        ));
    }
    let scale_x = SENSOR_WIDTH / output_width;
    let scale_y = SENSOR_HEIGHT / output_height;
    if scale_x < 2 || scale_y < 2 || scale_x & 1 != 0 || scale_y & 1 != 0 {
        return Err(format!(
            "scan thumbnail requires even Bayer-aware scale >=2, got {scale_x}x{scale_y}"
        ));
    }

    let original_x = graph.origin_x;
    let original_y = graph.origin_y;
    let mut origins = Vec::new();
    let maximum_y = SENSOR_HEIGHT - graph.height;
    let mut y = 0u32;
    loop {
        origins.push(y);
        if y == maximum_y {
            break;
        }
        y = y.saturating_add(graph.height).min(maximum_y);
    }

    let scan_frame_length = graph
        .height
        .saturating_add(SCAN_MIN_BLANKING_LINES)
        .min(u16::MAX as u32) as u16;
    let scan_exposure = scan_frame_length.saturating_sub(SCAN_EXPOSURE_MARGIN_LINES);
    graph
        .sensor
        .as_mut()
        .ok_or_else(|| "sensor is not open".to_string())?
        .set_exposure(scan_frame_length, scan_exposure, SCAN_GAIN)?;
    let scan_started = Instant::now();
    let scan_result = (|| {
        let mut payload = vec![0u8; (output_width * output_height * 2) as usize];
        let mut filled_rows = vec![false; output_height as usize];
        let mut timestamp_ns = 0u64;
        for (index, &origin_y) in origins.iter().enumerate() {
            // A group-held crop is visible only on a later frame boundary.
            // Consume two complete post-command exposures for every requested
            // origin before accepting the third frame as that band.  The
            // old pipelined version armed the next origin while reducing the
            // current frame, then sometimes labeled a queued old-origin frame
            // as the new band; MediaPipe coordinates were consequently
            // displaced even though the stitched thumbnail looked coherent.
            let frame = graph.capture_tile(0, origin_y, SCAN_ORIGIN_SETTLE_FRAMES)?;
            timestamp_ns = frame.timestamp_ns;
            let unique_end = origins.get(index + 1).copied().unwrap_or(SENSOR_HEIGHT);
            for output_y in 0..output_height as usize {
                let source_y0 = output_y * scale_y as usize;
                let sample_global_y = source_y0 + ((scale_y as usize - 2) / 2 & !1);
                if sample_global_y < origin_y as usize || sample_global_y + 1 >= unique_end as usize
                {
                    continue;
                }
                let sample_y = sample_global_y - origin_y as usize;
                for output_x in 0..output_width as usize {
                    let source_x0 = output_x * scale_x as usize;
                    let sample_x = source_x0 + ((scale_x as usize - 2) / 2 & !1);
                    let sum = packed_raw10_pixel(&frame, sample_x, sample_y) as u32
                        + packed_raw10_pixel(&frame, sample_x + 1, sample_y) as u32
                        + packed_raw10_pixel(&frame, sample_x, sample_y + 1) as u32
                        + packed_raw10_pixel(&frame, sample_x + 1, sample_y + 1) as u32;
                    put_u16(
                        &mut payload,
                        (output_y * output_width as usize + output_x) * 2,
                        (sum / 4) as u16,
                    );
                }
                filled_rows[output_y] = true;
            }
        }
        if let Some(missing) = filled_rows.iter().position(|filled| !*filled) {
            return Err(format!("sensor scan did not fill thumbnail row {missing}"));
        }
        Ok((payload, timestamp_ns))
    })();
    let scan_elapsed = scan_started.elapsed();

    let restore_started = Instant::now();
    let restore_registers = graph
        .sensor
        .as_mut()
        .ok_or_else(|| "sensor is not open".to_string())
        .and_then(|sensor| sensor.set_origin_live(original_x, original_y))
        .and_then(|()| {
            graph
                .sensor
                .as_mut()
                .ok_or_else(|| "sensor is not open".to_string())?
                .set_exposure(config.frame_length, config.coarse, config.gain)
        });
    graph.origin_x = original_x;
    graph.origin_y = original_y;
    let restore_result = restore_registers
        .and_then(|()| {
            // Remove already-queued scan frames, then consume the two
            // possibly in-flight scan-origin exposures without copying them.
            // MediaPipe and the reconnecting fine stream must never receive a
            // bottom scan band labeled as the restored eye band.
            graph.discard_queued_frames(2).map(|_| ())
        })
        .and_then(|()| graph.discard_next_frames(LIVE_ORIGIN_DISCARD_FRAMES))
        .and_then(|()| {
            graph
                .sensor
                .as_mut()
                .ok_or_else(|| "sensor is not open".to_string())?
                .verify_full_raw_mode(original_x, original_y)
        });
    let restore_elapsed = restore_started.elapsed();

    match (scan_result, restore_result) {
        (Ok((payload, timestamp_ns)), Ok(())) => Ok((
            payload,
            timestamp_ns,
            origins.len(),
            scan_elapsed,
            restore_elapsed,
        )),
        (Err(scan_error), Ok(())) => Err(scan_error),
        (Ok(_), Err(restore_error)) => Err(format!(
            "sensor scan completed but fine timing restore failed: {restore_error}"
        )),
        (Err(scan_error), Err(restore_error)) => Err(format!(
            "sensor scan failed: {scan_error}; fine timing restore failed: {restore_error}"
        )),
    }
}

fn downsample_raw10_gray(
    frame: &RawFrame,
    width: u32,
    height: u32,
    output_width: u32,
    output_height: u32,
) -> Result<Vec<u8>, String> {
    if output_width == 0 || output_height == 0 || output_width > width || output_height > height {
        return Err(format!(
            "thumbnail {}x{} must fit inside RAW tile {}x{}",
            output_width, output_height, width, height
        ));
    }
    let bayer_strobe =
        width >= output_width.saturating_mul(2) && height >= output_height.saturating_mul(2);
    let mut output = Vec::with_capacity((output_width * output_height * 2) as usize);
    for output_y in 0..output_height as usize {
        let source_y0 = output_y * height as usize / output_height as usize;
        let source_y1 = (output_y + 1) * height as usize / output_height as usize;
        for output_x in 0..output_width as usize {
            let source_x0 = output_x * width as usize / output_width as usize;
            let source_x1 = (output_x + 1) * width as usize / output_width as usize;
            let average = if bayer_strobe {
                // Strobe one phase-aligned 2x2 cell near the center of every
                // proportional source box. It contains every Bayer phase and
                // costs exactly four RAW10 decodes even for a non-integral
                // 2000x1500 -> 800x600 coarse thumbnail. MediaPipe does not
                // benefit from averaging every source pixel in each box.
                let sample_x = ((source_x0 + source_x1) / 2)
                    .saturating_sub(1)
                    .min(width as usize - 2)
                    & !1;
                let sample_y = ((source_y0 + source_y1) / 2)
                    .saturating_sub(1)
                    .min(height as usize - 2)
                    & !1;
                let sum = packed_raw10_pixel(frame, sample_x, sample_y) as u32
                    + packed_raw10_pixel(frame, sample_x + 1, sample_y) as u32
                    + packed_raw10_pixel(frame, sample_x, sample_y + 1) as u32
                    + packed_raw10_pixel(frame, sample_x + 1, sample_y + 1) as u32;
                (sum / 4) as u16
            } else {
                let mut sum = 0u32;
                for y in source_y0..source_y1 {
                    for x in source_x0..source_x1 {
                        sum += packed_raw10_pixel(frame, x, y) as u32;
                    }
                }
                let samples = (source_x1 - source_x0) * (source_y1 - source_y0);
                (sum / samples as u32) as u16
            };
            output.extend_from_slice(&average.to_le_bytes());
        }
    }
    Ok(output)
}

fn downsample_raw10_gray_strobe(
    frame: &RawFrame,
    width: u32,
    height: u32,
    output_width: u32,
    output_height: u32,
) -> Result<Vec<u8>, String> {
    if width < 2
        || height < 2
        || output_width == 0
        || output_height == 0
        || output_width > width
        || output_height > height
    {
        return Err(format!(
            "phase-safe thumbnail {}x{} must fit inside RAW tile {}x{}",
            output_width, output_height, width, height
        ));
    }
    let mut output = Vec::with_capacity((output_width * output_height * 2) as usize);
    for output_y in 0..output_height as usize {
        let sample_y =
            ((output_y * height as usize / output_height as usize).min(height as usize - 2)) & !1;
        for output_x in 0..output_width as usize {
            let sample_x =
                ((output_x * width as usize / output_width as usize).min(width as usize - 2)) & !1;
            let sum = packed_raw10_pixel(frame, sample_x, sample_y) as u32
                + packed_raw10_pixel(frame, sample_x + 1, sample_y) as u32
                + packed_raw10_pixel(frame, sample_x, sample_y + 1) as u32
                + packed_raw10_pixel(frame, sample_x + 1, sample_y + 1) as u32;
            output.extend_from_slice(&((sum / 4) as u16).to_le_bytes());
        }
    }
    Ok(output)
}

fn push_raw10_group(output: &mut Vec<u8>, pixels: [u16; 4]) {
    let packed = u64::from(pixels[0] & 0x03ff)
        | (u64::from(pixels[1] & 0x03ff) << 10)
        | (u64::from(pixels[2] & 0x03ff) << 20)
        | (u64::from(pixels[3] & 0x03ff) << 30);
    output.extend_from_slice(&packed.to_le_bytes()[..5]);
}

/// Resize a packed Bayer frame without mixing color phases. Every output 2x2
/// cell samples one proportional source 2x2 cell, so RGGB phase remains valid
/// for host-side demosaic. Native-size requests preserve every RAW10 sample.
fn resample_raw10_bayer(
    frame: &RawFrame,
    width: u32,
    height: u32,
    output_width: u32,
    output_height: u32,
) -> Result<Vec<u8>, String> {
    if width == 0
        || height == 0
        || width & 1 != 0
        || height & 1 != 0
        || output_width == 0
        || output_height == 0
        || output_width & 3 != 0
        || output_height & 1 != 0
        || output_width > width
        || output_height > height
    {
        return Err(format!(
            "phase-preserving RAW10 resize {}x{} -> {}x{} requires even input, output width divisible by 4, even output height, and no enlargement",
            width, height, output_width, output_height,
        ));
    }
    if frame.stride as usize * height as usize > frame.bytes.len() {
        return Err("captured RAW10 payload is shorter than its declared geometry".to_string());
    }
    if output_width == width && output_height == height {
        return crop_raw10_bytes(&frame.bytes, frame.stride as usize, 0, 0, width, height);
    }

    let source_cells_x = width as usize / 2;
    let source_cells_y = height as usize / 2;
    let output_cells_x = output_width as usize / 2;
    let output_cells_y = output_height as usize / 2;
    let mut output = Vec::with_capacity((output_width / 4 * 5 * output_height) as usize);
    for output_y in 0..output_height as usize {
        let output_cell_y = output_y / 2;
        let source_cell_y = ((2 * output_cell_y + 1) * source_cells_y / (2 * output_cells_y))
            .min(source_cells_y - 1);
        let source_y = source_cell_y * 2 + output_y % 2;
        for output_group_x in (0..output_width as usize).step_by(4) {
            let mut pixels = [0u16; 4];
            for (offset, pixel) in pixels.iter_mut().enumerate() {
                let output_x = output_group_x + offset;
                let output_cell_x = output_x / 2;
                let source_cell_x = ((2 * output_cell_x + 1) * source_cells_x
                    / (2 * output_cells_x))
                    .min(source_cells_x - 1);
                let source_x = source_cell_x * 2 + output_x % 2;
                *pixel = packed_raw10_pixel(frame, source_x, source_y);
            }
            push_raw10_group(&mut output, pixels);
        }
    }
    Ok(output)
}

fn downsample_raw10_tracking_context(
    frame: &RawFrame,
    width: u32,
    height: u32,
    output_width: u32,
    output_height: u32,
) -> Result<Vec<u8>, String> {
    downsample_raw10_tracking_context_bytes(
        &frame.bytes,
        frame.stride as usize,
        width,
        height,
        output_width,
        output_height,
    )
}

fn downsample_raw10_tracking_context_bytes(
    source: &[u8],
    source_stride: usize,
    width: u32,
    height: u32,
    output_width: u32,
    output_height: u32,
) -> Result<Vec<u8>, String> {
    if output_width == 0
        || output_height == 0
        || width % output_width != 0
        || height % output_height != 0
    {
        return Err(format!(
            "tracking context {}x{} must evenly divide RAW tile {}x{}",
            output_width, output_height, width, height,
        ));
    }
    let scale_x = width / output_width;
    let scale_y = height / output_height;
    if scale_x < 2 || scale_y < 2 {
        return Err(format!(
            "tracking context scale {scale_x}x{scale_y} is too small for fixed Bayer-phase sampling",
        ));
    }

    // Use one centered 2x2 Bayer cell per output pixel. This preserves every
    // color phase as stable monochrome texture for fine tracking and makes the
    // same already-streaming context suitable for a global MediaPipe patch.
    let mut output = Vec::with_capacity((output_width * output_height * 2) as usize);
    for output_y in 0..output_height as usize {
        let source_y = (output_y * scale_y as usize + (scale_y as usize - 2) / 2) & !1;
        for output_x in 0..output_width as usize {
            let source_x = (output_x * scale_x as usize + (scale_x as usize - 2) / 2) & !1;
            let sum = packed_raw10_pixel_bytes(source, source_stride, source_x, source_y) as u32
                + packed_raw10_pixel_bytes(source, source_stride, source_x + 1, source_y) as u32
                + packed_raw10_pixel_bytes(source, source_stride, source_x, source_y + 1) as u32
                + packed_raw10_pixel_bytes(source, source_stride, source_x + 1, source_y + 1)
                    as u32;
            output.extend_from_slice(&((sum / 4) as u16).to_le_bytes());
        }
    }
    Ok(output)
}

fn parse_context_stream(
    every: u32,
    width: u32,
    height: u32,
    source_width: u32,
    source_height: u32,
) -> Result<Option<ContextStream>, String> {
    if every == 0 {
        if width != 0 || height != 0 {
            return Err("disabled context stream requires 0x0 dimensions".to_string());
        }
        return Ok(None);
    }
    if width == 0 || height == 0 || source_width % width != 0 || source_height % height != 0 {
        return Err(format!(
            "context thumbnail {width}x{height} must evenly divide RAW tile {source_width}x{source_height}"
        ));
    }
    Ok(Some(ContextStream {
        every,
        width,
        height,
    }))
}

#[allow(clippy::too_many_arguments)]
fn thumbnail_packet_header(
    status: i32,
    sequence: u64,
    timestamp_ns: u64,
    sensor_x: u32,
    sensor_y: u32,
    width: u32,
    height: u32,
    payload_bytes: u32,
    settle_frames: u32,
) -> [u8; PACKET_HEADER_BYTES] {
    let mut header = [0u8; PACKET_HEADER_BYTES];
    header[0..4].copy_from_slice(b"OTH1");
    put_u16(&mut header, 4, 1);
    put_u16(&mut header, 6, PACKET_HEADER_BYTES as u16);
    put_u32(&mut header, 8, status as u32);
    put_u32(&mut header, 12, PACKET_FORMAT_GRAY16);
    put_u64(&mut header, 16, sequence);
    put_u64(&mut header, 24, timestamp_ns);
    put_u32(&mut header, 32, sensor_x);
    put_u32(&mut header, 36, sensor_y);
    put_u32(&mut header, 40, width);
    put_u32(&mut header, 44, height);
    put_u32(&mut header, 48, width * 2);
    put_u32(&mut header, 52, payload_bytes);
    put_u32(&mut header, 56, 16);
    put_u32(&mut header, 60, settle_frames);
    header
}

#[allow(clippy::too_many_arguments)]
fn eye_packet_header(
    eye_id: u32,
    sequence: u64,
    timestamp_ns: u64,
    sensor_x: u32,
    sensor_y: u32,
    width: u32,
    height: u32,
    stride: u32,
    payload_bytes: u32,
) -> [u8; PACKET_HEADER_BYTES] {
    let mut header = [0u8; PACKET_HEADER_BYTES];
    header[0..4].copy_from_slice(b"ORE1");
    put_u16(&mut header, 4, 1);
    put_u16(&mut header, 6, PACKET_HEADER_BYTES as u16);
    put_u32(&mut header, 8, eye_id);
    put_u32(&mut header, 12, PACKET_FORMAT_RAW10_LE40);
    put_u64(&mut header, 16, sequence);
    put_u64(&mut header, 24, timestamp_ns);
    put_u32(&mut header, 32, sensor_x);
    put_u32(&mut header, 36, sensor_y);
    put_u32(&mut header, 40, width);
    put_u32(&mut header, 44, height);
    put_u32(&mut header, 48, stride);
    put_u32(&mut header, 52, payload_bytes);
    put_u32(&mut header, 56, RAW10_RG);
    put_u32(&mut header, 60, 1);
    header
}

fn telemetry_packet_header(
    sequence: u64,
    timestamp_ns: u64,
    payload_bytes: u32,
) -> [u8; PACKET_HEADER_BYTES] {
    let mut header = [0u8; PACKET_HEADER_BYTES];
    header[0..4].copy_from_slice(b"OTM1");
    put_u16(&mut header, 4, 1);
    put_u16(&mut header, 6, PACKET_HEADER_BYTES as u16);
    put_u32(&mut header, 8, 0);
    put_u32(&mut header, 12, PACKET_FORMAT_TELEMETRY);
    put_u64(&mut header, 16, sequence);
    put_u64(&mut header, 24, timestamp_ns);
    put_u32(&mut header, 32, 0);
    put_u32(&mut header, 36, 0);
    put_u32(&mut header, 40, payload_bytes);
    put_u32(&mut header, 44, 1);
    put_u32(&mut header, 48, payload_bytes);
    put_u32(&mut header, 52, payload_bytes);
    put_u32(&mut header, 56, 8);
    put_u32(&mut header, 60, 1);
    header
}

#[allow(clippy::too_many_arguments)]
fn packet_header(
    status: i32,
    sequence: u64,
    timestamp_ns: u64,
    x: u32,
    y: u32,
    width: u32,
    height: u32,
    stride: u32,
    payload_bytes: u32,
    settle_frames: u32,
) -> [u8; PACKET_HEADER_BYTES] {
    let mut header = [0u8; PACKET_HEADER_BYTES];
    header[0..4].copy_from_slice(b"ORT1");
    put_u16(&mut header, 4, 1);
    put_u16(&mut header, 6, PACKET_HEADER_BYTES as u16);
    put_u32(&mut header, 8, status as u32);
    put_u32(&mut header, 12, PACKET_FORMAT_RAW10_LE40);
    put_u64(&mut header, 16, sequence);
    put_u64(&mut header, 24, timestamp_ns);
    put_u32(&mut header, 32, x);
    put_u32(&mut header, 36, y);
    put_u32(&mut header, 40, width);
    put_u32(&mut header, 44, height);
    put_u32(&mut header, 48, stride);
    put_u32(&mut header, 52, payload_bytes);
    put_u32(&mut header, 56, RAW10_RG);
    put_u32(&mut header, 60, settle_frames);
    header
}

fn validate_config(config: &Config) -> Result<(), String> {
    if config.tile_width == 0
        || config.tile_height == 0
        || config.tile_width & 3 != 0
        || config.tile_height & 1 != 0
        || config.tile_width > u16::MAX as u32
        || config.tile_height > u16::MAX as u32
    {
        return Err(
            "tile width must be a positive multiple of four and height must be positive and even"
                .to_string(),
        );
    }
    if config.frame_length < 16 {
        return Err("frame length must be at least 16".to_string());
    }
    if !matches!(config.sensor_binning, 1 | 4) {
        return Err("sensor binning must be 1 or 4".to_string());
    }
    if !matches!(config.sensor_scale, 1 | 2)
        || (config.sensor_binning == 1 && config.sensor_scale != 1)
    {
        return Err("sensor scale must be 1, or 2 with 4x4 binning".to_string());
    }
    let probe = Sensor {
        file: OpenOptions::new()
            .read(true)
            .open("/dev/null")
            .map_err(|e| format!("open /dev/null: {e}"))?,
        width: config.tile_width,
        height: config.tile_height,
        binning: config.sensor_binning,
        scale: config.sensor_scale,
    };
    probe.validate_origin(config.initial_x, config.initial_y)
}

fn parse_args(args: Vec<String>) -> Result<Config, String> {
    let mut config = Config::default();
    let mut index = 0usize;
    while index < args.len() {
        match args[index].as_str() {
            "--listen" => {
                index += 1;
                config.listen = value(&args, index, "--listen")?.to_string();
            }
            "--vcm-control" => {
                index += 1;
                config.vcm_control = value(&args, index, "--vcm-control")?.to_string();
            }
            "--tile" => {
                index += 1;
                (config.tile_width, config.tile_height) =
                    parse_size(value(&args, index, "--tile")?)?;
            }
            "--origin" => {
                index += 1;
                (config.initial_x, config.initial_y) =
                    parse_pair(value(&args, index, "--origin")?)?;
            }
            "--settle" => {
                index += 1;
                config.settle_frames = parse(value(&args, index, "--settle")?, "settle")?;
            }
            "--timeout-ms" => {
                index += 1;
                let milliseconds = parse(value(&args, index, "--timeout-ms")?, "timeout")?;
                config.frame_timeout = Duration::from_millis(milliseconds);
            }
            "--frame-length" => {
                index += 1;
                config.frame_length =
                    parse(value(&args, index, "--frame-length")?, "frame length")?;
            }
            "--coarse" => {
                index += 1;
                config.coarse = parse(value(&args, index, "--coarse")?, "coarse exposure")?;
            }
            "--gain" => {
                index += 1;
                config.gain = parse(value(&args, index, "--gain")?, "gain")?;
            }
            "--sensor-res" => {
                index += 1;
                config.sensor_resolution =
                    parse(value(&args, index, "--sensor-res")?, "sensor resolution")?;
            }
            "--sensor-fps" => {
                index += 1;
                config.sensor_fps = parse(value(&args, index, "--sensor-fps")?, "sensor fps")?;
            }
            "--sensor-binning" => {
                index += 1;
                config.sensor_binning =
                    parse(value(&args, index, "--sensor-binning")?, "sensor binning")?;
            }
            "--native-mode" => config.direct_full_raw = false,
            "-h" | "--help" => {
                println!(
                    "usage: pw203_camera_service [--listen ADDR] [--vcm-control ADDR] [--tile WxH] [--origin X,Y] \
                     [--settle N] [--timeout-ms N] [--frame-length N] [--coarse N] [--gain N] \
                     [--sensor-res N] [--sensor-fps N] [--sensor-binning 1|4] [--native-mode]\n\
                     image protocol: SENSOR GET|PREVIEW|SET X Y PHYSICAL_W PHYSICAL_H BINNING, CAPTURE_CURRENT, \
                     CAPTURE_THUMB, CAPTURE_SCAN, CAPTURE_COARSE/CAPTURE_GLOBAL sequence WIDTH HEIGHT [GRAY16|RAW10], or STREAM_EYES sequence LEFT_ABS_X LEFT_ABS_Y RIGHT_ABS_X RIGHT_ABS_Y \
                     EYE_W EYE_H [frames [context_every context_width context_height]]; \
                     VCM protocol: FOCUS GET|STATUS|SET|STEP | EXPOSURE GET|SET|STEP | \
                     ROI SET LEFT_ABS_X LEFT_ABS_Y RIGHT_ABS_X RIGHT_ABS_Y EYE_W EYE_H"
                );
                std::process::exit(0);
            }
            unknown => return Err(format!("unknown argument {unknown}")),
        }
        index += 1;
    }
    Ok(config)
}

fn value<'a>(args: &'a [String], index: usize, name: &str) -> Result<&'a str, String> {
    args.get(index)
        .map(String::as_str)
        .ok_or_else(|| format!("{name} needs a value"))
}

fn parse<T: std::str::FromStr>(text: &str, name: &str) -> Result<T, String>
where
    T::Err: std::fmt::Display,
{
    text.parse::<T>()
        .map_err(|e| format!("invalid {name} {text}: {e}"))
}

fn parse_size(text: &str) -> Result<(u32, u32), String> {
    let mut fields = text.split(['x', 'X']);
    let width = parse(fields.next().unwrap_or(""), "tile width")?;
    let height = parse(fields.next().unwrap_or(""), "tile height")?;
    if fields.next().is_some() {
        return Err(format!("invalid size {text}"));
    }
    Ok((width, height))
}

fn parse_pair(text: &str) -> Result<(u32, u32), String> {
    let mut fields = text.split(',');
    let x = parse(fields.next().unwrap_or(""), "origin x")?;
    let y = parse(fields.next().unwrap_or(""), "origin y")?;
    if fields.next().is_some() {
        return Err(format!("invalid origin {text}"));
    }
    Ok((x, y))
}

fn mi(name: &str, result: i32) -> Result<(), String> {
    if result == 0 {
        Ok(())
    } else {
        Err(format!("{name} failed: {result}"))
    }
}

fn read_u16(bytes: &[u8], offset: usize) -> u16 {
    u16::from_le_bytes([bytes[offset], bytes[offset + 1]])
}

fn read_u32(bytes: &[u8], offset: usize) -> u32 {
    u32::from_le_bytes([
        bytes[offset],
        bytes[offset + 1],
        bytes[offset + 2],
        bytes[offset + 3],
    ])
}

fn put_u16(bytes: &mut [u8], offset: usize, value: u16) {
    bytes[offset..offset + 2].copy_from_slice(&value.to_le_bytes());
}

fn put_u32(bytes: &mut [u8], offset: usize, value: u32) {
    bytes[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
}

fn put_u64(bytes: &mut [u8], offset: usize, value: u64) {
    bytes[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn timing_histogram_reports_bucketed_quantiles_max_and_jitter() {
        let mut histogram = TimingHistogram::default();
        for sample in [10, 20, 30, 40, 90, 1_100] {
            histogram.record_us(sample);
        }
        assert_eq!(histogram.count, 6);
        assert_eq!(histogram.percentile(1, 4), 32);
        assert_eq!(histogram.percentile(1, 2), 32);
        assert_eq!(histogram.percentile(3, 4), 128);
        assert_eq!(histogram.percentile(95, 100), 1_100);
        assert_eq!(histogram.max_us, 1_100);
        assert_eq!(histogram.max_jitter_us, 1_010);
    }

    #[test]
    fn timing_histogram_is_fixed_size_and_saturates_counts() {
        let mut histogram = TimingHistogram::default();
        histogram.buckets[0] = u16::MAX;
        histogram.count = u32::MAX;
        histogram.record_us(1);
        assert_eq!(histogram.buckets.len(), TELEMETRY_BUCKETS);
        assert_eq!(histogram.buckets[0], u16::MAX);
        assert_eq!(histogram.count, u32::MAX);
    }

    #[test]
    fn telemetry_payload_and_header_are_compact_and_self_describing() {
        let mut telemetry = CameraTelemetry::new();
        telemetry.record_us(CAMERA_STAGE_SENSOR_ACQUIRE, 83_000);
        telemetry.record_us(CAMERA_STAGE_SENSOR_ACQUIRE, 84_000);
        telemetry.record_us(CAMERA_STAGE_ROI_SLICE, 250);
        let payload = telemetry.payload(Instant::now());
        assert_eq!(read_u16(&payload, 0), TELEMETRY_VERSION);
        assert_eq!(read_u16(&payload, 2), 2);
        assert_eq!(
            payload.len(),
            TELEMETRY_PREFIX_BYTES + 2 * TELEMETRY_RECORD_BYTES
        );
        assert_eq!(payload[TELEMETRY_PREFIX_BYTES], CAMERA_STAGE_SENSOR_ACQUIRE);
        assert_eq!(payload[TELEMETRY_PREFIX_BYTES + 1], TELEMETRY_BUCKETS as u8);
        assert_eq!(
            payload[TELEMETRY_PREFIX_BYTES + TELEMETRY_RECORD_BYTES],
            CAMERA_STAGE_ROI_SLICE
        );

        let header = telemetry_packet_header(17, 99, payload.len() as u32);
        assert_eq!(&header[0..4], b"OTM1");
        assert_eq!(read_u32(&header, 12), PACKET_FORMAT_TELEMETRY);
        assert_eq!(read_u32(&header, 40), payload.len() as u32);
        assert_eq!(read_u32(&header, 52), payload.len() as u32);
        assert!(payload.len() < 1_024);
    }

    #[test]
    fn expired_camera_window_is_published_before_it_is_reset() {
        let now = Instant::now();
        let mut telemetry = CameraTelemetry::new();
        telemetry.window_started = now - Duration::from_secs(2);
        telemetry.record_us(CAMERA_STAGE_SENSOR_INTERVAL, 80_000);
        let expired = telemetry.payload(now);
        assert_eq!(read_u16(&expired, 2), 1);
        assert_eq!(
            expired[TELEMETRY_PREFIX_BYTES],
            CAMERA_STAGE_SENSOR_INTERVAL
        );
        let fresh = telemetry.payload(now);
        assert_eq!(read_u16(&fresh, 2), 0);
    }

    #[test]
    fn required_cadence_telemetry_stays_below_three_kib_per_second() {
        let mut telemetry = CameraTelemetry::new();
        for stage_id in CAMERA_STAGE_IDS {
            telemetry.record_us(stage_id, 1_000);
        }
        let payload = telemetry.payload(Instant::now());
        assert_eq!(
            payload.len(),
            TELEMETRY_PREFIX_BYTES + 7 * TELEMETRY_RECORD_BYTES
        );
        let telemetry_packets_per_second =
            MIN_RAW_ROI_SETS_PER_SECOND / TELEMETRY_EVERY_RAW_SETS as usize;
        let wire_bytes_per_second =
            (PACKET_HEADER_BYTES + payload.len()) * telemetry_packets_per_second;
        assert_eq!(wire_bytes_per_second, 2_640);
        assert!(wire_bytes_per_second < 3 * 1_024);
    }

    #[test]
    fn context_stream_accepts_exact_divisor() {
        assert_eq!(
            parse_context_stream(3, 320, 96, 8000, 384).unwrap(),
            Some(ContextStream {
                every: 3,
                width: 320,
                height: 96,
            })
        );
    }

    #[test]
    fn context_stream_disables_only_with_zero_geometry() {
        assert_eq!(parse_context_stream(0, 0, 0, 8000, 384).unwrap(), None);
        assert!(parse_context_stream(0, 320, 96, 8000, 384).is_err());
    }

    #[test]
    fn context_stream_rejects_non_divisor() {
        assert!(parse_context_stream(3, 321, 96, 8000, 384).is_err());
        assert!(parse_context_stream(3, 320, 100, 8000, 384).is_err());
    }

    #[test]
    fn four_by_four_output_can_cover_the_complete_sensor() {
        let mut config = Config::default();
        config.tile_width = 2000;
        config.tile_height = 1500;
        config.initial_x = 0;
        config.initial_y = 0;
        config.sensor_binning = 4;
        assert!(validate_config(&config).is_ok());
        config.initial_x = 4;
        assert!(validate_config(&config).is_err());
    }

    #[test]
    fn sensor_binning_argument_is_explicit() {
        let config = parse_args(vec!["--sensor-binning".into(), "4".into()]).unwrap();
        assert_eq!(config.sensor_binning, 4);
    }

    #[test]
    fn host_sensor_modes_cover_coarse_and_fine_geometry() {
        let base = Config::default();
        let coarse = sensor_crop_config(&base, 0, 0, 8000, 6000, 4).unwrap();
        assert_eq!(SensorMode::from_config(&coarse).output_width, 2000);
        assert_eq!(SensorMode::from_config(&coarse).output_height, 1500);
        let fine = sensor_crop_config(&coarse, 0, 2808, 8000, 384, 1).unwrap();
        assert_eq!(SensorMode::from_config(&fine).physical_width, 8000);
        assert_eq!(SensorMode::from_config(&fine).physical_height, 384);
    }

    #[test]
    fn scaled_global_mode_preserves_full_field_at_quarter_payload() {
        let fine = sensor_crop_config(&Config::default(), 0, 2712, 8000, 576, 1).unwrap();
        let global = full_sensor_scaled_config(&fine).unwrap();
        let mode = SensorMode::from_config(&global);
        assert_eq!((mode.physical_width, mode.physical_height), (8000, 6000));
        assert_eq!((mode.output_width, mode.output_height), (1000, 750));
        assert_eq!(global.sensor_binning, 4);
        assert_eq!(global.sensor_scale, 2);
        assert_eq!(
            sensor_transition(&fine, &global),
            SensorTransition::LiveGeometry
        );
    }

    #[test]
    fn global_capture_preserves_physical_integration_time_across_line_lengths() {
        for fine_lines in [FINE_EXPOSURE, 2800, 8000] {
            let coarse = matching_coarse_exposure(ExposureTiming {
                frame_length: fine_lines + 1,
                coarse_lines: fine_lines,
            });
            let fine_clocks = u64::from(fine_lines) * u64::from(FINE_LINE_LENGTH);
            let coarse_clocks = u64::from(coarse.coarse_lines) * u64::from(COARSE_LINE_LENGTH);
            assert!(fine_clocks.abs_diff(coarse_clocks) <= u64::from(COARSE_LINE_LENGTH) / 2);
            assert!(coarse.frame_length >= coarse.coarse_lines + COARSE_EXPOSURE_MARGIN_LINES);
        }
        assert_eq!(
            matching_coarse_exposure(ExposureTiming {
                frame_length: 2801,
                coarse_lines: 2800,
            }),
            ExposureTiming {
                frame_length: 14188,
                coarse_lines: 14138,
            },
        );
    }

    #[test]
    fn direct_raw_geometry_changes_keep_the_sigma_star_graph_resident() {
        let base = Config::default();
        let fine = sensor_crop_config(&base, 0, 2808, 8000, 768, 1).unwrap();
        let moved = sensor_crop_config(&fine, 0, 2912, 8000, 768, 1).unwrap();
        let resized = sensor_crop_config(&fine, 0, 2808, 8000, 864, 1).unwrap();
        let coarse = sensor_crop_config(&fine, 0, 0, 8000, 6000, 4).unwrap();
        let preview = scaled_acquisition_config(&fine).unwrap();
        assert_eq!(sensor_transition(&fine, &fine), SensorTransition::Unchanged);
        assert_eq!(
            sensor_transition(&fine, &moved),
            SensorTransition::LiveOrigin
        );
        assert_eq!(
            sensor_transition(&fine, &resized),
            SensorTransition::LiveGeometry
        );
        assert_eq!(
            sensor_transition(&fine, &coarse),
            SensorTransition::LiveGeometry
        );
        assert_eq!(
            sensor_transition(&fine, &preview),
            SensorTransition::Rebuild
        );
    }

    #[test]
    fn absolute_eye_rois_may_overlap_inside_the_sensor_band() {
        let mode = SensorMode {
            sensor_x: 0,
            sensor_y: 2000,
            physical_width: 8000,
            physical_height: 384,
            binning: 1,
            output_width: 8000,
            output_height: 384,
        };
        assert!(validate_absolute_eye_rois(mode, [(1200, 2052), (1440, 2078)], 384, 256,).is_ok());
        assert!(validate_absolute_eye_rois(mode, [(1200, 1952), (1440, 2078)], 384, 256,).is_err());
    }

    #[test]
    fn coarse_thumbnail_supports_noninteger_box_scaling() {
        let frame = RawFrame {
            bytes: vec![0; 8 * 10],
            stride: 10,
            timestamp_ns: 0,
        };
        assert_eq!(downsample_raw10_gray(&frame, 8, 8, 3, 3).unwrap().len(), 18);
        assert_eq!(
            downsample_raw10_gray_strobe(&frame, 8, 8, 7, 7)
                .unwrap()
                .len(),
            98,
        );
    }

    #[test]
    fn capture_format_is_explicit_and_gray16_remains_the_default() {
        assert_eq!(CaptureFormat::parse(None).unwrap(), CaptureFormat::Gray16);
        assert_eq!(
            CaptureFormat::parse(Some("gray16")).unwrap(),
            CaptureFormat::Gray16,
        );
        assert_eq!(
            CaptureFormat::parse(Some("RAW10")).unwrap(),
            CaptureFormat::Raw10,
        );
        assert!(CaptureFormat::parse(Some("JPEG")).is_err());
    }

    #[test]
    fn raw10_thumbnail_resize_preserves_bayer_phase_and_native_samples() {
        let width = 8usize;
        let height = 4usize;
        let stride = width / 4 * 5;
        let mut bytes = Vec::with_capacity(stride * height);
        for y in 0..height {
            for x in (0..width).step_by(4) {
                push_raw10_group(
                    &mut bytes,
                    [
                        (y * 100 + x) as u16,
                        (y * 100 + x + 1) as u16,
                        (y * 100 + x + 2) as u16,
                        (y * 100 + x + 3) as u16,
                    ],
                );
            }
        }
        let frame = RawFrame {
            bytes: bytes.clone(),
            stride: stride as u32,
            timestamp_ns: 0,
        };

        assert_eq!(resample_raw10_bayer(&frame, 8, 4, 8, 4).unwrap(), bytes,);
        let reduced = resample_raw10_bayer(&frame, 8, 4, 4, 2).unwrap();
        assert_eq!(reduced.len(), 10);
        assert_eq!(
            (0..4)
                .map(|x| packed_raw10_pixel_bytes(&reduced, 5, x, 0))
                .collect::<Vec<_>>(),
            vec![202, 203, 206, 207],
        );
        assert_eq!(
            (0..4)
                .map(|x| packed_raw10_pixel_bytes(&reduced, 5, x, 1))
                .collect::<Vec<_>>(),
            vec![302, 303, 306, 307],
        );
        assert!(resample_raw10_bayer(&frame, 8, 4, 4, 3).is_err());
    }

    #[test]
    fn tracking_context_uses_an_exact_fixed_bayer_phase() {
        let frame = RawFrame {
            bytes: vec![0; 8 * 10],
            stride: 10,
            timestamp_ns: 0,
        };
        assert_eq!(
            downsample_raw10_tracking_context(&frame, 8, 8, 2, 2)
                .unwrap()
                .len(),
            8,
        );
        assert!(downsample_raw10_tracking_context(&frame, 8, 8, 4, 3).is_err());

        let odd_scale = RawFrame {
            bytes: vec![0; 12 * 15],
            stride: 15,
            timestamp_ns: 0,
        };
        assert_eq!(
            downsample_raw10_tracking_context(&odd_scale, 12, 12, 4, 4)
                .unwrap()
                .len(),
            32,
        );
    }

    #[test]
    fn vcm_settle_estimate_is_conservative_and_distance_aware() {
        assert_eq!(vcm_settle_duration(400, 400), Duration::from_millis(250));
        assert_eq!(vcm_settle_duration(0, 1023), Duration::from_millis(505));
        assert_eq!(vcm_settle_duration(1023, 0), Duration::from_millis(505));
    }
}
