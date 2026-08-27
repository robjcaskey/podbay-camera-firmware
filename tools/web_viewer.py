#!/usr/bin/env python3
"""Host-side browser viewer for the Podbay camera service protocol.

Run this program on the user's computer, not on the camera. It is deliberately
dependency-free. It asks the camera over TCP for a downsampled 16-bit tracking
thumbnail and exposes a contrast-scaled BMP to a browser on the same computer.
It does not use UVC, a desktop capture, or a vendor camera application.
"""

from __future__ import annotations

import argparse
import json
import socket
import struct
import threading
import time
from array import array
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse


HEADER_BYTES = 64
PROTOCOL_VERSION = 26
FORMAT_GRAY16 = 3
CAMERA_LOCK = threading.Lock()

INDEX_HTML = b"""<!doctype html>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Podbay camera proof</title>
<style>
  :root { color-scheme: dark; font: 15px system-ui, sans-serif; }
  body { margin: 0; min-height: 100vh; display: grid; place-items: center;
         background: #101216; color: #e8edf2; }
  main { width: min(94vw, 1000px); }
  header { display: flex; align-items: baseline; justify-content: space-between;
           gap: 1rem; margin: 0 0 .8rem; }
  h1 { font-size: 1.05rem; font-weight: 650; margin: 0; }
  #status { color: #9eabb8; font-variant-numeric: tabular-nums; }
  img { display: block; width: 100%; height: auto; background: #000;
        border: 1px solid #303944; image-rendering: auto; }
  p { color: #8f9aa5; margin: .7rem 0 0; }
</style>
<main>
  <header><h1>Podbay camera protocol proof</h1><span id="status">connecting</span></header>
  <img id="frame" alt="Live lossless-protocol camera thumbnail">
  <p>Direct protocol thumbnail; no UVC or proprietary desktop camera process.</p>
</main>
<script>
const image = document.querySelector('#frame');
const status = document.querySelector('#status');
let frames = 0;
let started = performance.now();
async function next() {
  const began = performance.now();
  try {
    const response = await fetch('/frame.bmp?t=' + Date.now(), {cache: 'no-store'});
    if (!response.ok) throw new Error(await response.text());
    const blob = await response.blob();
    const old = image.src;
    image.src = URL.createObjectURL(blob);
    if (old.startsWith('blob:')) URL.revokeObjectURL(old);
    frames++;
    const fps = frames * 1000 / Math.max(1, performance.now() - started);
    status.textContent = fps.toFixed(1) + ' fps | ' +
      Math.round(performance.now() - began) + ' ms';
  } catch (error) {
    status.textContent = 'error: ' + error.message.trim();
  }
  setTimeout(next, 100);
}
next();
</script>
"""


def receive_exact(stream, size: int) -> bytes:
    chunks: list[bytes] = []
    remaining = size
    while remaining:
        chunk = stream.read(remaining)
        if not chunk:
            raise RuntimeError(f"camera closed with {remaining} bytes outstanding")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def parse_camera(value: str) -> tuple[str, int]:
    host, separator, port = value.rpartition(":")
    if separator and host and port.isdigit():
        return host, int(port)
    return value, 5001


def largest_divisor_at_most(value: int, limit: int) -> int:
    for candidate in range(min(value, limit), 0, -1):
        if value % candidate == 0:
            return candidate
    raise AssertionError("one always divides a positive integer")


def read_line(stream) -> str:
    line = stream.readline(4096)
    if not line:
        raise RuntimeError("camera closed before its text response")
    return line.decode("ascii", errors="replace").strip()


def camera_status(camera: tuple[str, int], timeout: float) -> dict[str, int | str]:
    with socket.create_connection(camera, timeout=timeout) as connection:
        connection.settimeout(timeout)
        stream = connection.makefile("rwb", buffering=0)
        stream.write(b"VERSION\n")
        version_line = read_line(stream)
        expected = f"OK CUSTOM_CAMERA {PROTOCOL_VERSION}"
        if version_line != expected:
            raise RuntimeError(f"protocol mismatch: expected {expected!r}, got {version_line!r}")
        stream.write(b"SENSOR GET\n")
        fields = read_line(stream).split()
        if len(fields) != 9 or fields[:2] != ["OK", "SENSOR"]:
            raise RuntimeError(f"invalid SENSOR response: {' '.join(fields)!r}")
        values = [int(value) for value in fields[2:]]
        names = ("sensor_x", "sensor_y", "physical_width", "physical_height",
                 "binning", "output_width", "output_height")
        return {"protocol": PROTOCOL_VERSION, **dict(zip(names, values, strict=True))}


def capture_thumbnail(
    camera: tuple[str, int], timeout: float, requested_width: int, requested_height: int
) -> tuple[bytes, dict[str, int | str]]:
    with CAMERA_LOCK, socket.create_connection(camera, timeout=timeout) as connection:
        connection.settimeout(timeout)
        stream = connection.makefile("rwb", buffering=0)
        stream.write(b"VERSION\n")
        version_line = read_line(stream)
        expected = f"OK CUSTOM_CAMERA {PROTOCOL_VERSION}"
        if version_line != expected:
            raise RuntimeError(f"protocol mismatch: {version_line}")

        stream.write(b"SENSOR GET\n")
        fields = read_line(stream).split()
        if len(fields) != 9 or fields[:2] != ["OK", "SENSOR"]:
            raise RuntimeError(f"invalid SENSOR response: {' '.join(fields)}")
        sensor_x, sensor_y, _, _, _, output_width, output_height = map(int, fields[2:])
        width = largest_divisor_at_most(output_width, requested_width)
        height = largest_divisor_at_most(output_height, requested_height)

        sequence = time.monotonic_ns() & 0xFFFF_FFFF_FFFF_FFFF
        command = f"CAPTURE_THUMB {sequence} {sensor_x} {sensor_y} 0 {width} {height}\n"
        stream.write(command.encode("ascii"))
        header = receive_exact(stream, HEADER_BYTES)
        if header[:4] != b"OTH1":
            raise RuntimeError(f"expected OTH1 thumbnail, got {header[:4]!r}")
        version, header_bytes = struct.unpack_from("<HH", header, 4)
        status, pixel_format = struct.unpack_from("<iI", header, 8)
        returned_sequence, timestamp_ns = struct.unpack_from("<QQ", header, 16)
        returned_width, returned_height, stride, payload_bytes = struct.unpack_from("<IIII", header, 40)
        if version != 1 or header_bytes != HEADER_BYTES or status != 0:
            raise RuntimeError(f"camera thumbnail failed: version={version} status={status}")
        if pixel_format != FORMAT_GRAY16 or returned_sequence != sequence:
            raise RuntimeError("camera returned an incompatible thumbnail packet")
        if (returned_width, returned_height, stride, payload_bytes) != (
            width, height, width * 2, width * height * 2
        ):
            raise RuntimeError("camera returned inconsistent thumbnail geometry")
        payload = receive_exact(stream, payload_bytes)
        metadata: dict[str, int | str] = {
            "protocol": PROTOCOL_VERSION,
            "sequence": sequence,
            "timestamp_ns": timestamp_ns,
            "sensor_x": sensor_x,
            "sensor_y": sensor_y,
            "width": width,
            "height": height,
        }
        return gray16_to_bmp(payload, width, height), metadata


def percentile(histogram: list[int], target: int) -> int:
    seen = 0
    for value, count in enumerate(histogram):
        seen += count
        if seen >= target:
            return value
    return len(histogram) - 1


def gray16_to_bmp(payload: bytes, width: int, height: int) -> bytes:
    pixels = array("H")
    pixels.frombytes(payload)
    if struct.pack("=H", 1) != b"\x01\x00":
        pixels.byteswap()
    histogram = [0] * 1024
    for value in pixels:
        histogram[min(value, 1023)] += 1
    count = len(pixels)
    low = percentile(histogram, max(1, count // 100))
    high = max(low + 1, percentile(histogram, max(1, count * 99 // 100)))
    row_bytes = width * 3
    padding = (-row_bytes) & 3
    image = bytearray()
    for y in range(height - 1, -1, -1):
        row = bytearray()
        start = y * width
        for value in pixels[start : start + width]:
            level = max(0, min(255, (int(value) - low) * 255 // (high - low)))
            row.extend((level, level, level))
        row.extend(b"\0" * padding)
        image.extend(row)
    offset = 14 + 40
    file_size = offset + len(image)
    file_header = struct.pack("<2sIHHI", b"BM", file_size, 0, 0, offset)
    info_header = struct.pack(
        "<IiiHHIIiiII", 40, width, height, 1, 24, 0, len(image), 2835, 2835, 0, 0
    )
    return file_header + info_header + image


class ViewerServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address, handler, args: argparse.Namespace):
        super().__init__(address, handler)
        self.args = args


class Handler(BaseHTTPRequestHandler):
    server: ViewerServer

    def do_GET(self) -> None:
        path = urlparse(self.path).path
        try:
            if path == "/":
                self.send_bytes(HTTPStatus.OK, "text/html; charset=utf-8", INDEX_HTML)
            elif path == "/status.json":
                with CAMERA_LOCK:
                    result = camera_status(self.server.args.camera, self.server.args.timeout)
                self.send_json(HTTPStatus.OK, result)
            elif path == "/frame.bmp":
                frame, metadata = capture_thumbnail(
                    self.server.args.camera,
                    self.server.args.timeout,
                    self.server.args.width,
                    self.server.args.height,
                )
                self.send_bytes(
                    HTTPStatus.OK,
                    "image/bmp",
                    frame,
                    {"X-Podbay-Frame": json.dumps(metadata, separators=(",", ":"))},
                )
            else:
                self.send_json(HTTPStatus.NOT_FOUND, {"error": "not found"})
        except (OSError, RuntimeError, ValueError) as error:
            self.send_json(HTTPStatus.BAD_GATEWAY, {"error": str(error)})

    def send_json(self, status: HTTPStatus, value: object) -> None:
        self.send_bytes(status, "application/json", json.dumps(value).encode("utf-8"))

    def send_bytes(
        self, status: HTTPStatus, content_type: str, body: bytes, extra: dict[str, str] | None = None
    ) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        for name, value in (extra or {}).items():
            self.send_header(name, value)
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, format: str, *args: object) -> None:
        return


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera", default="192.168.88.10:5001")
    parser.add_argument("--listen", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=360)
    parser.add_argument("--timeout", type=float, default=5.0)
    args = parser.parse_args()
    if args.width <= 0 or args.height <= 0 or args.timeout <= 0:
        parser.error("width, height, and timeout must be positive")
    args.camera = parse_camera(args.camera)
    server = ViewerServer((args.listen, args.port), Handler, args)
    print(f"viewer: http://{args.listen}:{args.port}/", flush=True)
    print(f"camera: {args.camera[0]}:{args.camera[1]} protocol={PROTOCOL_VERSION}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
