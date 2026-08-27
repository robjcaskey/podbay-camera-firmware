# Source-only boundary

This repository starts with two source files selectively copied from the local
development repository and one newly written proof viewer:

- `camera-service/src/main.rs`: standalone camera-side RAW10 service authored
  in the development project.
- `tools/install.py`: fail-closed one-shot boot-hook installer authored in the
  development project.
- `tools/web_viewer.py`: dependency-free protocol proof client created for this
  source-only repository.
- `tools/deploy.py`: authored host-side orchestration that reads supported
  camera files into a private temporary directory. It contains hashes and file
  paths but no camera-provided file content.
- `tools/pw203_netmode.py`: authored host-side implementation of the observed
  PW203 UVC RPC framing. Every message is generated from field-level command
  identifiers, lengths, sequence numbers, address data, and computed CRCs. It
  contains no captured 60-byte transfer buffer or response.
- `tools/pw203_bootstrap_ssh.py`: authored host-side temporary SSH bootstrap.
  Its shell script and SSH host key are generated locally at runtime; no camera
  file is copied into the repository.
- `tools/inspect_sensor_abi.py`: authored bounded ELF metadata inspector. It
  reports hashes, module metadata, section sizes, and undefined symbol names;
  it does not reproduce instructions or section contents.
- `tools/inspect_sensor_callbacks.py`: authored relocation-fact verifier. It
  reports callback names and handle offsets but no module instructions or data.
- `sensor-provider/`: independently authored portable IMX582 register-plan and
  SigmaStar handle code, host tests, and kernel gates. The warm provider is
  based on measured structure/calling facts and contains no copied vendor
  header, register table, firmware, or binary data.
- `sensor-provider/registration-canary/`: independently authored lifecycle
  probe using only six measured SigmaStar export signatures. Its callback
  refuses activation without dereferencing the framework-owned handle.

No generated executable, patched kernel module, firmware image, filesystem
extraction, vendor installer, SDK archive, shared library, captured frame,
model, or media file is part of the repository.

The earlier USB mode-switch implementation and its captured transfer buffers
remain excluded. The replacement retains only protocol facts: standard USB
control-transfer parameters, the semantic preflight order, command identifiers,
field encodings, and checksum algorithm. Padding is generated as zeroes rather
than copied from a trace. The generated sequence has not yet been exercised on
hardware from this repository.

The SSH bootstrap invokes an HTTP shell endpoint and SSH daemon already present
in the user's installed firmware. Those camera programs are not copied,
downloaded, or distributed here. This is a runtime dependency on supported
installed firmware, not a bundled dependency.

The camera service declares and calls the camera's existing SigmaStar `mi_sys`,
`mi_sensor`, and `mi_vif` library ABIs. Those libraries are not included. It
also directly controls I2C devices. A future reproducible build/install design
must obtain any required target libraries from hardware the user owns or use a
documented redistributable SDK; it must not silently download or redistribute
them.

This is a technical content boundary, not a legal opinion. Before publication,
review the authored source and ABI/register descriptions for licensing, patent,
and reverse-engineering obligations in the intended jurisdictions, and choose
an explicit license for the new repository.
