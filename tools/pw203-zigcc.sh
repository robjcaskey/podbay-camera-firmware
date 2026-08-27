#!/bin/sh
set -eu

# This wrapper never installs or downloads Zig. The caller must provide an
# existing `zig` executable or set ZIG to its absolute path.
zig_bin=${ZIG:-zig}
exec "$zig_bin" cc -target arm-linux-gnueabihf.2.24 "$@"

