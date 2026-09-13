#!/usr/bin/env bash
# Build the three boot probes.
#
# These are NOT the firmware. SageboxBridge brings up stdio on CORE 1; GcDiag,
# the only build proven to enumerate on this board, brings it up on CORE 0 and
# launches core 1 afterwards. a10dce2 was built but never flashed, so "core 1
# owns the USB stack" has never once been observed to work here. Each probe adds
# one ingredient to the shape that is known to work:
#
#   varA  stdio on core 0 and nothing else. No core 1, no PIO, no clock change,
#         and GP25 never touched.
#   varB  varA + core 1 launched into an empty sleep loop.
#   varC  varB + set_sys_clock_khz(130 MHz) before stdio init.
#   varD  varC + the GP25 toggle.
#
# Flash in order; the first one that does NOT enumerate names the ingredient.
# GP25 is last and alone because this is a Pico 2 W built as PICO_BOARD=pico2,
# where GP25 is the CYW43 chip select rather than an LED.
#
# Nothing lights up on this board whatever GP25 does, so the serial heartbeat
# each probe prints twice a second is the only signal there is:
#
#   picotool load -f -x examples/build/SageboxBridge/SageboxBridge-varA.uf2
#   screen /dev/cu.usbmodem* 115200      (exit: C-a k y)
#
# Leaves the build tree configured at stage 3, so an ordinary build still
# produces the real firmware afterwards.
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
build="$here/../build"
out="$build/SageboxBridge"

build_one() {
  local stage="$1" name="$2"
  echo "=== $name (stage $stage) ==="
  cmake -DSAGEBOX_BOOT_STAGE="$stage" "$build" >/dev/null
  cmake --build "$build" --target SageboxBridge -j8 >/dev/null
  cp "$out/SageboxBridge.uf2" "$out/SageboxBridge-$name.uf2"
}

build_one -1 varA
build_one -2 varB
build_one -3 varC
build_one -4 varD

# Back to the real firmware.
cmake -DSAGEBOX_BOOT_STAGE=3 "$build" >/dev/null
cmake --build "$build" --target SageboxBridge -j8 >/dev/null

echo
ls -l "$out"/SageboxBridge-var*.uf2
md5 "$out"/SageboxBridge-var*.uf2
