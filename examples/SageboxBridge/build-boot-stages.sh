#!/usr/bin/env bash
# Build the four boot-bisect uf2s.
#
# A firmware that does not enumerate cannot tell you why it did not enumerate.
# The only instrument left is which build comes up, so the boot path is cut into
# four and flashed one at a time. See SAGEBOX_BOOT_STAGE in main.cpp.
#
#   stage0  core 1, USB and the LED. No PIO touched at all.
#   stage1  + the two front ports, polled and framed. What a10dce2 did.
#   stage2  + the two console ports claimed and their program loaded, never answered.
#   stage3  + answering the consoles. Identical to a plain build.
#
# The LED blinks from core 0 at every stage. A dark board means core 0 never
# reached its loop; a blinking board with no USB means core 1 did not survive.
#
# Flash one with:
#   picotool load -f -x examples/build/SageboxBridge/SageboxBridge-stage0.uf2
#
# Leaves the build tree configured at stage 3, so an ordinary
# `cmake --build examples/build --target SageboxBridge` still produces the real
# firmware afterwards.
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
build="$here/../build"
out="$build/SageboxBridge"

for stage in 0 1 2 3; do
  echo "=== stage $stage ==="
  cmake -DSAGEBOX_BOOT_STAGE="$stage" "$build" >/dev/null
  cmake --build "$build" --target SageboxBridge -j8 >/dev/null
  cp "$out/SageboxBridge.uf2" "$out/SageboxBridge-stage$stage.uf2"
  arm-none-eabi-size "$out/SageboxBridge.elf" | tail -1
done

echo
ls -l "$out"/SageboxBridge-stage*.uf2
