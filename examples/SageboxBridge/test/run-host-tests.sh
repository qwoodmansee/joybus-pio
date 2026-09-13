#!/usr/bin/env bash
# Compile and run the Sagebox framing tests with the HOST compiler.
#
# The encoder deliberately has no pico-sdk dependency so it can be checked on a
# laptop. This is the only part of the firmware that can be proven correct
# without hardware, and it is the part that matters most: a wrong byte layout
# produces plausible-looking wrong data all the way to the overlay.
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
out="$here/build"
mkdir -p "$out"

cc -std=c11 -Wall -Wextra -Werror -O1 \
  -o "$out/framing_test" \
  "$here/framing_test.c" \
  "$here/../sagebox_framing.c"

# The routing matrix is the other half that needs no hardware: which input feeds
# which console cable, how several merge into one report, and how a GameCube
# report becomes an N64 one. All of it is wrong-but-plausible when it breaks.
cc -std=c11 -Wall -Wextra -Werror -O1 \
  -o "$out/routing_test" \
  "$here/routing_test.c" \
  "$here/../sagebox_routing.c" \
  "$here/../sagebox_framing.c"

echo "framing:"
"$out/framing_test"
echo "routing:"
"$out/routing_test"
