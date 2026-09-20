#!/bin/sh
# gen-variants.sh v1.0 - 2026-09-20
#
# Usage: gen-variants.sh <variant> [<variant> ...]
#
# For each <variant>, reads ../<variant>.LST (the sidecar listing asm2650
# writes next to the source .asm) and extracts the CHIN/COUT I/O addresses
# using the same grep idiom already used for the offline manual workflow
# (grep -n "^CHIN \|^COUT \|^ROMEND " %1.LST). Emits a JSON manifest on
# stdout mapping each variant to its assembled .hex path and the
# pipbug_wrap CLI args needed to run it in the browser simulator.
#
# --entry and --crlf are fixed across all variants (entry 0; crlf 7fff is
# the deprecated-but-required setting) per project convention - only
# --chin/--cout differ per variant/version.
#
# History:
#   v1.0 - initial version

set -eu

first=1
printf '{\n  "variants": [\n'
for v in "$@"; do
  lst="../${v}.LST"
  if [ ! -f "$lst" ]; then
    echo "gen-variants.sh: missing $lst (assemble $v first)" >&2
    exit 1
  fi
  chin=$(grep -m1 "^CHIN " "$lst" | sed -E 's/^CHIN[ \t]+\$([0-9A-Fa-f]{4}).*/\1/')
  cout=$(grep -m1 "^COUT " "$lst" | sed -E 's/^COUT[ \t]+\$([0-9A-Fa-f]{4}).*/\1/')
  if [ -z "$chin" ] || [ -z "$cout" ]; then
    echo "gen-variants.sh: could not find CHIN/COUT in $lst" >&2
    exit 1
  fi
  [ "$first" -eq 1 ] || printf ',\n'
  first=0
  printf '    { "id": "%s", "hex": "assets/%s.hex", "args": "-i --entry 0 --chin %s --cout %s -n 0 --crlf 7fff" }' \
    "$v" "$v" "$chin" "$cout"
done
printf '\n  ]\n}\n'
