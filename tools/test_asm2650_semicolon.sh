#!/usr/bin/env bash
set -euo pipefail

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
tmp_dir=$(mktemp -d)
trap 'rm -rf "$tmp_dir"' EXIT

asm_bin="$tmp_dir/asm2650"
asm_src="$tmp_dir/semicolon_literals.asm"
out_bin="$tmp_dir/out.bin"

gcc -Wall -O2 -o "$asm_bin" "$script_dir/asm2650.c"

cat > "$asm_src" <<'ASM'
        ORG $0000
        DB "PRINT ;"
        LODI,R1 ';'
        DB "A,B;C", ';', "X;" ; trailing comments still begin outside quotes
        END
ASM

"$asm_bin" -NoList --binary -o "$out_bin" -r '$0000-$0010' "$asm_src" >/dev/null

actual=$(od -An -tx1 -v "$out_bin" | tr -d ' \n')
expected="5052494e54203b053b412c423b433b583b"

if [[ "$actual" != "$expected" ]]; then
    echo "Unexpected bytes for quoted semicolon/comma regression" >&2
    echo "expected: $expected" >&2
    echo "actual:   $actual" >&2
    exit 1
fi

echo "quoted semicolon/comma regression passed"
