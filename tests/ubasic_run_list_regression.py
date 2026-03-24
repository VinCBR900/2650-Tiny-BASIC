#!/usr/bin/env python3
"""Regression probe for the current uBASIC RUN/LIST parser bug on sim2650.

Builds assembler/simulator, assembles uBASIC, feeds a small script, and flags
failure if output contains '?0' syntax errors or the simulator hits the
instruction-limit loop trap.
"""

import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
ASM = ROOT / "asm2650"
SIM = ROOT / "sim2650"
ASM_SRC = ROOT / "asm2650_v1.2.c"
SIM_SRC = ROOT / "sim2650_v1.2.c"
UBASIC_SRC = ROOT / "uBASIC2650.asm"
UBASIC_HEX = ROOT / "ubasic.hex"

SCRIPT = """10 LET A=1
20 PRINT A
LIST
RUN
"""


def run(cmd):
    return subprocess.run(cmd, capture_output=True, text=True, cwd=ROOT)


def main() -> int:
    for out, src in [(ASM, ASM_SRC), (SIM, SIM_SRC)]:
        p = run(["gcc", "-Wall", "-O2", "-o", str(out), str(src)])
        if p.returncode != 0:
            print(p.stderr)
            return 2

    p = run([str(ASM), str(UBASIC_SRC), str(UBASIC_HEX)])
    if p.returncode != 0:
        print(p.stderr)
        return 2

    rx = ROOT / "tests" / "_run_list_rx.txt"
    rx.write_text(SCRIPT)
    p = run([str(SIM), "-rx", str(rx), "--allow-ram-image", str(UBASIC_HEX)])

    stdout = p.stdout
    stderr = p.stderr

    has_syntax = "?0" in stdout
    hit_instr_limit = "Instruction limit" in stderr or p.returncode == 3

    print("=== sim stdout ===")
    print(stdout)
    print("=== sim stderr tail ===")
    print("\n".join(stderr.strip().splitlines()[-8:]))

    if has_syntax or hit_instr_limit:
        print("REGRESSION: RUN/LIST scenario still fails on current simulator build.")
        return 1

    print("PASS: RUN/LIST scenario completed without ?0 or instruction-limit loop.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
