#!/usr/bin/env python3
"""Micro equivalency tests for sim2650 against winarcadia-2650 semantics.

These are differential-oracle tests derived from the behavior in local 2650.c,
focused on areas that previously diverged.
"""

from __future__ import annotations

import re
import subprocess
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SIM_SRC = ROOT / "sim2650.c"
SIM_BIN = ROOT / "sim2650_test"


def intel_hex(records: list[tuple[int, list[int]]]) -> str:
    lines: list[str] = []
    for addr, data in records:
        n = len(data)
        rec = [n, (addr >> 8) & 0xFF, addr & 0xFF, 0x00, *data]
        csum = ((~(sum(rec) & 0xFF) + 1) & 0xFF)
        lines.append(":" + "".join(f"{b:02X}" for b in rec) + f"{csum:02X}")
    lines.append(":00000001FF")
    return "\n".join(lines) + "\n"


def build() -> None:
    subprocess.run(
        ["gcc", "-Wall", "-Wextra", "-O2", "-o", str(SIM_BIN), str(SIM_SRC)],
        check=True,
        cwd=ROOT,
    )


def run_prog(code: list[int], extra_args: list[str] | None = None) -> tuple[int, str]:
    extra_args = extra_args or []
    with tempfile.TemporaryDirectory() as td:
        hex_path = Path(td) / "prog.hex"
        hex_path.write_text(intel_hex([(0x0000, code)]), encoding="ascii")
        proc = subprocess.run(
            [str(SIM_BIN), *extra_args, str(hex_path)],
            cwd=ROOT,
            text=True,
            capture_output=True,
        )
    return proc.returncode, proc.stderr


def parse_state(stderr: str) -> dict[str, int]:
    m_regs = re.search(r"R0=\$(..) R1=\$(..) R2=\$(..) R3=\$(..)", stderr)
    m_psl = re.search(r"PSL=\$(..)", stderr)
    if not m_regs or not m_psl:
        raise AssertionError(f"Could not parse simulator state from output:\n{stderr}")
    return {
        "R0": int(m_regs.group(1), 16),
        "R1": int(m_regs.group(2), 16),
        "R2": int(m_regs.group(3), 16),
        "R3": int(m_regs.group(4), 16),
        "PSL": int(m_psl.group(1), 16),
    }


def test_halt_mode_toggle() -> None:
    code = [0x40, 0x04, 0x2A, 0xC0]

    rc, out = run_prog(code)
    st = parse_state(out)
    assert rc == 0
    assert st["R0"] == 0x00, f"default HALT should stop before LODI, got R0={st['R0']:02X}"

    rc, out = run_prog(code, ["--halt-continue", "-b", "0003"])
    st = parse_state(out)
    assert rc == 0
    assert st["R0"] == 0x2A, f"--halt-continue should execute LODI past HALT, got R0={st['R0']:02X}"


def test_dar_sets_cc_like_reference() -> None:
    # LODI,R1 #09; DAR,R1; HALT
    code = [0x05, 0x09, 0x95, 0x40]
    rc, out = run_prog(code)
    st = parse_state(out)
    assert rc == 0
    assert st["R1"] == 0xA3, f"DAR result mismatch, expected A3 got {st['R1']:02X}"
    assert (st["PSL"] & 0xC0) == 0x80, f"DAR CC mismatch, expected LT (0x80), got {st['PSL'] & 0xC0:02X}"


def test_strz_sets_cc() -> None:
    # LODI,R0 #00; LODI,R1 #55; STRZ,R1; HALT
    code = [0x04, 0x00, 0x05, 0x55, 0xC1, 0x40]
    rc, out = run_prog(code)
    st = parse_state(out)
    assert rc == 0
    assert st["R1"] == 0x00, "STRZ should copy R0 into R1"
    assert (st["PSL"] & 0xC0) == 0x00, f"STRZ should set CC=EQ for zero write, got {st['PSL'] & 0xC0:02X}"


def test_undefined_90_91_no_fault() -> None:
    # 0x90; 0x91; LODI,R0 #42; HALT
    code = [0x90, 0x91, 0x04, 0x42, 0x40]
    rc, out = run_prog(code)
    st = parse_state(out)
    assert rc == 0, f"Undefined opcodes should not fault, rc={rc}\n{out}"
    assert st["R0"] == 0x42, "Execution should continue after 0x90/0x91"


def test_rrr_rrl_ovf_edges() -> None:
    # RRR case: LODI,R1 #80; PPSL #08 (WC=1); RRR,R1; HALT
    code_rrr = [0x05, 0x80, 0x77, 0x08, 0x51, 0x40]
    rc, out = run_prog(code_rrr)
    st = parse_state(out)
    assert rc == 0
    assert (st["PSL"] & 0x04) == 0x00, f"RRR OVF edge mismatch, expected OVF clear got PSL={st['PSL']:02X}"

    # RRL case: LODI,R1 #80; PPSL #08 (WC=1); RRL,R1; HALT
    code_rrl = [0x05, 0x80, 0x77, 0x08, 0xD1, 0x40]
    rc, out = run_prog(code_rrl)
    st = parse_state(out)
    assert rc == 0
    assert (st["PSL"] & 0x04) == 0x00, f"RRL OVF edge mismatch, expected OVF clear got PSL={st['PSL']:02X}"


def main() -> None:
    build()
    tests = [
        test_halt_mode_toggle,
        test_dar_sets_cc_like_reference,
        test_strz_sets_cc,
        test_undefined_90_91_no_fault,
        test_rrr_rrl_ovf_edges,
    ]
    for t in tests:
        t()
        print(f"PASS: {t.__name__}")
    print("All equivalency micro-tests passed.")


if __name__ == "__main__":
    main()
