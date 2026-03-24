#!/usr/bin/env python3
import subprocess
import tempfile
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SIM_SRC = ROOT / "sim2650_v1.2.c"
SIM_BIN = ROOT / "sim2650"


def build_sim():
    subprocess.run(["gcc", "-Wall", "-O2", "-o", str(SIM_BIN), str(SIM_SRC)], check=True)


def to_ihex(image: dict[int, int]) -> str:
    lines = []
    addresses = sorted(image)
    i = 0
    while i < len(addresses):
        start = addresses[i]
        chunk = [image[start]]
        i += 1
        while i < len(addresses) and addresses[i] == start + len(chunk) and len(chunk) < 16:
            chunk.append(image[addresses[i]])
            i += 1
        count = len(chunk)
        addr_hi = (start >> 8) & 0xFF
        addr_lo = start & 0xFF
        rectype = 0
        csum = (-(count + addr_hi + addr_lo + rectype + sum(chunk))) & 0xFF
        line = ":" + f"{count:02X}{start:04X}{rectype:02X}" + "".join(f"{b:02X}" for b in chunk) + f"{csum:02X}"
        lines.append(line)
    lines.append(":00000001FF")
    return "\n".join(lines) + "\n"


def run_image(image: dict[int, int]):
    with tempfile.NamedTemporaryFile("w", suffix=".hex", delete=False) as f:
        f.write(to_ihex(image))
        hex_path = f.name
    p = subprocess.run([str(SIM_BIN), hex_path], capture_output=True, text=True)
    stderr = p.stderr

    regs_match = re.search(r"R0=\$(..) R1=\$(..) R2=\$(..) R3=\$(..)", stderr)
    iar_match = re.search(r"IAR=\$(....) PSU=\$(..) PSL=\$(..) CC=(\d)", stderr)
    if not regs_match or not iar_match:
        raise AssertionError(f"Simulator output parse failed. stderr:\n{stderr}")

    regs = tuple(int(x, 16) for x in regs_match.groups())
    iar, psu, psl, cc = iar_match.groups()
    return {
        "code": p.returncode,
        "regs": regs,
        "iar": int(iar, 16),
        "psu": int(psu, 16),
        "psl": int(psl, 16),
        "cc": int(cc),
        "stderr": stderr,
    }


def put(image, addr, *bytes_):
    for i, b in enumerate(bytes_):
        image[addr + i] = b & 0xFF


def test_comi_bcta(cond_opcode, a, b, should_branch):
    image = {}
    put(image, 0x0000, 0x04, a)                # LODI,R0,#a
    put(image, 0x0002, 0xE4, b)                # COMI,R0,#b
    put(image, 0x0004, cond_opcode, 0x00, 0x10)  # BCTA,cond $0010
    put(image, 0x0007, 0x05, 0x11, 0x40)       # LODI,R1,#0x11 ; HALT
    put(image, 0x0010, 0x05, 0x22, 0x40)       # LODI,R1,#0x22 ; HALT
    res = run_image(image)
    expected_r1 = 0x22 if should_branch else 0x11
    assert res["code"] == 0, res["stderr"]
    assert res["regs"][1] == expected_r1, res["stderr"]


def test_subi_bcta(cond_opcode, a, b, should_branch):
    image = {}
    put(image, 0x0000, 0x05, a)                # LODI,R1,#a
    put(image, 0x0002, 0xA5, b)                # SUBI,R1,#b
    put(image, 0x0004, cond_opcode, 0x00, 0x10)  # BCTA,cond $0010
    put(image, 0x0007, 0x06, 0x11, 0x40)       # LODI,R2,#0x11 ; HALT
    put(image, 0x0010, 0x06, 0x22, 0x40)       # LODI,R2,#0x22 ; HALT
    res = run_image(image)
    expected_r2 = 0x22 if should_branch else 0x11
    assert res["code"] == 0, res["stderr"]
    assert res["regs"][2] == expected_r2, res["stderr"]


def test_addi_wrap_eq_and_carry():
    image = {}
    put(image, 0x0000, 0x05, 0xFF)                 # LODI,R1,#0xFF
    put(image, 0x0002, 0x85, 0x01)                 # ADDI,R1,#1 => 0x00, C=1, CC=EQ
    put(image, 0x0004, 0x1C, 0x00, 0x10)           # BCTA,EQ $0010
    put(image, 0x0007, 0x07, 0x10, 0x40)           # fail: R3=0x10 ; HALT
    put(image, 0x0010, 0xB5, 0x01)                 # TPSL #C
    put(image, 0x0012, 0x1D, 0x00, 0x18)           # BCTA,GT $0018 (carry should set -> GT)
    put(image, 0x0015, 0x07, 0x11, 0x40)           # fail: R3=0x11 ; HALT
    put(image, 0x0018, 0x07, 0x33, 0x40)           # pass: R3=0x33 ; HALT
    res = run_image(image)
    assert res["code"] == 0, res["stderr"]
    assert res["regs"][3] == 0x33, res["stderr"]


def test_addi_nonwrap_gt_and_no_carry():
    image = {}
    put(image, 0x0000, 0x05, 0x10)                 # LODI,R1,#0x10
    put(image, 0x0002, 0x85, 0x01)                 # ADDI,R1,#1 => 0x11, C=0, CC=GT
    put(image, 0x0004, 0x1D, 0x00, 0x10)           # BCTA,GT $0010
    put(image, 0x0007, 0x07, 0x20, 0x40)           # fail: R3=0x20 ; HALT
    put(image, 0x0010, 0xB5, 0x01)                 # TPSL #C (expect zero)
    put(image, 0x0012, 0x1C, 0x00, 0x18)           # BCTA,EQ $0018
    put(image, 0x0015, 0x07, 0x21, 0x40)           # fail: R3=0x21 ; HALT
    put(image, 0x0018, 0x07, 0x44, 0x40)           # pass: R3=0x44 ; HALT
    res = run_image(image)
    assert res["code"] == 0, res["stderr"]
    assert res["regs"][3] == 0x44, res["stderr"]


def test_addi_zero_without_carry_still_gt():
    image = {}
    put(image, 0x0000, 0x05, 0x00)                 # LODI,R1,#0x00
    put(image, 0x0002, 0x85, 0x00)                 # ADDI,R1,#0x00 => 0x00, C=0, expect CC=GT
    put(image, 0x0004, 0x1D, 0x00, 0x10)           # BCTA,GT $0010
    put(image, 0x0007, 0x07, 0x50, 0x40)           # fail: R3=0x50 ; HALT
    put(image, 0x0010, 0x07, 0x55, 0x40)           # pass: R3=0x55 ; HALT
    res = run_image(image)
    assert res["code"] == 0, res["stderr"]
    assert res["regs"][3] == 0x55, res["stderr"]


def test_bsta_retc_nested_chain():
    image = {}
    put(image, 0x0000, 0x3F, 0x00, 0x20)           # BSTA,UN $0020
    put(image, 0x0003, 0x04, 0x55, 0x40)           # LODI,R0,#0x55 ; HALT

    put(image, 0x0020, 0x05, 0x11)                 # sub1: LODI,R1,#0x11
    put(image, 0x0022, 0x3F, 0x00, 0x30)           # BSTA,UN $0030
    put(image, 0x0025, 0x85, 0x01)                 # ADDI,R1,#1 => 0x12 after sub2 returns
    put(image, 0x0027, 0x17)                       # RETC,UN

    put(image, 0x0030, 0x06, 0x22)                 # sub2: LODI,R2,#0x22
    put(image, 0x0032, 0x17)                       # RETC,UN

    res = run_image(image)
    assert res["code"] == 0, res["stderr"]
    assert res["regs"][0] == 0x55, res["stderr"]
    assert res["regs"][1] == 0x12, res["stderr"]
    assert res["regs"][2] == 0x22, res["stderr"]
    assert (res["psu"] & 0x07) == 0, res["stderr"]  # SP restored after nested returns


def test_bsta_conditional_not_taken_does_not_push():
    image = {}
    put(image, 0x0000, 0x3F, 0x00, 0x20)           # BSTA,UN $0020
    put(image, 0x0003, 0x40)                       # HALT

    put(image, 0x0020, 0x05, 0x00)                 # sub1: LODI,R1,#0
    put(image, 0x0022, 0x04, 0x00)                 # LODI,R0,#0
    put(image, 0x0024, 0xE4, 0x01)                 # COMI,R0,#1 => LT (so EQ is false)
    put(image, 0x0026, 0x3C, 0x00, 0x30)           # BSTA,EQ $0030 (must NOT push when not taken)
    put(image, 0x0029, 0x85, 0x01)                 # ADDI,R1,#1 (should execute once)
    put(image, 0x002B, 0x17)                       # RETC,UN

    put(image, 0x0030, 0x05, 0xFF)                 # sub2: unreachable if EQ false
    put(image, 0x0032, 0x17)                       # RETC,UN

    res = run_image(image)
    assert res["code"] == 0, res["stderr"]
    assert res["regs"][1] == 0x01, res["stderr"]   # fails as 0x02 if non-taken BSTA still pushes
    assert (res["psu"] & 0x07) == 0, res["stderr"]


def test_stra_does_not_clobber_cc():
    image = {}
    put(image, 0x0000, 0x05, 0x00)                 # LODI,R1,#0
    put(image, 0x0002, 0xE5, 0x01)                 # COMI,R1,#1 => LT
    put(image, 0x0004, 0xCD, 0x14, 0x90)           # STRA,R1,$1490 (must not alter CC)
    put(image, 0x0007, 0x1E, 0x00, 0x10)           # BCTA,LT $0010
    put(image, 0x000A, 0x07, 0x60, 0x40)           # fail: R3=0x60 ; HALT
    put(image, 0x0010, 0x07, 0x66, 0x40)           # pass: R3=0x66 ; HALT
    res = run_image(image)
    assert res["code"] == 0, res["stderr"]
    assert res["regs"][3] == 0x66, res["stderr"]


def main():
    build_sim()

    test_comi_bcta(0x1C, 0x2A, 0x2A, True)   # EQ
    test_comi_bcta(0x1D, 0x05, 0x01, True)   # GT
    test_comi_bcta(0x1E, 0x01, 0x05, True)   # LT

    test_subi_bcta(0x1C, 0x33, 0x33, True)   # EQ
    test_subi_bcta(0x1D, 0x05, 0x01, True)   # GT
    test_subi_bcta(0x1E, 0x01, 0x02, True)   # LT

    test_addi_wrap_eq_and_carry()
    test_addi_nonwrap_gt_and_no_carry()
    test_addi_zero_without_carry_still_gt()

    test_bsta_retc_nested_chain()
    test_bsta_conditional_not_taken_does_not_push()
    test_stra_does_not_clobber_cc()

    test_bsta_retc_nested_chain()
    test_bsta_conditional_not_taken_does_not_push()

    print("All instruction micro-tests passed.")


if __name__ == "__main__":
    main()
