#!/usr/bin/env python3
"""
extract_aa_defines.py  —  Compare real aa.h against our aa.h stub
Version: 1.0
Date:    2026-04-07

Usage (Windows 11, offline, no extra packages needed):
    python extract_aa_defines.py  <real_aa.h>  [our_aa.h]

What it does:
  1. Parses every #define in the real aa.h (name + value/body).
  2. Parses every #define in our stub aa.h.
  3. Reports three categories:
       MISSING   - in real aa.h but not in our stub  (need to add)
       DIFFERENT - in both but value/body differs     (need to check)
       EXTRA     - in our stub but not in real aa.h   (harmless stubs)
  4. Writes a patch file  aa_patch.h  containing only the MISSING and
     DIFFERENT defines, ready to paste into our aa.h.

The script also extracts typedef definitions and flags any that are
present in the real aa.h but absent from our stub.
"""

import sys
import re
import os

# ── helpers ──────────────────────────────────────────────────────────────────

def parse_defines(path):
    """Return dict {name: body_string} for every #define in the file.
    Handles simple #define NAME and #define NAME value/expr.
    Does NOT expand macros — values are raw strings.
    Multi-line defines (backslash continuation) are joined."""
    defines = {}
    try:
        with open(path, 'r', encoding='utf-8', errors='replace') as f:
            raw = f.read()
    except FileNotFoundError:
        print(f"ERROR: Cannot open '{path}'")
        sys.exit(1)

    # Join backslash-continued lines
    raw = re.sub(r'\\\n', ' ', raw)

    for line in raw.splitlines():
        line = line.strip()
        # Strip C++ // comments
        line = re.sub(r'//.*$', '', line).strip()
        m = re.match(r'#\s*define\s+(\w+)(\s+(.*))?$', line)
        if not m:
            continue
        name = m.group(1)
        body = (m.group(3) or '').strip()
        defines[name] = body
    return defines


def parse_typedefs(path):
    """Return set of typedef names defined in the file."""
    names = set()
    try:
        with open(path, 'r', encoding='utf-8', errors='replace') as f:
            text = f.read()
    except FileNotFoundError:
        return names
    # Match: typedef ... NAME;  or  typedef ... NAME\n
    for m in re.finditer(r'\btypedef\b[^;]+\b(\w+)\s*;', text):
        names.add(m.group(1))
    return names


def normalise(s):
    """Collapse whitespace for comparison."""
    return re.sub(r'\s+', ' ', s).strip()

# ── main ─────────────────────────────────────────────────────────────────────

def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(0)

    real_path = sys.argv[1]
    stub_path = sys.argv[2] if len(sys.argv) >= 3 else 'aa.h'
    patch_path = 'aa_patch.h'

    print(f"Real aa.h : {real_path}")
    print(f"Stub aa.h : {stub_path}")
    print()

    real_defs = parse_defines(real_path)
    stub_defs = parse_defines(stub_path)

    real_types = parse_typedefs(real_path)
    stub_types = parse_typedefs(stub_path)

    missing    = {}   # in real, not in stub
    different  = {}   # in both, but body differs
    extra      = {}   # in stub, not in real

    for name, body in real_defs.items():
        if name not in stub_defs:
            missing[name] = body
        elif normalise(body) != normalise(stub_defs[name]):
            different[name] = (real_defs[name], stub_defs[name])

    for name in stub_defs:
        if name not in real_defs:
            extra[name] = stub_defs[name]

    missing_types = real_types - stub_types
    extra_types   = stub_types - real_types

    # ── Report ───────────────────────────────────────────────────────────────

    print(f"{'='*60}")
    print(f"  MISSING from stub  ({len(missing)} defines)")
    print(f"{'='*60}")
    if missing:
        for name in sorted(missing):
            val = missing[name]
            preview = val[:60] + ('...' if len(val) > 60 else '')
            print(f"  #define {name:<40s} {preview}")
    else:
        print("  (none)")

    print()
    print(f"{'='*60}")
    print(f"  DIFFERENT values   ({len(different)} defines)")
    print(f"{'='*60}")
    if different:
        for name in sorted(different):
            real_v, stub_v = different[name]
            print(f"  {name}")
            print(f"    real : {real_v[:70]}")
            print(f"    stub : {stub_v[:70]}")
    else:
        print("  (none)")

    print()
    print(f"{'='*60}")
    print(f"  EXTRA in stub only ({len(extra)} defines — usually harmless)")
    print(f"{'='*60}")
    if extra:
        for name in sorted(extra):
            print(f"  #define {name:<40s} {extra[name][:60]}")
    else:
        print("  (none)")

    print()
    print(f"{'='*60}")
    print(f"  TYPEDEFS missing from stub  ({len(missing_types)})")
    print(f"{'='*60}")
    if missing_types:
        for t in sorted(missing_types):
            print(f"  typedef ... {t};")
    else:
        print("  (none)")

    # ── Write patch file ─────────────────────────────────────────────────────

    with open(patch_path, 'w', encoding='utf-8') as f:
        f.write("/* aa_patch.h  — Auto-generated by extract_aa_defines.py\n")
        f.write(" * Paste the sections you need into aa.h.\n")
        f.write(" * Review DIFFERENT entries manually before replacing.\n")
        f.write(" */\n\n")

        f.write("/* ── MISSING defines (add these to aa.h) ── */\n")
        if missing:
            for name in sorted(missing):
                body = missing[name]
                if body:
                    f.write(f"#define {name} {body}\n")
                else:
                    f.write(f"#define {name}\n")
        else:
            f.write("/* none */\n")

        f.write("\n/* ── DIFFERENT defines (check and replace as needed) ── */\n")
        if different:
            for name in sorted(different):
                real_v, stub_v = different[name]
                f.write(f"/* stub has: #define {name} {stub_v} */\n")
                if real_v:
                    f.write(f"#define {name} {real_v}\n")
                else:
                    f.write(f"#define {name}\n")
        else:
            f.write("/* none */\n")

    print()
    print(f"Patch file written to: {patch_path}")
    print()
    print(f"Summary: {len(missing)} missing, {len(different)} different, "
          f"{len(extra)} extra, {len(missing_types)} typedef(s) missing")
    print()
    print("Next steps:")
    print("  1. Review aa_patch.h")
    print("  2. Add MISSING defines to aa.h (most are machine-specific")
    print("     constants that will never be reached for PIPBUG — value 0 is fine)")
    print("  3. Check DIFFERENT defines — WRAPMEM, GET_RR, BRANCHCODE, PAGE etc.")
    print("     are critical; colour constants (RED/BLUE etc.) don't matter")
    print("  4. Rebuild:  gcc -Wall -O2 -DGAMER -o pipbug_wrap pipbug_wrap.c")


if __name__ == '__main__':
    main()
