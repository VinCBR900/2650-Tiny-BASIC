# Equivalency tests

`equivalency_tests.py` runs micro-programs on `sim2650.c` and validates outcomes
against expected semantics from the local WinArcadia-derived `2650.c` core.

Run:

```bash
python3 tests/equivalency_tests.py
```

Current coverage focuses on parity-sensitive areas:
- HALT stop-vs-continue mode behavior
- DAR condition code behavior
- STRZ condition code behavior
- Undefined opcodes `0x90`/`0x91`
- RRR/RRL OVF edge behavior in WC mode
