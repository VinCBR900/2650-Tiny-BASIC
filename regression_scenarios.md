# Tiny BASIC Regression Scenarios

These scenarios are intended for simulator/manual regression runs.

## Program memory correctness

1. **Sorted inserts**
   - Enter lines in this order: `30 PRINT 3`, `10 PRINT 1`, `20 PRINT 2`.
   - Run `LIST`.
   - Expected order: 10, 20, 30.

2. **Replace with longer text**
   - `100 PRINT 1`
   - `100 PRINT 12345+6789`
   - `LIST`
   - Expected: only one `100` line, with longer replacement text.

3. **Replace with shorter text**
   - `110 PRINT 12345+6789`
   - `110 PRINT 1`
   - `LIST`
   - Expected: only one `110` line, with shorter replacement text.

4. **Delete first/middle/last**
   - `10 PRINT 10`
   - `20 PRINT 20`
   - `30 PRINT 30`
   - `10` (delete first), `20` (delete middle), then re-add and delete `30` (last).
   - `LIST`
   - Expected: deletes are exact and do not corrupt neighbors.

5. **Out-of-memory insert guard**
   - Keep inserting long lines until memory is exhausted.
   - Expected: explicit `?1` out-of-memory error, interpreter remains responsive.

6. **Bad GOTO target during RUN**
   - `10 GOTO 9999`
   - `RUN`
   - Expected: explicit `?2` runtime bad-GOTO error.

7. **Malformed record defense (runtime)**
   - Corrupt program area in debugger/simulator and run.
   - Expected: explicit `?3` malformed-record error, run stops safely.

8. **Line number bounds (1..32767)**
   - `0 PRINT 1` => `?0` (rejected)
   - `32767 PRINT 1` then `LIST` => line is accepted
   - `32768 PRINT 1` => `?0` (rejected)
   - `10 GOTO 0` and `10 GOTO 32768` during `RUN` => `?0` syntax error

## Expression language

1. **Precedence**
   - `PRINT 2+3*4` => 14
   - `PRINT (2+3)*4` => 20

2. **Unary plus/minus**
   - `PRINT -5+2` => -3
   - `PRINT +5*2` => 10
   - `PRINT -(2+3)` => -5

3. **Division/multiplication**
   - `PRINT 20/5` => 4
   - `PRINT 7/3` => 2 (truncates toward zero)

4. **Overflow policy confirmation**
   - `PRINT 32767+1` wraps to -32768 (16-bit two's-complement wrap)
   - `PRINT -32768-1` wraps to 32767
