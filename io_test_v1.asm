; io_test_v1.asm  -  Kowalski-style I/O smoke test
; PUTCH = WRTD,R1  (write R1 to stdout)
; GETCH = REDE,R1  (read stdin into R1)
; Sends "HI\r\n", reads one char, echoes it + "\r\n", HALTs.
;
; Run: sim2650 io_test_v1.hex          (type a char when prompted)
;      sim2650 -rx input.txt io_test_v1.hex
;
; Build: asm2650 io_test_v1.asm io_test_v1.hex

CR      EQU     $0D
LF      EQU     $0A

        ORG     $0000

RESET:
        LODI,R1 'H'
        BSTA,UN PUTCH
        LODI,R1 'I'
        BSTA,UN PUTCH
        LODI,R1 CR
        BSTA,UN PUTCH
        LODI,R1 LF
        BSTA,UN PUTCH

        BSTA,UN GETCH       ; R1 = char from stdin
        BSTA,UN PUTCH       ; echo it
        LODI,R1 CR
        BSTA,UN PUTCH
        LODI,R1 LF
        BSTA,UN PUTCH
        HALT

; PUTCH: transmit R1 — direct output via WRTD
PUTCH:
        WRTD,R1             ; write R1 to output port (sim: putchar)
        RETC,UN

; GETCH: receive into R1 — direct input via REDE
GETCH:
        REDE,R1             ; read input port into R1 (sim: getchar)
        RETC,UN

        END
