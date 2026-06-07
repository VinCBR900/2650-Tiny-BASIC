; ============================================================
; exercise.asm  -  Signetics 2650 assembler exercise file
; Exercises every instruction from the opcode table.
; Assemble:  asm2650 exercise.asm exercise.hex
; Check:     exercise.LST
;
; Design notes:
;   - Data patches (DATBYTE etc.) placed immediately before each
;     relative-memory group so they fall within ±63 byte reach.
;   - Absolute-memory forms use a fixed data page at $0F00.
;   - ZBSR target placed within its tight ±63 byte range.
;   - ZBRR/ZBSR: 2-byte, signed 7-bit displacement from address zero (NOT PC-relative).
; ============================================================

TESTVAL equ     $A5
MASK_HI equ     $F0
MASK_LO equ     $0F
ABSDAT  equ     $0F00           ; data page for absolute forms

        org     $0000

; ============================================================
; GROUP 1: LOAD IMMEDIATE  (LODI)  $04-$07
; ============================================================
        lodi,r0 TESTVAL         ; $04 A5
        lodi,r1 $FF             ; $05 FF
        lodi,r2 %10101010       ; $06 AA
        lodi,r3 'Z'             ; $07 5A

; ============================================================
; GROUP 2: LOAD ZERO-REG  (LODZ)  $01-$03
; ============================================================
        lodz,r1                 ; $01
        lodz,r2                 ; $02
        lodz,r3                 ; $03

; ============================================================
; GROUP 3: LOAD RELATIVE  (LODR)  $08-$0B
; data must be within ±63 bytes of instruction
; ============================================================
        lodr,r0 DAT_LODR        ; $08 xx
        lodr,r1 DAT_LODR        ; $09 xx
        lodr,r2 DAT_LODR        ; $0A xx
        lodr,r3 DAT_LODR        ; $0B xx
        bctr,un DAT_LODR+1      ; skip over inline data byte
DAT_LODR db     $55             ; data byte - within relative reach above

; ============================================================
; GROUP 4: LOAD ABSOLUTE  (LODA)  $0C-$0F
; ============================================================
        loda,r0 ABSDAT          ; $0C 00 xx  plain
        loda,r1 ABSDAT,r2       ; $0D xx xx  indexed R2
        loda,r2 ABSDAT,r3+      ; $0E xx xx  post-inc
        loda,r3 ABSDAT,r1-      ; $0F xx xx  post-dec
        loda,r0 *ABSDAT         ; $0C 80 xx  indirect

; ============================================================
; GROUP 5: STORE ZERO-REG  (STRZ)  $C1-$C3
; ============================================================
        strz,r1                 ; $C1
        strz,r2                 ; $C2
        strz,r3                 ; $C3

; ============================================================
; GROUP 6: STORE RELATIVE  (STRR)  $C8-$CB
; ============================================================
        strr,r0 DAT_STRR        ; $C8 xx
        strr,r1 DAT_STRR        ; $C9 xx
        strr,r2 DAT_STRR        ; $CA xx
        strr,r3 DAT_STRR        ; $CB xx
        bctr,un DAT_STRR+1      ; skip over inline data byte
DAT_STRR db     $00             ; scratch byte - within relative reach above

; ============================================================
; GROUP 7: STORE ABSOLUTE  (STRA)  $CC-$CF
; ============================================================
        stra,r0 ABSDAT          ; $CC 00 xx  plain
        stra,r1 ABSDAT,r2       ; $CD xx xx  indexed
        stra,r2 ABSDAT,r3+      ; $CE xx xx  post-inc
        stra,r3 ABSDAT,r1-      ; $CF xx xx  post-dec

; ============================================================
; GROUP 8: ADD  (ADDZ/ADDI/ADDR/ADDA)  $80-$8F
; ============================================================
        addz,r1                 ; $81
        addz,r2                 ; $82
        addz,r3                 ; $83
        addi,r0 TESTVAL         ; $84 A5
        addi,r1 $01             ; $85 01
        addi,r2 $02             ; $86 02
        addi,r3 $03             ; $87 03
        addr,r0 DAT_ADDR        ; $88 xx
        addr,r1 DAT_ADDR        ; $89 xx
        addr,r2 DAT_ADDR        ; $8A xx
        addr,r3 DAT_ADDR        ; $8B xx
        bctr,un DAT_ADDR+1      ; skip over inline data byte
DAT_ADDR db     $01             ; data byte - within relative reach above
        adda,r0 ABSDAT          ; $8C 00 xx
        adda,r1 ABSDAT,r2       ; $8D xx xx
        adda,r2 ABSDAT          ; $8E 00 xx
        adda,r3 ABSDAT          ; $8F 00 xx

; ============================================================
; GROUP 9: SUBTRACT  (SUBZ/SUBI/SUBR/SUBA)  $A0-$AF
; ============================================================
        subz,r1                 ; $A1
        subz,r2                 ; $A2
        subz,r3                 ; $A3
        subi,r0 TESTVAL         ; $A4 A5
        subi,r1 $01             ; $A5 01
        subi,r2 $02             ; $A6 02
        subi,r3 $03             ; $A7 03
        subr,r0 DAT_SUBR        ; $A8 xx
        subr,r1 DAT_SUBR        ; $A9 xx
        subr,r2 DAT_SUBR        ; $AA xx
        subr,r3 DAT_SUBR        ; $AB xx
        bctr,un DAT_SUBR+1      ; skip over inline data byte
DAT_SUBR db     $01
        suba,r0 ABSDAT          ; $AC 00 xx
        suba,r1 ABSDAT,r2       ; $AD xx xx
        suba,r2 ABSDAT          ; $AE 00 xx
        suba,r3 ABSDAT          ; $AF 00 xx

; ============================================================
; GROUP 10: EXCLUSIVE OR  (EORZ/EORI/EORR/EORA)  $20-$2F
; ============================================================
        eorz,r1                 ; $21
        eorz,r2                 ; $22
        eorz,r3                 ; $23
        eori,r0 TESTVAL         ; $24 A5
        eori,r1 $55             ; $25 55
        eori,r2 $AA             ; $26 AA
        eori,r3 $FF             ; $27 FF
        eorr,r0 DAT_EORR        ; $28 xx
        eorr,r1 DAT_EORR        ; $29 xx
        eorr,r2 DAT_EORR        ; $2A xx
        eorr,r3 DAT_EORR        ; $2B xx
        bctr,un DAT_EORR+1      ; skip over inline data byte
DAT_EORR db     $FF
        eora,r0 ABSDAT          ; $2C 00 xx
        eora,r1 ABSDAT,r2       ; $2D xx xx
        eora,r2 ABSDAT          ; $2E 00 xx
        eora,r3 ABSDAT          ; $2F 00 xx

; ============================================================
; GROUP 11: AND  (ANDZ/ANDI/ANDR/ANDA)  $41-$4F
; ============================================================
        andz,r1                 ; $41
        andz,r2                 ; $42
        andz,r3                 ; $43
        andi,r0 MASK_HI         ; $44 F0
        andi,r1 MASK_LO         ; $45 0F
        andi,r2 $FF             ; $46 FF
        andi,r3 $00             ; $47 00
        andr,r0 DAT_ANDR        ; $48 xx
        andr,r1 DAT_ANDR        ; $49 xx
        andr,r2 DAT_ANDR        ; $4A xx
        andr,r3 DAT_ANDR        ; $4B xx
        bctr,un DAT_ANDR+1      ; skip over inline data byte
DAT_ANDR db     $FF
        anda,r0 ABSDAT          ; $4C 00 xx
        anda,r1 ABSDAT,r2       ; $4D xx xx
        anda,r2 ABSDAT          ; $4E 00 xx
        anda,r3 ABSDAT          ; $4F 00 xx

; ============================================================
; GROUP 12: OR  (IORZ/IORI/IORR/IORA)  $60-$6F
; ============================================================
        iorz,r1                 ; $61
        iorz,r2                 ; $62
        iorz,r3                 ; $63
        iori,r0 TESTVAL         ; $64 A5
        iori,r1 $55             ; $65 55
        iori,r2 $AA             ; $66 AA
        iori,r3 $FF             ; $67 FF
        iorr,r0 DAT_IORR        ; $68 xx
        iorr,r1 DAT_IORR        ; $69 xx
        iorr,r2 DAT_IORR        ; $6A xx
        iorr,r3 DAT_IORR        ; $6B xx
        bctr,un DAT_IORR+1      ; skip over inline data byte
DAT_IORR db     $AA
        iora,r0 ABSDAT          ; $6C 00 xx
        iora,r1 ABSDAT,r2       ; $6D xx xx
        iora,r2 ABSDAT          ; $6E 00 xx
        iora,r3 ABSDAT          ; $6F 00 xx

; ============================================================
; GROUP 13: COMPARE  (COMZ/COMI/COMR/COMA)  $E0-$EF
; ============================================================
        comz,r1                 ; $E1
        comz,r2                 ; $E2
        comz,r3                 ; $E3
        comi,r0 TESTVAL         ; $E4 A5
        comi,r1 $55             ; $E5 55
        comi,r2 $AA             ; $E6 AA
        comi,r3 $FF             ; $E7 FF
        comr,r0 DAT_COMR        ; $E8 xx
        comr,r1 DAT_COMR        ; $E9 xx
        comr,r2 DAT_COMR        ; $EA xx
        comr,r3 DAT_COMR        ; $EB xx
        bctr,un DAT_COMR+1      ; skip over inline data byte
DAT_COMR db     $A5
        coma,r0 ABSDAT          ; $EC 00 xx
        coma,r1 ABSDAT,r2       ; $ED xx xx
        coma,r2 ABSDAT          ; $EE 00 xx
        coma,r3 ABSDAT          ; $EF 00 xx

; ============================================================
; GROUP 14: TEST MASK IMMEDIATE  (TMI)  $F4-$F7
; Correctly encodes mask byte. Fixed in asm2650 v1.11.
; ============================================================
        tmi,r0  $FF             ; $F4 FF
        tmi,r1  MASK_HI         ; $F5 F0
        tmi,r2  MASK_LO         ; $F6 0F
        tmi,r3  TESTVAL         ; $F7 A5

; ============================================================
; GROUP 15: ROTATE  (RRR/RRL)  $50-$53, $D0-$D3
; ============================================================
        rrr,r0                  ; $50
        rrr,r1                  ; $51
        rrr,r2                  ; $52
        rrr,r3                  ; $53
        rrl,r0                  ; $D0
        rrl,r1                  ; $D1
        rrl,r2                  ; $D2
        rrl,r3                  ; $D3

; ============================================================
; GROUP 16: DECIMAL ADJUST  (DAR)  $94-$97
; ============================================================
        dar,r0                  ; $94
        dar,r1                  ; $95
        dar,r2                  ; $96
        dar,r3                  ; $97

; ============================================================
; GROUP 17: PSU / PSL OPERATIONS
; ============================================================
        spsu                    ; $12
        spsl                    ; $13
        lpsu                    ; $92
        lpsl                    ; $93
        cpsu    $80             ; $74 80
        cpsl    $40             ; $75 40
        ppsu    $20             ; $76 20
        ppsl    $10             ; $77 10
        tpsu    $80             ; $B4 80
        tpsl    $40             ; $B5 40

; ============================================================
; GROUP 18: I/O PORTS
; ============================================================
        redc,r0                 ; $30
        redc,r1                 ; $31
        redc,r2                 ; $32
        redc,r3                 ; $33
        wrtc,r0                 ; $B0
        wrtc,r1                 ; $B1
        wrtc,r2                 ; $B2
        wrtc,r3                 ; $B3
        rede,r0                 ; $54
        rede,r1                 ; $55
        rede,r2                 ; $56
        rede,r3                 ; $57
        wrte,r0                 ; $D4
        wrte,r1                 ; $D5
        wrte,r2                 ; $D6
        wrte,r3                 ; $D7
        redd,r0                 ; $70
        redd,r1                 ; $71
        redd,r2                 ; $72
        redd,r3                 ; $73
        wrtd,r0                 ; $F0
        wrtd,r1                 ; $F1
        wrtd,r2                 ; $F2
        wrtd,r3                 ; $F3

; ============================================================
; GROUP 19: BRANCH CONDITION TRUE relative  BCTR  $18-$1B
; ============================================================
BT_EQ   bctr,eq BT_EQ          ; $18 7E  self-loop
        bctr,gt BT_EQ          ; $19 7C
        bctr,lt BT_EQ          ; $1A 7A
        bctr,un BT_EQ          ; $1B 78

; ============================================================
; GROUP 20: BRANCH CONDITION TRUE absolute  BCTA  $1C-$1F
; ============================================================
        bcta,eq $0000           ; $1C 00 00
        bcta,gt $0000           ; $1D 00 00
        bcta,lt $0000           ; $1E 00 00
        bcta,un $0000           ; $1F 00 00

; ============================================================
; GROUP 21: BRANCH CONDITION FALSE relative  BCFR  $98-$9A
;           $9B = ZBRR  (1-byte, no operand)
; ============================================================
BF_EQ   bcfr,eq BF_EQ          ; $98 7E
        bcfr,gt BF_EQ          ; $99 7C
        bcfr,lt BF_EQ          ; $9A 7A
        zbrr    $00            ; $9B 00  zero-page displacement to addr $0000

; ============================================================
; GROUP 22: BRANCH CONDITION FALSE absolute  BCFA  $9C-$9E
;           $9F = BXA
; ============================================================
        bcfa,eq $0000           ; $9C 00 00
        bcfa,gt $0000           ; $9D 00 00
        bcfa,lt $0000           ; $9E 00 00
        bxa                     ; $9F 00 00

; ============================================================
; GROUP 23: BRANCH TO SUB TRUE relative  BSTR  $38-$3B
; ============================================================
BS_EQ   bstr,eq BS_EQ          ; $38 7E
        bstr,gt BS_EQ          ; $39 7C
        bstr,lt BS_EQ          ; $3A 7A
        bstr,un BS_EQ          ; $3B 78

; ============================================================
; GROUP 24: BRANCH TO SUB TRUE absolute  BSTA  $3C-$3F
; ============================================================
        bsta,eq $0000           ; $3C 00 00
        bsta,gt $0000           ; $3D 00 00
        bsta,lt $0000           ; $3E 00 00
        bsta,un $0000           ; $3F 00 00

; ============================================================
; GROUP 25: BRANCH TO SUB FALSE relative  BSFR  $B8-$BA
;           $BB = ZBSR  (relative, tight ±63 byte range)
; ============================================================
BF2_EQ  bsfr,eq BF2_EQ         ; $B8 7E
        bsfr,gt BF2_EQ         ; $B9 7C
        bsfr,lt BF2_EQ         ; $BA 7A
        zbsr    $00            ; $BB 00  zero-page displacement (manual: -64..+63 from page zero)

; ============================================================
; GROUP 26: BRANCH TO SUB FALSE absolute  BSFA  $BC-$BE
;           $BF = BSXA
; ============================================================
        bsfa,eq $0000           ; $BC 00 00
        bsfa,gt $0000           ; $BD 00 00
        bsfa,lt $0000           ; $BE 00 00
        bsxa                    ; $BF 00 00

; ============================================================
; GROUP 27: RETURN FROM SUBROUTINE  RETE $34-$37  RETC $14-$17
; ============================================================
        rete,eq                 ; $34
        rete,gt                 ; $35
        rete,lt                 ; $36
        rete,un                 ; $37
        retc,eq                 ; $14
        retc,gt                 ; $15
        retc,lt                 ; $16
        retc,un                 ; $17

; ============================================================
; GROUP 28: BRANCH IF REG NON-ZERO relative  BRNR  $58-$5B
; ============================================================
BNZ_LBL brnr,r0 BNZ_LBL        ; $58 7E
        brnr,r1 BNZ_LBL        ; $59 7C
        brnr,r2 BNZ_LBL        ; $5A 7A
        brnr,r3 BNZ_LBL        ; $5B 78

; ============================================================
; GROUP 29: BRANCH IF REG NON-ZERO absolute  BRNA  $5C-$5F
; ============================================================
        brna,r0 $0000           ; $5C 00 00
        brna,r1 $0000           ; $5D 00 00
        brna,r2 $0000           ; $5E 00 00
        brna,r3 $0000           ; $5F 00 00

; ============================================================
; GROUP 30: BRANCH TO SUB IF REG NON-ZERO relative  BSNR  $78-$7B
; ============================================================
BSN_LBL bsnr,r0 BSN_LBL        ; $78 7E
        bsnr,r1 BSN_LBL        ; $79 7C
        bsnr,r2 BSN_LBL        ; $7A 7A
        bsnr,r3 BSN_LBL        ; $7B 78

; ============================================================
; GROUP 31: BRANCH TO SUB IF REG NON-ZERO absolute  BSNA  $7C-$7F
; ============================================================
        bsna,r0 $0000           ; $7C 00 00
        bsna,r1 $0000           ; $7D 00 00
        bsna,r2 $0000           ; $7E 00 00
        bsna,r3 $0000           ; $7F 00 00

; ============================================================
; GROUP 32: BRANCH AND INC/DEC REGISTER
;           BIRR $D8-$DB  BDRR $F8-$FB  (relative)
;           BIRA $DC-$DF  BDRA $FC-$FF  (absolute)
; ============================================================
BID_LBL birr,r0 BID_LBL        ; $D8 7E
        birr,r1 BID_LBL        ; $D9 7C
        birr,r2 BID_LBL        ; $DA 7A
        birr,r3 BID_LBL        ; $DB 78
        bdrr,r0 BID_LBL        ; $F8 xx
        bdrr,r1 BID_LBL        ; $F9 xx
        bdrr,r2 BID_LBL        ; $FA xx
        bdrr,r3 BID_LBL        ; $FB xx
        bira,r0 $0000           ; $DC 00 00
        bira,r1 $0000           ; $DD 00 00
        bira,r2 $0000           ; $DE 00 00
        bira,r3 $0000           ; $DF 00 00
        bdra,r0 $0000           ; $FC 00 00
        bdra,r1 $0000           ; $FD 00 00
        bdra,r2 $0000           ; $FE 00 00
        bdra,r3 $0000           ; $FF 00 00

; ============================================================
; GROUP 33: MISCELLANEOUS
; ============================================================
        nop                     ; $C0
        halt                    ; $40

; ============================================================
; Absolute data page
; ============================================================
        org     ABSDAT
        db      $55,$AA         ; test bytes
        dw      $1234           ; test word
        ds      8               ; scratch

        end
