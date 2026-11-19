;==============================================================================
; wingtip_boot.asm  -  half-duplex UART bootloader for the wingtip controller
;
; Copyright (c) 2026 KaliAssistant <work.kaliassistant.github@gmail.com>
; Copyright (c) 2026 HyperPROBE Labs. <hyperprobelabs@proton.me>
;
; MCU    : ATtiny85, 5 V, internal RC 8 MHz (lfuse 0xE2, hfuse 0xDF)
; Wire   : PB0, one-wire half-duplex UART, 9600 8N1, idle HIGH.
;          The bootloader transmits open-drain (DDR toggled, pull-up for 1),
;          so host and chip can never fight. External ~10k pull-up recommended.
; Flash  : 0x0000-0x0DFF  application (word addresses)
;          0x0DFF         app's ORIGINAL reset vector (moved here by the host)
;          0x0E00-0x0FFF  this bootloader (512 words, ~250 used)
; Vector : word 0 is always  rjmp BOOT_ADDR (0xCDFF); the loader forces it
;          on every write of page 0, so a bad host image can't unlink it.
;
; ENTRY  : cold  (power-up/reset, via word 0): trims OSCCAL, listens for a
;                HELLO for ~0.5 s, then starts the app (if one is valid).
;          warm  (app hook "F0 B0 07"): listens for ~8 s, no OSCCAL trim.
;          No valid app (word 0x0DFF is not an rjmp) -> waits forever.
;          While in the loader the NAV LED (PB2) is lit.
;
; FRAME  host -> loader : B1 CMD ARG [64 data bytes if CMD=WRITE] CRCL CRCH
;        loader -> host : B2 STATUS [payload]                    CRCL CRCH
;        CRC16 = avr-libc _crc_ccitt_update, init 0xFFFF (reflected 0x1021 = CRC-16/MCRF4XX, check 0x6F91)
;        over CMD,ARG,DATA (host->loader) / STATUS,payload (loader->host),
;        sent low byte first. Frame bytes must follow each other within 100 ms.
;
; CMD 01 HELLO  -> payload: version, page bytes (64), app pages (112), 0
;     02 WRITE  ARG = page 0..111 (64 bytes/page): erase+write+verify
;     03 READ   ARG = page 0..127 -> payload: 64 bytes
;     04 RUN    -> checks app, ACKs, jumps to the app
; STATUS 00 ok, 01 bad CRC, 02 unknown cmd, 03 page out of range,
;        04 verify failed, 05 no valid application
;
; The loader waits ~1 ms before every reply so slow half-duplex adapters can
; turn the line around.
;==============================================================================

.include "tn85def.inc"

;------------------------------------------------------------------------------
; Configuration
;------------------------------------------------------------------------------
.equ F_CPU        = 8000000
.equ BAUD         = 9600
.equ OSCCAL_ADJ   = 0x00           ; same trim as the app (applied on cold entry only)

.equ PIN_SIG      = PB0

.equ BOOT_ADDR    = 0x0E00         ; word address, page aligned
.equ APP_ENTRY    = BOOT_ADDR - 1  ; moved original reset vector lives here
.equ PAGE_BYTES   = 64
.equ APP_PAGES    = BOOT_ADDR / 32 ; 112
.equ ALL_PAGES    = 128
.equ BL_VERSION   = 1

.equ MAGIC_H      = 0xB1
.equ MAGIC_R      = 0xB2
.equ CMD_HELLO    = 0x01
.equ CMD_WRITE    = 0x02
.equ CMD_READ     = 0x03
.equ CMD_RUN      = 0x04
.equ ST_OK        = 0x00
.equ ST_CRC       = 0x01
.equ ST_CMD       = 0x02
.equ ST_RANGE     = 0x03
.equ ST_VERIFY    = 0x04
.equ ST_NOAPP     = 0x05

.equ T_COLD       = 10             ; ~0.49 s  (outer count * 65536 * 6 cycles)
.equ T_WARM       = 160            ; ~7.9 s

; soft-UART timing @ 8 MHz / 9600 (recompute for other values)
.equ BIT_CYC      = F_CPU/BAUD                  ; 833
.equ DLY_BIT      = (BIT_CYC - 15)/4            ; rx per-bit
.equ DLY_HALF     = (BIT_CYC/2 - 6)/4           ; rx start-bit centre
.equ TX_BIT_N     = (BIT_CYC - 19)/4            ; tx per-bit
.equ TX_START_N   = (BIT_CYC - 13)/4            ; tx start bit
.equ TX_GUARD_N   = F_CPU/1000/4                ; ~1 ms turnaround guard

.equ PAGEBUF      = SRAM_START                  ; 64 byte page buffer

;------------------------------------------------------------------------------
; Registers
;------------------------------------------------------------------------------
.def cmd      = r2                 ; received command
.def arg      = r3                 ; received argument (page)
.def cnt      = r6                 ; loop counter
.def rcl      = r7                 ; received CRC low
.def crcl     = r20                ; running CRC16
.def crch     = r21
.def stat     = r22                ; rx_frame result / reply status
.def tmo      = r23                ; first-byte timeout (outer count), 0 = forever
; r16..r19 scratch, r24:r25 delay counter, X = page buffer ptr, Z = flash ptr

;------------------------------------------------------------------------------
; Word 0 (the only word of the application area the loader owns)
;------------------------------------------------------------------------------
.cseg
.org 0x0000
    rjmp BOOT_ADDR                 ; = 0xCDFF

.org BOOT_ADDR
    rjmp bl_cold                   ; BOOT_ADDR+0  cold entry
    rjmp bl_warm                   ; BOOT_ADDR+1  warm entry (used by the app)

bl_cold:
    in    r16, OSCCAL
    ldi   r17, OSCCAL_ADJ
    add   r16, r17
    out   OSCCAL, r16
    ldi   tmo, T_COLD
    rjmp  bl_init
bl_warm:
    ldi   tmo, T_WARM
bl_init:
    cli
    ldi   r16, low(RAMEND)
    out   SPL, r16
    ldi   r16, high(RAMEND)
    out   SPH, r16
    clr   r16                      ; app timers off
    out   TIMSK, r16
    out   TCCR0B, r16
    out   TCCR1, r16
    ldi   r16, 0x05                ; SIG pull-up, NAV LED on, others low
    out   PORTB, r16
    ldi   r16, 0x1E                ; PB1..PB4 outputs
    out   DDRB, r16

;------------------------------------------------------------------------------
; Entry window: wait for a valid HELLO, otherwise start the app
;------------------------------------------------------------------------------
win_loop:
    rcall rx_frame
    tst   stat
    breq  win_frame
    cpi   stat, 1
    brne  win_loop                 ; noise / bad frame: keep listening
    rcall app_valid                ; nothing heard within the window
    brcs  win_noapp
    rjmp  run_app
win_noapp:
    clr   tmo                      ; no app: wait for the host forever
    rjmp  win_loop
win_frame:
    mov   r16, cmd
    cpi   r16, CMD_HELLO
    brne  win_loop
    rcall do_hello
    clr   tmo                      ; session: no timeouts

;------------------------------------------------------------------------------
; Session loop
;------------------------------------------------------------------------------
sess_loop:
    rcall rx_frame
    tst   stat
    breq  sess_cmd
    cpi   stat, 2
    brne  sess_loop                ; noise: ignore silently
    ldi   stat, ST_CRC
    rcall reply_status
    rjmp  sess_loop
sess_cmd:
    mov   r16, cmd
    cpi   r16, CMD_HELLO
    brne  sc_1
    rcall do_hello
    rjmp  sess_loop
sc_1:
    cpi   r16, CMD_WRITE
    brne  sc_2
    rcall do_write
    rjmp  sess_loop
sc_2:
    cpi   r16, CMD_READ
    brne  sc_3
    rcall do_read
    rjmp  sess_loop
sc_3:
    cpi   r16, CMD_RUN
    brne  sc_4
    rcall do_run                   ; returns only if there is no valid app
    rjmp  sess_loop
sc_4:
    ldi   stat, ST_CMD
    rcall reply_status
    rjmp  sess_loop

;------------------------------------------------------------------------------
; Commands
;------------------------------------------------------------------------------
do_hello:
    rcall tx_begin
    ldi   r16, ST_OK
    rcall tx_crc
    ldi   r16, BL_VERSION
    rcall tx_crc
    ldi   r16, PAGE_BYTES
    rcall tx_crc
    ldi   r16, APP_PAGES
    rcall tx_crc
    ldi   r16, 0
    rcall tx_crc
    rjmp  tx_crc_end

do_write:
    mov   r16, arg
    cpi   r16, APP_PAGES
    brlo  dw_ok
    ldi   stat, ST_RANGE
    rjmp  reply_status
dw_ok:
    rcall page_write
    brcs  dw_fail
    ldi   stat, ST_OK
    rjmp  reply_status
dw_fail:
    ldi   stat, ST_VERIFY
    rjmp  reply_status

do_read:
    mov   r16, arg
    cpi   r16, ALL_PAGES
    brlo  dr_ok
    ldi   stat, ST_RANGE
    rjmp  reply_status
dr_ok:
    rcall page_to_z
    rcall tx_begin
    ldi   r16, ST_OK
    rcall tx_crc
    ldi   r16, PAGE_BYTES
    mov   cnt, r16
dr_loop:
    lpm   r16, Z+
    rcall tx_crc
    dec   cnt
    brne  dr_loop
    rjmp  tx_crc_end

do_run:
    rcall app_valid
    brcc  drun_ok
    ldi   stat, ST_NOAPP
    rjmp  reply_status
drun_ok:
    ldi   stat, ST_OK
    rcall reply_status
run_app:
    cli
    ldi   ZL, low(APP_ENTRY)
    ldi   ZH, high(APP_ENTRY)
    ijmp                           ; executes the moved reset vector

; C=0 if word APP_ENTRY is an rjmp, C=1 otherwise (erased / no app)
app_valid:
    ldi   ZL, low(APP_ENTRY*2 + 1)
    ldi   ZH, high(APP_ENTRY*2 + 1)
    lpm   r16, Z
    andi  r16, 0xF0
    cpi   r16, 0xC0
    brne  av_bad
    clc
    ret
av_bad:
    sec
    ret

;------------------------------------------------------------------------------
; Flash page handling
;------------------------------------------------------------------------------
; Z = byte address of page <arg>  (arg * 64)
page_to_z:
    mov   ZH, arg
    clr   ZL
    lsr   ZH
    ror   ZL
    lsr   ZH
    ror   ZL
    ret

; erase + write page <arg> from PAGEBUF, then read back and compare
; out: C=0 ok, C=1 verify failed
page_write:
    tst   arg
    brne  pw_go
    ldi   XL, low(PAGEBUF)         ; page 0: force word 0 = rjmp BOOT_ADDR
    ldi   XH, high(PAGEBUF)
    ldi   r16, low(0xC000 | ((BOOT_ADDR - 1) & 0x0FFF))
    st    X+, r16
    ldi   r16, high(0xC000 | ((BOOT_ADDR - 1) & 0x0FFF))
    st    X, r16
pw_go:
    rcall page_to_z
    ldi   r17, (1<<PGERS)|(1<<SELFPRGEN)
    rcall do_spm                   ; erase
    ldi   XL, low(PAGEBUF)
    ldi   XH, high(PAGEBUF)
    ldi   r16, PAGE_BYTES/2
    mov   cnt, r16
pw_fill:
    ld    r0, X+
    ld    r1, X+
    ldi   r17, (1<<SELFPRGEN)
    rcall do_spm                   ; load page buffer word
    adiw  ZL, 2
    dec   cnt
    brne  pw_fill
    subi  ZL, PAGE_BYTES           ; back to page start
    sbci  ZH, 0
    ldi   r17, (1<<PGWRT)|(1<<SELFPRGEN)
    rcall do_spm                   ; write page
    ldi   XL, low(PAGEBUF)         ; verify
    ldi   XH, high(PAGEBUF)
    ldi   r16, PAGE_BYTES
    mov   cnt, r16
pw_ver:
    lpm   r16, Z+
    ld    r17, X+
    cp    r16, r17
    brne  pw_fail
    dec   cnt
    brne  pw_ver
    clc
    ret
pw_fail:
    sec
    ret

do_spm:                            ; r17 = SPMCSR value
    out   SPMCSR, r17
    spm
dsp_w:
    in    r17, SPMCSR
    sbrc  r17, SELFPRGEN
    rjmp  dsp_w
    ret

;------------------------------------------------------------------------------
; Frame receive.  in: tmo (first-byte timeout, 0 = forever)
;   out: stat = 0 ok (cmd, arg valid; PAGEBUF filled for WRITE)
;               1 nothing heard within the timeout
;               2 complete frame, bad CRC
;               3 noise / partial frame
;------------------------------------------------------------------------------
rx_frame:
    tst   tmo
    brne  rf_to
    rcall rx_byte
    rjmp  rf_first
rf_to:
    mov   r19, tmo
    rcall rx_byte_to
rf_first:
    brcc  rf_ok0
    cpi   r16, 1                   ; 1 = timeout, 2 = glitch / framing
    brne  rf_noise
    ldi   stat, 1
    ret
rf_ok0:
    cpi   r16, MAGIC_H
    brne  rf_noise
    ser   crcl
    ser   crch
    rcall rx_b
    brcs  rf_noise
    mov   cmd, r16
    rcall rx_b
    brcs  rf_noise
    mov   arg, r16
    mov   r16, cmd
    cpi   r16, CMD_WRITE
    brne  rf_crc
    ldi   XL, low(PAGEBUF)
    ldi   XH, high(PAGEBUF)
    ldi   r16, PAGE_BYTES
    mov   cnt, r16
rf_data:
    rcall rx_b
    brcs  rf_noise
    st    X+, r16
    dec   cnt
    brne  rf_data
rf_crc:
    rcall rx_raw
    brcs  rf_noise
    mov   rcl, r16
    rcall rx_raw
    brcs  rf_noise
    cp    rcl, crcl
    brne  rf_bad
    cp    r16, crch
    brne  rf_bad
    clr   stat
    ret
rf_bad:
    ldi   stat, 2
    ret
rf_noise:
    ldi   stat, 3
    ret

rx_raw:                            ; byte with 100 ms timeout, no CRC
    ldi   r19, 2
    rjmp  rx_byte_to
rx_b:                              ; byte with 100 ms timeout, CRC updated
    rcall rx_raw
    brcs  rxb_ret
    rcall crc_upd
    clc
rxb_ret:
    ret

;------------------------------------------------------------------------------
; CRC16-CCITT step (avr-libc _crc_ccitt_update).  in: r16 = byte (kept),
; crcl:crch.  clobbers r17, r18, r19
;------------------------------------------------------------------------------
crc_upd:
    mov   r18, r16
    eor   r18, crcl                ; d = byte ^ lo(crc)
    mov   r17, r18
    swap  r17
    andi  r17, 0xF0
    eor   r18, r17                 ; d ^= d << 4
    mov   r17, r18
    clr   r19
    lsl   r17                      ; r19:r17 = d << 3
    rol   r19
    lsl   r17
    rol   r19
    lsl   r17
    rol   r19
    mov   crcl, r17                ; new lo = hi(crc) ^ (d >> 4) ^ lo(d << 3)
    eor   crcl, crch
    mov   r17, r18
    swap  r17
    andi  r17, 0x0F
    eor   crcl, r17
    eor   r19, r18                 ; new hi = d ^ hi(d << 3)
    mov   crch, r19
    ret

;------------------------------------------------------------------------------
; Reply helpers
;------------------------------------------------------------------------------
reply_status:                      ; stat = status, no payload
    rcall tx_begin
    mov   r16, stat
    rcall tx_crc
    rjmp  tx_crc_end

tx_begin:                          ; guard time, CRC init, send MAGIC_R
    ldi   r24, low(TX_GUARD_N)
    ldi   r25, high(TX_GUARD_N)
    rcall dly
    ser   crcl
    ser   crch
    ldi   r16, MAGIC_R
    rjmp  tx_byte

tx_crc:                            ; send r16, add it to the CRC
    rcall crc_upd
    rjmp  tx_byte

tx_crc_end:                        ; send the CRC, low byte first
    mov   r16, crcl
    rcall tx_byte
    mov   r16, crch
    rjmp  tx_byte

;------------------------------------------------------------------------------
; Open-drain soft UART transmit of r16, 9600 8N1.
; low = DDR bit set / PORT 0 ; high = DDR bit clear / PORT 1 (pull-up)
; clobbers r17, r18, r24, r25
;------------------------------------------------------------------------------
tx_byte:
    mov   r18, r16
    sbi   DDRB, PIN_SIG            ; start bit
    cbi   PORTB, PIN_SIG
    ldi   r17, 8
    ldi   r24, low(TX_START_N)
    ldi   r25, high(TX_START_N)
    rcall dly
tx_bit:
    ror   r18
    brcc  tx_zero
    cbi   DDRB, PIN_SIG
    sbi   PORTB, PIN_SIG
    rjmp  tx_wait
tx_zero:
    sbi   DDRB, PIN_SIG
    cbi   PORTB, PIN_SIG
    nop
tx_wait:
    ldi   r24, low(TX_BIT_N)
    ldi   r25, high(TX_BIT_N)
    rcall dly
    dec   r17
    brne  tx_bit
    cbi   DDRB, PIN_SIG            ; stop bit
    sbi   PORTB, PIN_SIG
    ldi   r24, low(TX_BIT_N)
    ldi   r25, high(TX_BIT_N)
    rcall dly
    ret

;------------------------------------------------------------------------------
; Soft UART receive, polled, 9600 8N1 LSB first (same engine as the app).
;   rx_byte    : wait forever for a start bit
;   rx_byte_to : wait for a start bit, r19 = outer count (2 ~= 100 ms)
;   out : r16 = byte, C=0 ok
;         C=1 : r16 = 1 timeout, r16 = 2 glitch / framing error
;   clobbers r16, r17, r24, r25 (and r19)
;------------------------------------------------------------------------------
rx_byte_to:
    clr   r24
    clr   r25
rxt_wait:
    sbis  PINB, PIN_SIG            ; line high (idle) -> skip
    rjmp  rx_start                 ; line low -> start bit
    sbiw  r24, 1
    brne  rxt_wait
    dec   r19
    brne  rxt_wait
    ldi   r16, 1                   ; timeout
    sec
    ret

rx_byte:
    sbic  PINB, PIN_SIG
    rjmp  rx_byte

rx_start:
    ldi   r24, low(DLY_HALF)       ; middle of the start bit
    ldi   r25, high(DLY_HALF)
    rcall dly
    sbic  PINB, PIN_SIG            ; still low?
    rjmp  rx_glitch
    ldi   r17, 8
rx_bit:
    ldi   r24, low(DLY_BIT)        ; middle of the next bit
    ldi   r25, high(DLY_BIT)
    rcall dly
    clc
    sbic  PINB, PIN_SIG
    sec
    ror   r16
    dec   r17
    brne  rx_bit

    ldi   r24, low(DLY_BIT)        ; middle of the stop bit
    ldi   r25, high(DLY_BIT)
    rcall dly
    sbis  PINB, PIN_SIG            ; stop bit must be high
    rjmp  rx_frame_err
    clc
    ret

rx_frame_err:
rx_wait_hi:
    sbis  PINB, PIN_SIG            ; wait for the line to go idle again
    rjmp  rx_wait_hi
rx_glitch:
    ldi   r16, 2
    sec
    ret

dly:                               ; 4*N-1 cycles + call/ret
    sbiw  r24, 1
    brne  dly
    ret

;==============================================================================
; end - must stay below 0x1000 words (flash end)
;==============================================================================
