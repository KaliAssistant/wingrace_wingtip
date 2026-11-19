;==============================================================================
; wingtip_ctrl.asm  -  stand-alone RIGHT wingtip light controller
;
; Copyright (c) 2026 KaliAssistant <work.kaliassistant.github@gmail.com>
; Copyright (c) 2026 HyperPROBE Labs. <hyperprobelabs@proton.me>
;
; MCU    : ATtiny85-20SUR, 5 V, internal RC oscillator 8 MHz (CKDIV8 OFF)
; Signal : PB0, rx-only soft UART, 9600 8N1, idle HIGH (add ~10k pull-up)
; Outputs: PB2 NAV, PB3 BAKL, PB4 STB_F, PB1 STB_B   (active HIGH)
;          PB5 stays RESET, so ISP keeps working
;
; FUSES  : lfuse = 0xE2  (8 MHz RC, no CKDIV8)
;          hfuse = 0xDF  (default, RESET pin + SPIEN kept -> ISP works)
;          ISP note: PB0(MOSI)/PB1(MISO)/PB2(SCK) are shared with SIG/STB_B/NAV,
;          so lights may flicker while programming; disconnect the host's SIG
;          driver while flashing.
;
; STROBES: each TRIG = ONE flash of the stored high time (no auto-repeat).
;
; TIMING : STB_F pulse timer = Timer0 CTC, 1 ms tick  (ISR, non-blocking)
;          STB_B pulse timer = Timer1 CTC, 1 ms tick  (ISR, non-blocking)
;          UART RX           = polled in main, cycle-counted delays
;          Pulses < 1 ms are generated blocking with interrupts off (<=500 us)
;
; HIGH-TIME CODE (byte between 0xD4/0xD5 and 0xE4/0xE5):
;          0x00 = 50 ms (default)     0x01 = 1 us     0x02 = 10 us
;          0x03 = 50 us               0x04 = 100 us   0x05 = 500 us
;          0x06..0xFF = 1..250 ms     (ms = code - 5)
;
; FLASH MAP (words):
;          0x0000-0x0DFF  application (this file, uses ~0.4 KB)
;          0x0E00-0x0FFF  reserved for a future bootloader (see BOOT_*)
;==============================================================================

.include "tn85def.inc"

;------------------------------------------------------------------------------
; Configuration
;------------------------------------------------------------------------------
.equ F_CPU        = 8000000
.equ BAUD         = 9600

; Signed trim added to the factory OSCCAL (0xFD = -3, 0x02 = +2 ...).
; The RC osc is factory-calibrated at 3 V; at 5 V it runs a bit fast.
; Measure your baud error and trim. Don't cross the 0x7F/0x80 boundary.
.equ OSCCAL_ADJ   = 0x00

; Pins
.equ PIN_SIG      = PB0
.equ PIN_NAV      = PB2
.equ PIN_BAKL     = PB3
.equ PIN_STBF     = PB4
.equ PIN_STBB     = PB1

.equ OUT_MASK     = (1<<PIN_NAV)|(1<<PIN_BAKL)|(1<<PIN_STBF)|(1<<PIN_STBB)
; pull-up on the signal pin (and on PB1/spare if it is not an output), outputs start LOW
.equ PORTB_INIT   = ((1<<PIN_SIG)|(1<<PB1)) & (0xFF ^ OUT_MASK)

; Protocol
.equ SIG_NAV_ON     = 0xA2
.equ SIG_NAV_OFF    = 0xB2
.equ SIG_BAKL_ON    = 0xA3
.equ SIG_BAKL_OFF   = 0xB3
.equ SIG_STBF_ON    = 0xA4
.equ SIG_STBF_OFF   = 0xB4
.equ SIG_STBF_TRIG  = 0xC4
.equ SIG_STBF_HT    = 0xD4
.equ SIG_STBF_HT_END= 0xE4
.equ SIG_STBB_ON    = 0xA5
.equ SIG_STBB_OFF   = 0xB5
.equ SIG_STBB_TRIG  = 0xC5
.equ SIG_STBB_HT    = 0xD5
.equ SIG_STBB_HT_END= 0xE5

.equ DEFAULT_CODE   = 55           ; code 0x00 -> 50 ms  (55 - 5)

; Reserved for future firmware upgrade (0xF0..0xFF are reserved opcodes,
; unknown bytes are silently ignored so new commands stay backward compatible)
.equ BOOT_PRESENT = 1              ; 1 = wingtip_boot.asm is installed at BOOT_ADDR (set 0 without it)
.equ SIG_BOOT     = 0xF0           ; 0xF0, BOOT_MAGIC1, BOOT_MAGIC2 -> bootloader
.equ BOOT_MAGIC1  = 0xB0
.equ BOOT_MAGIC2  = 0x07
.equ BOOT_ADDR    = 0x0E00         ; word address

; Soft-UART timing (tuned for 8 MHz / 9600, recompute for other values)
.equ BIT_CYC      = F_CPU/BAUD                 ; 833 cycles per bit
.equ DLY_BIT      = (BIT_CYC - 15)/4           ; per-bit delay loop count
.equ DLY_HALF     = (BIT_CYC/2 - 6)/4          ; start-bit centre check
.equ TIMEOUT_OUT  = 2                          ; 2 * 65536 * 6 cyc ~= 98 ms

; 1 ms tick: CK/64, 125 counts
.equ T_TOP        = (F_CPU/64/1000) - 1        ; 124

; flag bits
.equ KEEP_F       = 0
.equ KEEP_B       = 1

;------------------------------------------------------------------------------
; Registers
;------------------------------------------------------------------------------
.def isr_sreg = r2                 ; SREG save, ISRs only
.def cntF     = r4                 ; STB_F remaining ms (0 = idle)
.def cntB     = r5                 ; STB_B remaining ms (0 = idle)
.def flags    = r20                ; KEEP_F / KEEP_B
.def hiF      = r22                ; STB_F high-time code
.def hiB      = r23                ; STB_B high-time code
; r16..r19 scratch, r24:r25 delay counter, r30:r31 (Z) table pointer

;------------------------------------------------------------------------------
; Vectors (ATtiny85: one word per vector)
;------------------------------------------------------------------------------
.cseg
.org 0x0000
    rjmp RESET                     ; 0x00 RESET   (keep as plain rjmp: a
                                   ;      bootloader will patch this word)
    reti                           ; 0x01 INT0
    reti                           ; 0x02 PCINT0
    rjmp T1_COMPA_ISR              ; 0x03 TIMER1 COMPA -> STB_B timer
    reti                           ; 0x04 TIMER1 OVF
    reti                           ; 0x05 TIMER0 OVF
    reti                           ; 0x06 EE_RDY
    reti                           ; 0x07 ANA_COMP
    reti                           ; 0x08 ADC
    reti                           ; 0x09 TIMER1 COMPB
    rjmp T0_COMPA_ISR              ; 0x0A TIMER0 COMPA -> STB_F timer
    reti                           ; 0x0B TIMER0 COMPB
    reti                           ; 0x0C WDT
    reti                           ; 0x0D USI_START
    reti                           ; 0x0E USI_OVF

;------------------------------------------------------------------------------
; ISRs: 1 ms tick, count down the running strobe pulse, drop the pin at 0.
; (a pulse is only ever started when KEEP is off, so "expire" = pin low)
;------------------------------------------------------------------------------
T0_COMPA_ISR:
    in   isr_sreg, SREG
    tst  cntF
    breq t0_done
    dec  cntF
    brne t0_done
    cbi  PORTB, PIN_STBF
t0_done:
    out  SREG, isr_sreg
    reti

T1_COMPA_ISR:
    in   isr_sreg, SREG
    tst  cntB
    breq t1_done
    dec  cntB
    brne t1_done
    cbi  PORTB, PIN_STBB
t1_done:
    out  SREG, isr_sreg
    reti

;------------------------------------------------------------------------------
; Reset / init
;------------------------------------------------------------------------------
RESET:
    cli
    ldi  r16, low(RAMEND)
    out  SPL, r16
    ldi  r16, high(RAMEND)
    out  SPH, r16

    in   r16, OSCCAL               ; trim the RC oscillator (see OSCCAL_ADJ)
    ldi  r17, OSCCAL_ADJ
    add  r16, r17
    out  OSCCAL, r16

    ldi  r16, PORTB_INIT           ; outputs low, pull-ups on SIG + spare pin
    out  PORTB, r16
    ldi  r16, OUT_MASK
    out  DDRB, r16

    sbi  ACSR, ACD                 ; power savings: comparator, ADC, USI off
    ldi  r16, (1<<PRADC)|(1<<PRUSI)
    out  PRR, r16

    clr  cntF
    clr  cntB
    clr  flags
    clr  hiF                       ; 0x00 = default 50 ms
    clr  hiB

    ; Timer0: CTC, CK/64, 1 ms  (STB_F)
    ldi  r16, (1<<WGM01)
    out  TCCR0A, r16
    ldi  r16, T_TOP
    out  OCR0A, r16
    ldi  r16, (1<<CS01)|(1<<CS00)
    out  TCCR0B, r16

    ; Timer1: CTC on OCR1C, CK/64, 1 ms  (STB_B)
    ldi  r16, T_TOP
    out  OCR1C, r16
    out  OCR1A, r16
    ldi  r16, (1<<CTC1)|(1<<CS12)|(1<<CS11)|(1<<CS10)
    out  TCCR1, r16

    ldi  r16, (1<<OCIE0A)|(1<<OCIE1A)
    out  TIMSK, r16
    sei

;------------------------------------------------------------------------------
; Main loop: wait for a byte, dispatch
;------------------------------------------------------------------------------
main:
    rcall rx_byte                  ; r16 = byte, C=1 on framing error
    brcs  main

    cpi   r16, SIG_NAV_ON
    breq  h_nav_on
    cpi   r16, SIG_NAV_OFF
    breq  h_nav_off
    cpi   r16, SIG_BAKL_ON
    breq  h_bakl_on
    cpi   r16, SIG_BAKL_OFF
    breq  h_bakl_off
    cpi   r16, SIG_STBF_ON
    breq  h_stbf_on
    cpi   r16, SIG_STBF_OFF
    breq  h_stbf_off
    cpi   r16, SIG_STBF_TRIG
    breq  h_stbf_trig
    cpi   r16, SIG_STBF_HT
    breq  h_stbf_ht
    cpi   r16, SIG_STBB_ON
    breq  h_stbb_on
    cpi   r16, SIG_STBB_OFF
    breq  h_stbb_off
    cpi   r16, SIG_STBB_TRIG
    breq  h_stbb_trig
    cpi   r16, SIG_STBB_HT
    breq  h_stbb_ht
.if BOOT_PRESENT
    cpi   r16, SIG_BOOT
    breq  h_boot
.endif
    rjmp  main                     ; unknown / reserved / stray E4,E5 -> ignore

h_nav_on:
    sbi   PORTB, PIN_NAV
    rjmp  main
h_nav_off:
    cbi   PORTB, PIN_NAV
    rjmp  main
h_bakl_on:
    sbi   PORTB, PIN_BAKL
    rjmp  main
h_bakl_off:
    cbi   PORTB, PIN_BAKL
    rjmp  main

h_stbf_on:                         ; steady on, cancels a running pulse
    clr   cntF
    sbr   flags, (1<<KEEP_F)
    sbi   PORTB, PIN_STBF
    rjmp  main
h_stbf_off:
    clr   cntF
    cbr   flags, (1<<KEEP_F)
    cbi   PORTB, PIN_STBF
    rjmp  main
h_stbf_trig:
    rcall blink_f
    rjmp  main
h_stbf_ht:
    rcall set_hi_f
    rjmp  main

h_stbb_on:
    clr   cntB
    sbr   flags, (1<<KEEP_B)
    sbi   PORTB, PIN_STBB
    rjmp  main
h_stbb_off:
    clr   cntB
    cbr   flags, (1<<KEEP_B)
    cbi   PORTB, PIN_STBB
    rjmp  main
h_stbb_trig:
    rcall blink_b
    rjmp  main
h_stbb_ht:
    rcall set_hi_b
    rjmp  main

.if BOOT_PRESENT
h_boot:
    rcall boot_cmd                 ; returns only if magic did not match
    rjmp  main
.endif

;------------------------------------------------------------------------------
; Blink triggers.  Ignored while the strobe is in KEEP (steady on).
;   code 0x00        -> 50 ms
;   code 0x01..0x05  -> blocking us pulse (interrupts off, <= 500 us)
;   code 0x06..0xFF  -> (code-5) ms, timed by the channel's 1 ms ISR
;------------------------------------------------------------------------------
blink_f:
    sbrc  flags, KEEP_F
    ret
    clr   cntF                     ; restart / cancel a running pulse
    mov   r16, hiF
    ldi   r18, (1<<PIN_STBF)
    tst   r16
    brne  bf_1
    ldi   r16, DEFAULT_CODE
bf_1:
    cpi   r16, 6
    brlo  bf_us
    subi  r16, 5                   ; ms = code - 5
    cli
    ldi   r17, (1<<OCF0A)          ; clear stale flag, restart the 1 ms tick
    out   TIFR, r17
    clr   r17
    out   TCNT0, r17
    sbi   PORTB, PIN_STBF
    mov   cntF, r16
    sei
    ret
bf_us:
    rjmp  pulse_us                 ; tail call, returns to our caller

blink_b:
    sbrc  flags, KEEP_B
    ret
    clr   cntB
    mov   r16, hiB
    ldi   r18, (1<<PIN_STBB)
    tst   r16
    brne  bb_1
    ldi   r16, DEFAULT_CODE
bb_1:
    cpi   r16, 6
    brlo  bb_us
    subi  r16, 5
    cli
    ldi   r17, (1<<OCF1A)
    out   TIFR, r17
    clr   r17
    out   TCNT1, r17
    sbi   PORTB, PIN_STBB
    mov   cntB, r16
    sei
    ret
bb_us:
    rjmp  pulse_us

;------------------------------------------------------------------------------
; pulse_us : blocking sub-ms pulse, interrupts off.
;   in : r16 = 1..5 (see table), r18 = pin mask
;   width = 4*N cycles exactly (1 us = 8 cycles @ 8 MHz) from table
;------------------------------------------------------------------------------
pulse_us:
    ldi   ZL, low(us_table*2)
    ldi   ZH, high(us_table*2)
    dec   r16
    lsl   r16
    add   ZL, r16
    clr   r17
    adc   ZH, r17
    lpm   r24, Z+
    lpm   r25, Z
    cli
    in    r17, PORTB               ; snapshot (ISRs are blocked now)
    mov   r19, r17
    or    r17, r18                 ; pin high value
    com   r18
    and   r19, r18                 ; pin low value
    out   PORTB, r17               ; ---- pulse start
us_loop:
    sbiw  r24, 1                   ; N iterations = 4N-1 cycles
    brne  us_loop
    out   PORTB, r19               ; ---- pulse end   (width = 4N cycles)
    sei
    ret

us_table:                          ; N for 1 us, 10 us, 50 us, 100 us, 500 us
    .dw   2, 20, 100, 200, 1000

;------------------------------------------------------------------------------
; Set high time:  0xD4 <value> 0xE4   (resp. 0xD5 <value> 0xE5)
; Each following byte must arrive within ~100 ms, otherwise the whole
; command is dropped and the old value is kept.
;------------------------------------------------------------------------------
set_hi_f:
    ldi   r19, SIG_STBF_HT_END
    rcall rx_set_hi
    brcs  sf_ret
    mov   hiF, r18
sf_ret:
    ret

set_hi_b:
    ldi   r19, SIG_STBB_HT_END
    rcall rx_set_hi
    brcs  sb_ret
    mov   hiB, r18
sb_ret:
    ret

; in : r19 = expected terminator
; out: r18 = value, C=0 ok / C=1 aborted
rx_set_hi:
    rcall rx_byte_to
    brcs  rsh_ret
    mov   r18, r16
    rcall rx_byte_to
    brcs  rsh_ret
    cp    r16, r19
    brne  rsh_bad
    clc
    ret
rsh_bad:
    sec
rsh_ret:
    ret

;------------------------------------------------------------------------------
; Future firmware upgrade hook (disabled while BOOT_PRESENT = 0)
; Bootloader entry: 0xF0, BOOT_MAGIC1, BOOT_MAGIC2.
; The loader is wingtip_boot.asm (one-wire half-duplex, 9600 8N1). The host
; tool (blflash) moves word 0 (this file's rjmp RESET) to word 0x0DFF and
; patches word 0 to point at the loader, so keep word 0 a plain rjmp and keep
; the application below 0x0DFF words.
;------------------------------------------------------------------------------
.if BOOT_PRESENT
boot_cmd:
    rcall rx_byte_to
    brcs  boot_ret
    cpi   r16, BOOT_MAGIC1
    brne  boot_ret
    rcall rx_byte_to
    brcs  boot_ret
    cpi   r16, BOOT_MAGIC2
    brne  boot_ret
    cli
    ldi   ZL, low(BOOT_ADDR + 1)   ; +1 = warm entry (no OSCCAL re-trim)
    ldi   ZH, high(BOOT_ADDR + 1)
    ijmp
boot_ret:
    ret
.endif

;------------------------------------------------------------------------------
; Soft UART receiver, 9600 8N1, LSB first, polled.
;   rx_byte    : wait forever for a start bit
;   rx_byte_to : wait up to ~100 ms for a start bit (C=1 on timeout)
;   out : r16 = byte, C=0 ok ; C=1 timeout / glitch / framing error
;   clobbers r16, r17, r24, r25
;------------------------------------------------------------------------------
rx_byte_to:
    clr   r24
    clr   r25
    ldi   r17, TIMEOUT_OUT
rxt_wait:
    sbis  PINB, PIN_SIG            ; line high (idle) -> skip
    rjmp  rx_start                 ; line low -> start bit
    sbiw  r24, 1
    brne  rxt_wait
    dec   r17
    brne  rxt_wait
    sec                            ; timeout
    ret

rx_byte:
    sbic  PINB, PIN_SIG            ; line low -> start bit, fall through
    rjmp  rx_byte

rx_start:
    ldi   r24, low(DLY_HALF)       ; go to the middle of the start bit
    ldi   r25, high(DLY_HALF)
    rcall dly
    sbic  PINB, PIN_SIG            ; still low? (skip next if low)
    rjmp  rx_glitch                ; high -> was just a glitch
    ldi   r17, 8
rx_bit:
    ldi   r24, low(DLY_BIT)        ; one bit time -> middle of next bit
    ldi   r25, high(DLY_BIT)
    rcall dly
    clc
    sbic  PINB, PIN_SIG
    sec                            ; line high -> 1
    ror   r16                      ; LSB first
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
    sbis  PINB, PIN_SIG            ; wait until the line is idle again
    rjmp  rx_wait_hi
rx_glitch:
    sec
    ret

dly:                               ; 4*N-1 cycles + call/ret
    sbiw  r24, 1
    brne  dly
    ret

;==============================================================================
; end of file - application must stay below BOOT_ADDR (0x0E00 words)
;==============================================================================
