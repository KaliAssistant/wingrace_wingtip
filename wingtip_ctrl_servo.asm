;==============================================================================
; wingtip_ctrl_servo.asm  -  RIGHT wingtip light controller, serial + servo mode
;
; Copyright (c) 2026 KaliAssistant <work.kaliassistant.github@gmail.com>
; Copyright (c) 2026 HyperPROBE Labs. <hyperprobelabs@proton.me>
;
; Same as wingtip_ctrl.asm (all serial commands, bootloader hook), plus:
;
;   * After power-up the signal pin listens for a serial command for WIN_MS
;     (5 s). The first recognised command (including the bootloader entry
;     F0 B0 07) latches serial mode: the app then behaves exactly like
;     wingtip_ctrl.asm until the next power cycle.
;   * If no command arrives within the window the app switches to SERVO MODE
;     and reads an RC/servo PWM pulse (1000..2000 us, 50..400 Hz) on the same
;     pin, e.g. a flight-controller PWM / servo output:
;
;       pulse (us)      zone  scene (see scene_table)
;       < 1083            0   all off
;       1083 .. 1250      1   NAV
;       1250 .. 1417      2   NAV + BAKL
;       1417 .. 1583      3   NAV + BAKL + strobes flashing (both)
;       1583 .. 1750      4   NAV + strobes flashing (both)
;       1750 .. 1917      5   strobes steady on
;       >= 1917           6   everything steady on
;
;     Zone centres sit on 1000 / 1167 / 1333 / 1500 / 1667 / 1833 / 2000 us,
;     so a 3-position switch (1000 / 1500 / 2000) lands in the middle of a zone.
;     A new zone must be seen for CONFIRM frames in a row before it is applied.
;     Pulses outside 800..2200 us are ignored. With no valid pulse for FS_MS
;     the FAILSAFE_SCENE is shown (default: NAV + BAKL + flashing strobes).
;     Edit scene_table / the constants below to change the mapping.
;   * Flashing strobes in servo mode: one flash of AUTO_HIGH_MS every
;     AUTO_PERIOD_MS, both strobes together.
;
; MCU    : ATtiny85-20SUR, 5 V, internal RC oscillator 8 MHz (CKDIV8 OFF)
; Signal : PB0, 9600 8N1 soft UART in the first 5 s, then servo PWM input
;          (add ~10k pull-up for the UART case; a push-pull PWM output is fine)
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
.equ F_UART       = 2              ; a serial command was seen: stay in serial mode
.equ F_FS         = 3              ; failsafe scene is showing

; ---- servo mode ----
.equ WIN_MS         = 5000         ; serial command window after power-up
.equ AUTO_PERIOD_MS = 1000         ; flashing strobes: period ...
.equ AUTO_HIGH_MS   = 50           ; ... and flash length
.equ FS_MS          = 500          ; no valid pulse for this long -> failsafe
.equ CONFIRM        = 3            ; frames a new zone must persist
.equ FAILSAFE_SCENE = 0x33         ; NAV + BAKL + flashing strobes

; scene bits: 0 NAV, 1 BAKL, 2 STB_F steady, 3 STB_B steady,
;             4 STB_F flashing, 5 STB_B flashing

; Pulse width is counted in 6-cycle steps: N = us * (F_CPU/1e6) / 6
.equ NPU        = F_CPU/1000000
.equ N_MIN      = 800  * NPU / 6
.equ N_MAX      = 2200 * NPU / 6
.equ N_B1       = 1083 * NPU / 6   ; zone boundaries (us)
.equ N_B2       = 1250 * NPU / 6
.equ N_B3       = 1417 * NPU / 6
.equ N_B4       = 1583 * NPU / 6
.equ N_B5       = 1750 * NPU / 6
.equ N_B6       = 1917 * NPU / 6

;------------------------------------------------------------------------------
; Registers
;------------------------------------------------------------------------------
.def isr_sreg = r2                 ; SREG save, ISRs only
.def cntF     = r4                 ; STB_F remaining ms (0 = idle)
.def cntB     = r5                 ; STB_B remaining ms (0 = idle)
.def perL     = r6                 ; flashing-strobe period countdown (ms)
.def perH     = r7
.def tickL    = r8                 ; free-running 1 ms counter
.def tickH    = r9
.def amode    = r10                ; bit0 STB_F flashing, bit1 STB_B flashing
.def lastL    = r12                ; tick of the last valid pulse
.def lastH    = r13
.def cand     = r14                ; candidate zone
.def candcnt  = r15                ; frames the candidate has persisted
.def flags    = r20                ; KEEP_F / KEEP_B / F_UART / F_FS
.def zone_cur = r21                ; applied zone (0xFF = none)
.def hiF      = r22                ; STB_F high-time code
.def hiB      = r23                ; STB_B high-time code
; r16..r19 scratch, r24:r25 delay counter, r30:r31 (Z) table pointer,
; XL (r26) = ISR scratch

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
    inc  tickL                     ; 1 ms free-running counter
    brne t0_a
    inc  tickH
t0_a:
    tst  cntF
    breq t0_b
    dec  cntF
    brne t0_b
    cbi  PORTB, PIN_STBF
t0_b:
    tst  amode                     ; servo mode: flashing strobes
    breq t0_done
    ldi  XL, 1
    sub  perL, XL
    ldi  XL, 0
    sbc  perH, XL
    brne t0_done                   ; period not over yet
    ldi  XL, low(AUTO_PERIOD_MS)
    mov  perL, XL
    ldi  XL, high(AUTO_PERIOD_MS)
    mov  perH, XL
    ldi  XL, AUTO_HIGH_MS
    sbrs amode, 0
    rjmp t0_nof
    sbi  PORTB, PIN_STBF
    mov  cntF, XL
t0_nof:
    sbrs amode, 1
    rjmp t0_done
    sbi  PORTB, PIN_STBB
    mov  cntB, XL
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
    clr  perL
    clr  perH
    clr  tickL
    clr  tickH
    clr  amode
    clr  lastL
    clr  lastH
    clr  cand
    clr  candcnt
    ldi  zone_cur, 0xFF

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
    sbrc  flags, F_UART
    rjmp  main_uart
    rcall read_tick                ; still inside the serial command window?
    cpi   r16, low(WIN_MS)
    sbci  r17, high(WIN_MS)
    brlo  main_win
    rjmp  servo_mode               ; no command arrived -> servo (PWM) mode
main_win:
    rcall rx_byte_to               ; <= ~100 ms, then the window is re-checked
    rjmp  main_chk
main_uart:
    rcall rx_byte                  ; serial mode: block forever
main_chk:
    brcs  main                     ; r16 = byte, C=1 on timeout / framing error

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
    rjmp  cmd_done
h_nav_off:
    cbi   PORTB, PIN_NAV
    rjmp  cmd_done
h_bakl_on:
    sbi   PORTB, PIN_BAKL
    rjmp  cmd_done
h_bakl_off:
    cbi   PORTB, PIN_BAKL
    rjmp  cmd_done

h_stbf_on:                         ; steady on, cancels a running pulse
    clr   cntF
    sbr   flags, (1<<KEEP_F)
    sbi   PORTB, PIN_STBF
    rjmp  cmd_done
h_stbf_off:
    clr   cntF
    cbr   flags, (1<<KEEP_F)
    cbi   PORTB, PIN_STBF
    rjmp  cmd_done
h_stbf_trig:
    rcall blink_f
    rjmp  cmd_done
h_stbf_ht:
    rcall set_hi_f
    rjmp  cmd_done

h_stbb_on:
    clr   cntB
    sbr   flags, (1<<KEEP_B)
    sbi   PORTB, PIN_STBB
    rjmp  cmd_done
h_stbb_off:
    clr   cntB
    cbr   flags, (1<<KEEP_B)
    cbi   PORTB, PIN_STBB
    rjmp  cmd_done
h_stbb_trig:
    rcall blink_b
    rjmp  cmd_done
h_stbb_ht:
    rcall set_hi_b
    rjmp  cmd_done

.if BOOT_PRESENT
h_boot:
    rcall boot_cmd                 ; returns only if magic did not match
    rjmp  cmd_done
.endif

cmd_done:                          ; a command was handled: serial mode for good
    sbr   flags, (1<<F_UART)
    rjmp  main

;------------------------------------------------------------------------------
; SERVO MODE
;------------------------------------------------------------------------------
servo_mode:
    sbr   flags, (1<<F_FS)         ; show the failsafe scene until pulses arrive
    ldi   r16, FAILSAFE_SCENE
    rcall apply_scene
    rcall read_tick
    mov   lastL, r16
    mov   lastH, r17
sv_loop:
    rcall measure_pulse            ; r25:r24 = width, C=1 no pulse
    brcs  sv_fs
    rcall classify                 ; r18 = zone 0..6, C=1 out of range
    brcs  sv_fs
    rcall read_tick
    mov   lastL, r16
    mov   lastH, r17
    cp    r18, cand
    breq  sv_same
    mov   cand, r18                ; new candidate: start counting frames
    ldi   r16, 1
    mov   candcnt, r16
    rjmp  sv_fs
sv_same:
    mov   r16, candcnt
    cpi   r16, 255
    breq  sv_cnt
    inc   candcnt
    inc   r16
sv_cnt:
    cpi   r16, CONFIRM
    brlo  sv_fs
    cp    cand, zone_cur
    breq  sv_fs                    ; already applied
    mov   zone_cur, cand
    cbr   flags, (1<<F_FS)
    ldi   ZL, low(scene_table*2)
    ldi   ZH, high(scene_table*2)
    add   ZL, cand
    clr   r17
    adc   ZH, r17
    lpm   r16, Z
    rcall apply_scene
sv_fs:
    rcall read_tick                ; link lost?
    sub   r16, lastL
    sbc   r17, lastH
    cpi   r16, low(FS_MS)
    sbci  r17, high(FS_MS)
    brlo  sv_loop
    sbrc  flags, F_FS
    rjmp  sv_loop                  ; failsafe already showing
    sbr   flags, (1<<F_FS)
    ldi   r16, FAILSAFE_SCENE
    rcall apply_scene
    ldi   r16, 0xFF
    mov   zone_cur, r16            ; re-apply the real zone when pulses return
    rjmp  sv_loop

; r17:r16 = 1 ms tick counter, read atomically
read_tick:
    cli
    mov   r16, tickL
    mov   r17, tickH
    sei
    ret

; Measure one positive pulse on PB0.
; out: C=0 r25:r24 = width in 6-cycle steps ; C=1 no pulse within ~50 ms
measure_pulse:
    clr   r24
    clr   r25
mp_lo:                             ; first wait for the line to be LOW
    sbis  PINB, PIN_SIG
    rjmp  mp_rise
    sbiw  r24, 1
    brne  mp_lo
    sec                            ; stuck high / idle high: no pulse
    ret
mp_rise:
    clr   r24
    clr   r25
mp_r:                              ; wait for the rising edge
    sbic  PINB, PIN_SIG
    rjmp  mp_meas
    sbiw  r24, 1
    brne  mp_r
    sec
    ret
mp_meas:
    clr   r24
    clr   r25
mp_m:                              ; 6 cycles per step while the line is high
    adiw  r24, 1
    brcs  mp_stuck
    sbic  PINB, PIN_SIG
    rjmp  mp_m
    clc
    ret
mp_stuck:
    sec
    ret

; in : r25:r24 = width ; out: r18 = zone 0..6, C=1 if outside N_MIN..N_MAX
classify:
    cpi   r24, low(N_MIN)
    ldi   r19, high(N_MIN)
    cpc   r25, r19
    brlo  cl_bad
    cpi   r24, low(N_MAX + 1)
    ldi   r19, high(N_MAX + 1)
    cpc   r25, r19
    brsh  cl_bad
    ldi   r18, 0
    cpi   r24, low(N_B1)
    ldi   r19, high(N_B1)
    cpc   r25, r19
    brlo  cl_ok
    inc   r18
    cpi   r24, low(N_B2)
    ldi   r19, high(N_B2)
    cpc   r25, r19
    brlo  cl_ok
    inc   r18
    cpi   r24, low(N_B3)
    ldi   r19, high(N_B3)
    cpc   r25, r19
    brlo  cl_ok
    inc   r18
    cpi   r24, low(N_B4)
    ldi   r19, high(N_B4)
    cpc   r25, r19
    brlo  cl_ok
    inc   r18
    cpi   r24, low(N_B5)
    ldi   r19, high(N_B5)
    cpc   r25, r19
    brlo  cl_ok
    inc   r18
    cpi   r24, low(N_B6)
    ldi   r19, high(N_B6)
    cpc   r25, r19
    brlo  cl_ok
    inc   r18
cl_ok:
    clc
    ret
cl_bad:
    sec
    ret

; Apply a scene (r16, bit layout above). clobbers r17, r18, r19
apply_scene:
    mov   r18, r16
    sbrc  r18, 0
    sbi   PORTB, PIN_NAV
    sbrs  r18, 0
    cbi   PORTB, PIN_NAV
    sbrc  r18, 1
    sbi   PORTB, PIN_BAKL
    sbrs  r18, 1
    cbi   PORTB, PIN_BAKL
    clr   amode                    ; stop the flasher while we change things
    clr   cntF
    clr   cntB
    cbi   PORTB, PIN_STBF
    cbi   PORTB, PIN_STBB
    sbrc  r18, 2
    sbi   PORTB, PIN_STBF
    sbrc  r18, 3
    sbi   PORTB, PIN_STBB
    mov   r17, r18
    swap  r17
    andi  r17, 0x03                ; bits 4,5 -> flashing front / back
    ldi   r19, 1
    mov   perL, r19                ; first flash right away
    clr   perH
    mov   amode, r17
    ret

; zone -> scene
scene_table:
    .db   0x00, 0x01, 0x03, 0x33, 0x31, 0x0C, 0x0F, 0x00

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
    clr   r24
    clr   r25
rx_wait_hi:                        ; wait (max ~50 ms) until the line is idle again
    sbic  PINB, PIN_SIG
    rjmp  rx_glitch                ; high -> done
    sbiw  r24, 1
    brne  rx_wait_hi
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
