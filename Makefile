# Wingtip light controller - build + flash (Linux and macOS)
#
#   make                 build blflash and both hex files
#   make flash           flash the app through the loader (serial port auto-detected)
#   make flash ENTER=1   ... from a running app (sends the enter-loader sequence)
#   make flash PORT=/dev/cu.usbserial-1410      pin a port (only if auto-detect picks wrong)
#   make flash APP=wingtip_ctrl_servo           flash the serial + servo-mode app instead
#   make flash ECHO=1    half-duplex adapter that echoes our own bytes
#   make ports           list the serial ports blflash would probe
#   make isp             one-time: flash the loader + fuses with avrdude (ISP)
#
# Serial ports are never hard-coded: blflash probes /dev/ttyUSB*, /dev/ttyACM*
# (Linux) or /dev/cu.usbserial*, /dev/cu.usbmodem* (macOS) itself. An exported
# BLFLASH_PORT environment variable is also honoured.

UNAME_S := $(shell uname -s)

CC      ?= cc
CFLAGS  ?= -O2 -Wall -Wextra

AVRA    ?= avra
# Only needed if avra cannot find tn85def.inc by itself, e.g.
#   Debian/Ubuntu: /usr/share/avra        Homebrew: $(brew --prefix)/share/avra
AVRA_INC ?=

# which application to flash / verify: wingtip_ctrl (serial only) or
# wingtip_ctrl_servo (serial for 5 s after power-up, then servo PWM mode)
APP ?= wingtip_ctrl

PROGRAMMER ?= ch341a
AVRDUDE    ?= avrdude

# optional switches -> blflash flags (empty = not passed)
BLFLAGS = $(if $(PORT),-p $(PORT)) $(if $(ECHO),-e) $(if $(ENTER),-E) $(EXTRA)

.PHONY: all flash verify read info run ports isp clean

all: blflash wingtip_boot.hex wingtip_ctrl.hex wingtip_ctrl_servo.hex

blflash: blflash.c
	$(CC) $(CFLAGS) -o $@ $<

wingtip_%.hex: wingtip_%.asm
	$(AVRA) $(if $(AVRA_INC),-I $(AVRA_INC)) $<

flash: blflash $(APP).hex
	./blflash $(BLFLAGS) flash $(APP).hex

verify: blflash $(APP).hex
	./blflash $(BLFLAGS) verify $(APP).hex

read: blflash
	./blflash $(BLFLAGS) read dump.bin

info: blflash
	./blflash $(BLFLAGS) info

run: blflash
	./blflash $(BLFLAGS) run

ports: blflash
	./blflash -l

# one-time ISP install of the loader (keeps RESET + SPIEN: hfuse 0xDF)
isp: wingtip_boot.hex
	$(AVRDUDE) -p t85 -c $(PROGRAMMER) $(if $(ISPPORT),-P $(ISPPORT)) \
	    -U flash:w:wingtip_boot.hex:i -U lfuse:w:0xE2:m -U hfuse:w:0xDF:m

clean:
	rm -f blflash *.hex *.obj *.cof *.eep.hex dump.bin
