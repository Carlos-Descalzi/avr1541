CC=avr-gcc
LD=avr-gcc
MCU=atmega644p
F_CPU=16000000L
OPTLEVEL=s
COMMONFLAGS=-mmcu=$(MCU)
CFLAGS=$(COMMONFLAGS) -O$(OPTLEVEL) -DF_CPU=$(F_CPU) -DUSE_SERIAL_STDIO -DDEBUGMSG -std=gnu99 -Wall
LDFLAGS=$(COMMONFLAGS) 
OBJCOPY=avr-objcopy
OBJDUMP=avr-objdump
FORMAT=ihex
OBJS=main.o ff.o sdmm.o d64.o up6502.o serial-avr644p.o sdmgr.o drivectrl.o busctrl.o via.o util.o

all: avr1541 lst

avr1541: avr1541.hex

lst: avr1541.lst

clean:
	rm -rf *.hex *.elf *.o *.lst

%.hex: %.elf
	$(OBJCOPY) -O $(FORMAT) -R .eeprom $< $@

%.lst: %.elf
	$(OBJDUMP) -h -S $< > $@

avr1541.elf: $(OBJS)
	$(LD) $(LDFLAGS) $(OBJS) -o avr1541.elf

%.o: %.c
	$(CC) $(CFLAGS) -c $<

burn: avr1541
	avrdude -V -c usbasp -p $(MCU) -U flash:w:avr1541.hex 

fuses: avr1541
	avrdude -c usbasp -p $(MCU) -U lfuse:w:0xff:m -U hfuse:w:0xdf:m 

