MCU=atmega328p
AVRDUDEMCU=m328p
DEFINES=-DF_CPU=1000000UL -DMY_ADDRESS=$(THIS_ADDRESS) -DASMP_PANID=0xF122
CC=/usr/bin/avr-gcc
CFLAGS=-g $(DEFINES) -Os -Wall -mcall-prologues -mmcu=$(MCU)
OBJ2HEX=/usr/bin/avr-objcopy
AVRDUDE=/usr/local/bin/avrdude

SOURCES = $(wildcard *.c)
OBJECTS = $(patsubst %.c, %.o, $(SOURCES))
 
TARGET=mrfe1

all: $(TARGET).hex

%.o: %.c
	$(CC) -c $(CFLAGS) $< -o $@
	
$(TARGET).elf: $(OBJECTS)
	$(CC) $(CFLAGS) $(OBJECTS) -o $(TARGET).elf
	
$(TARGET).hex: $(TARGET).elf	
	$(OBJ2HEX) -R .eeprom -O ihex $(TARGET).elf $(TARGET).hex

install: all
	sudo gpio -g mode 22 out
	sudo gpio -g write 22 0
	sudo $(AVRDUDE) -p $(AVRDUDEMCU) -P /dev/spidev0.0 -c linuxspi -b 10000 -U flash:w:$(TARGET).hex
	sudo gpio -g write 22 1
	
noreset: all
	sudo $(AVRDUDE) -p $(AVRDUDEMCU) -P /dev/spidev0.0 -c linuxspi -b 10000 -U flash:w:$(TARGET).hex
	
fuse:
	sudo gpio -g mode 22 out
	sudo gpio -g write 22 0
	sudo $(AVRDUDE) -p $(AVRDUDEMCU) -P /dev/spidev0.0 -c linuxspi -b 10000 -U lfuse:w:0x22:m -U hfuse:w:0xd9:m -U efuse:w:0x07:m
	sudo gpio -g write 22 1

clean:
	rm -f *.hex *.obj *.elf *.o

