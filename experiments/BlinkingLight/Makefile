MCU=attiny88
AVRDUDEMCU=t88
PORT=/dev/ttyACM0
DEFINES=-DF_CPU=1000000UL 
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

install-rpi: all
	sudo gpio -g mode 22 out
	sudo gpio -g write 22 0
	sudo $(AVRDUDE) -p $(AVRDUDEMCU) -P /dev/spidev0.0 -c linuxspi -b 10000 -U flash:w:$(TARGET).hex
	sudo gpio -g write 22 1
	
install-pololu: all
	sudo avrdude -c stk500v2 -P $(PORT) -p $(MCU) -b 25000 -U flash:w:$(TARGET).hex
	
noreset: all
	sudo $(AVRDUDE) -p $(AVRDUDEMCU) -P /dev/spidev0.0 -c linuxspi -b 10000 -U flash:w:$(TARGET).hex
	
fuse:
	sudo gpio -g mode 22 out
	sudo gpio -g write 22 0
	sudo $(AVRDUDE) -p $(AVRDUDEMCU) -P /dev/spidev0.0 -c linuxspi -b 10000 -U lfuse:w:0x22:m -U hfuse:w:0xd9:m -U efuse:w:0x07:m
	sudo gpio -g write 22 1

clean:
	rm -f *.hex *.obj *.elf *.o

