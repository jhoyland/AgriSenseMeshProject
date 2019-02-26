MCU=attiny85
AVRDUDEMCU=t85
TOOLCHAIN=/c/'Program Files (x86)'/Atmel/Studio/7.0/toolchain/avr8/avr8-gnu-toolchain/bin/
CC=avr-gcc.exe
CFLAGS=-g -Os -Wall -mcall-prologues -mmcu=$(MCU)
OBJ2HEX=/c/'Program Files (x86)'/Atmel/Studio/7.0/toolchain/avr8/avr8-gnu-toolchain/bin/avr-objcopy.exe
AVRDUDE=avrdude
TARGET=spi-master-adc1

all:
	$(CC) $(CFLAGS) $(TARGET).c -o $(TARGET)
	$(OBJ2HEX) -R .eeprom -O ihex $(TARGET) $(TARGET).hex
	rm -f $(TARGET)

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
	sudo $(AVRDUDE) -p $(AVRDUDEMCU) -P /dev/spidev0.0 -c linuxspi -b 10000 -U lfuse:w:0xe2:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m
	sudo gpio -g write 22 1

clean:
	rm -f *.hex *.obj *.o

