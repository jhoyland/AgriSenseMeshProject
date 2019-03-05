MCU=attiny88
AVRDUDEMCU=t88
TOOLCHAIN=/c/'Program Files (x86)'/Atmel/Studio/7.0/toolchain/avr8/avr8-gnu-toolchain/bin/
CC=$(TOOLCHAIN)avr-gcc.exe
THIS_ADDRESS=0x1010
DEFINES=-DF_CPU=1000000UL -DMY_ADDRESS=$(THIS_ADDRESS)
CFLAGS=-g $(DEFINES) -Os -Wall -mcall-prologues -mmcu=$(MCU)
OBJ2HEX=$(TOOLCHAIN)avr-objcopy.exe
AVRDUDE=avrdude
PORT=COM4

# User directories to search
BUILDDIR  = ./build
INCDIR    = ./include
SRCDIR    = ./src

HOMEDIR   = /home/pi

COMMONINCDIR = ../common/include
COMMONSRCDIR = ../common/src

# 3rd Party library directories to search
LIBDIR    = 
LIBINCDIR = 

# List sources, object files and libraries to be used
SOURCES   = $(wildcard $(SRCDIR)/*.c)
OBJECTS   = $(patsubst $(SRCDIR)/%.c, $(BUILDDIR)/%.o, $(SOURCES))
COMMONSOURCES =  $(wildcard $(COMMONSRCDIR)/*.c)
COMMONOBJECTS =  $(patsubst $(COMMONSRCDIR)/%.c, $(BUILDDIR)/%.o, $(COMMONSOURCES))
LIBRARIES = 

# Create command line arguments
LIBCMD    = $(addprefix -l,$(LIBRARIES))
LIBDIRCMD = $(addprefix -L,$(LIBDIR))
INCCMD    = $(addprefix -I,$(INCDIR) $(LIBINCDIR) $(COMMONINCDIR)) 


TARGET=fieldspy-blueberry-node

all: $(TARGET).hex

$(BUILDDIR)/%.o: $(SRCDIR)/%.c
	$(CC) -c $(CFLAGS) $(INCCMD) $< -o $@

$(BUILDDIR)/%.o: $(COMMONSRCDIR)/%.c
	$(CC) -c $(CFLAGS) $(INCCMD) $< -o $@
	
$(TARGET).elf: $(OBJECTS) $(COMMONOBJECTS)
	$(CC) $(CFLAGS) $(OBJECTS) $(COMMONOBJECTS) -o $(TARGET).elf
	
$(TARGET).hex: $(TARGET).elf	 	
	$(OBJ2HEX) -R .eeprom -O ihex $(TARGET).elf $(TARGET).hex

install-rpi: all
	sudo gpio -g mode 22 out
	sudo gpio -g write 22 0
	sudo $(AVRDUDE) -p $(AVRDUDEMCU) -P /dev/spidev0.0 -c linuxspi -b 10000 -U flash:w:$(TARGET).hex
	sudo gpio -g write 22 1
	
install-pololu: all
	avrdude -c stk500v2 -P "$(PORT)" -p $(MCU) -b 10000 -U flash:w:$(TARGET).hex
	
noreset: all
	sudo $(AVRDUDE) -p $(AVRDUDEMCU) -P /dev/spidev0.0 -c linuxspi -b 10000 -U flash:w:$(TARGET).hex
	
fuse:
	sudo gpio -g mode 22 out
	sudo gpio -g write 22 0
	sudo $(AVRDUDE) -p $(AVRDUDEMCU) -P /dev/spidev0.0 -c linuxspi -b 10000 -U lfuse:w:0x22:m -U hfuse:w:0xd9:m -U efuse:w:0x07:m
	sudo gpio -g write 22 1

clean:
	rm -f *.hex *.obj *.elf *.o
