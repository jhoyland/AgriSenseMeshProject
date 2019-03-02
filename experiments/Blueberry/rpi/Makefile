

CC=gcc.exe
CFLAGS=-g $(DEFINES) -Os -Wall -mcall-prologues 

# User directories to search
BUILDDIR  = ./build
INCDIR    = ./include
SRCDIR    = ./src

HOMEDIR   = /home/pi

COMMONINCDIR = ../common/include
COMMONSRCDIR = ../common/src

# 3rd Party library directories to search
LIBDIR    = /usr/local/lib
LIBINCDIR = 

# List sources, object files and libraries to be used
SOURCES   = $(wildcard $(SRCDIR)/*.c)
OBJECTS   = $(patsubst $(SRCDIR)/%.c, $(BUILDDIR)/%.o, $(SOURCES))
COMMONSOURCES =  $(wildcard $(COMMONSRCDIR)/*.c)
COMMONOBJECTS =  $(patsubst $(COMMONSRCDIR)/%.c, $(BUILDDIR)/%.o, $(COMMONSOURCES))
LIBRARIES = wiringPi wiringPiDev pthread

# Create command line arguments
LIBCMD    = $(addprefix -l,$(LIBRARIES))
LIBDIRCMD = $(addprefix -L,$(LIBDIR))
INCCMD    = $(addprefix -I,$(INCDIR) $(LIBINCDIR) $(COMMONINCDIR)) 


TARGET=blueberry-base

all: $(TARGET).hex

# The rules

all: $(SOURCES) $(TARGET)
    
$(BUILDDIR)/%.o: $(SRCDIR)/%.c
	$(CC) $(CFLAGS) $(INCCMD) $< -o $@

$(TARGET): $(OBJECTS) 
	$(CC) $(LDFLAGS) $(OBJECTS) $(LIBDIRCMD) $(LIBCMD) -o $@

clean:
	rm -f *.hex *.obj *.elf *.o
