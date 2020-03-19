#RPi Makefiel

CC=gcc
CFLAGS=-g -v

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


TARGET=sniffer-1

# The rules

all: $(SOURCES) $(TARGET)
    
$(BUILDDIR)/%.o: $(SRCDIR)/%.c
	$(CC) -c $(CFLAGS) $(INCCMD) $< -o $@
    
$(BUILDDIR)/%.o: $(COMMONSRCDIR)/%.c
	$(CC) -c $(CFLAGS) $(INCCMD) $< -o $@

$(TARGET): $(OBJECTS) $(COMMONOBJECTS)
	$(CC) $(LDFLAGS) $(OBJECTS) $(COMMONOBJECTS) $(LIBDIRCMD) $(LIBCMD) -o $@

clean:
	rm -f *.hex *.obj *.elf *.o
