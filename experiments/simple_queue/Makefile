
# Compiler and linker options
CC        = gcc
CFLAGS    = -c -v
LDFLAGS   =

# User directories to search
BUILDDIR  = .
INCDIR    = .
SRCDIR    = .

HOMEDIR   = #/home/pi

COMMONINCDIR = ../common/include
COMMONSRCDIR = ../common/src

# 3rd Party library directories to search
LIBDIR    = /usr/local/lib
LIBINCDIR = 

# List sources, object files and libraries to be used
SOURCES   = $(wildcard $(SRCDIR)/*.c )
OBJECTS   = $(patsubst $(SRCDIR)/%.c, $(BUILDDIR)/%.o, $(SOURCES))
LIBRARIES = m #wiringPi wiringPiDev pthread 

# Create command line arguments
LIBCMD    = $(addprefix -l,$(LIBRARIES))
LIBDIRCMD = $(addprefix -L,$(LIBDIR))
INCCMD    = $(addprefix -I,$(INCDIR) $(LIBINCDIR)) 

# The final executable name
TARGET    = bitmanip

# The rules

all: $(SOURCES) $(TARGET)
    
$(BUILDDIR)/%.o: $(SRCDIR)/%.c
	$(CC) $(CFLAGS) $(INCCMD) $< -o $@

$(TARGET): $(OBJECTS) 
	$(CC) $(LDFLAGS) $(OBJECTS) $(LIBCMD) -lm $(LIBDIRCMD) -o $@


