# Makefile for this AVR project

# make clean code Compiles the source code into hex files.
# make fuses      Program fuses
# make program    Program flash and eeprom

# make list       Create generated code listing
# make clean      Delete all generated files


# Programmer hardware settings for avrdude
# Linux:   /dev/ttyUSB0 is the first virtual serial port 
# Windows: //./COM20    is the virtual port COM20
AVRDUDE_HW = -c avr910 -P /dev/ttyUSB0 -b 115200

# The setting above does not matter when you use a graphical application to
# transfer the firmware into the microcontroller.

# Name of the program without extension
PRG = ServoController_tiny2313

# Source files, separated by space.
SRC = ServoController.c

# Microcontroller type and fuses
MCU = attiny2313
F_CPU = 4000000
LFUSE = 0xE2
HFUSE = 0xDF

# Binaries to be used
# You may add the path to them if they are not in the PATH variable.
CC      = avr-gcc
OBJCOPY = avr-objcopy
OBJDUMP = avr-objdump
AVRDUDE = avrdude

# Do we need to write Eeprom? (yes/no)
EEPROM = no

# Libraries
#LIBS = -L path/to/libraries -llibrary1 -llibrary2

# Includes
#INCLUDES = -Ipath/to/include/files

# Compiler options for all c source files
CFLAGS = -std=c99 -Wall -Os -mmcu=$(MCU) -DF_CPU=$(F_CPU) $(INCLUDES)

# Linker options 
LDFLAGS = -Wl,-Map,$(PRG).map

# Enable floating-point support in printf
#LDFLAGS += -Wl,-u,vfprintf -lprintf_flt -lm


# Collect fuse operations for avrdude
ifdef FUSE
  FUSES += -U fuse:w:$(FUSE):m
endif
ifdef LFUSE
  FUSES += -U lfuse:w:$(LFUSE):m
endif
ifdef HFUSE
  FUSES += -U hfuse:w:$(HFUSE):m
endif
ifdef EFUSE
  FUSES += -U efuse:w:$(EFUSE):m
endif
ifdef FUSE0
  FUSES += -U fuse0:w:$(FUSE0):m
endif
ifdef FUSE1
  FUSES += -U fuse1:w:$(FUSE1):m
endif
ifdef FUSE2
  FUSES += -U fuse2:w:$(FUSE2):m
endif
ifdef FUSE3
  FUSES += -U fuse3:w:$(FUSE3):m
endif
ifdef FUSE4
  FUSES += -U fuse4:w:$(FUSE4):m
endif
ifdef FUSE5
  FUSES += -U fuse5:w:$(FUSE5):m
endif
ifdef FUSE6
  FUSES += -U fuse6:w:$(FUSE6):m
endif
ifdef FUSE7
  FUSES += -U fuse7:w:$(FUSE7):m
endif

# Default sections
ifeq ($(EEPROM),yes)
all: code eeprom
else
all: code
endif

# Program code
code: $(PRG).hex

# Alias for AVR Studio
USB-IO-Modul: $(PRG).hex

# Eeprom content
eeprom: $(PRG)_eeprom.hex

# Generated code listing
list: $(PRG).lst

# Remove all generated files
clean:
	rm -f *.o $(PRG).hex $(PRG).elf $(PRG).lst $(PRG).map $(PRG)_eeprom.hex

# Program flash memory with or without eeprom
ifeq ($(EEPROM),yes)
program: code eeprom
	$(AVRDUDE) -p $(MCU) $(AVRDUDE_HW) -U flash:w:$(PRG).hex:i -U eeprom:w:$(PRG)_eeprom.hex:i
else
program: code 
	$(AVRDUDE) -p $(MCU) $(AVRDUDE_HW) -U flash:w:$(PRG).hex:i
endif

# Program fuses
fuses:
	$(AVRDUDE) -p $(MCU) $(AVRDUDE_HW) $(FUSES)

$(PRG).elf: $(SRC:.c=.o)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

%.lst: %.elf
	$(OBJDUMP) -h -S $< > $@

%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

%_eeprom.hex: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O ihex $< $@ 
