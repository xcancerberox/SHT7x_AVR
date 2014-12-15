TARGET = test_code
MMCU = atmega32
F_CPU = 16000000UL

CC = avr-gcc
CFLAGS = -Os -g
SRC = $(shell find .  -name '*.[c]')
INC = $(shell find .  -name '*.[h]')

TARGET_P = m32
HFUSE =
LFUSE =

CDEFINES = -DF_CPU=$(F_CPU)

all: bin

bin: $(SRC) $(INC)
	$(CC) $(CDEFINES) -mmcu=$(MMCU) $(CFLAGS) -o $(TARGET).bin $(SRC)
	avr-size -d $(TARGET).bin

hex: bin
	avr-objcopy -j .text -O ihex $(TARGET).bin $(TARGET).hex

program: hex
	avrdude -c usbtiny -p $(TARGET_P) -U f:w:$(TARGET).hex

program_dw: bin
	avarice -w -j usb --erase --program --file $(TARGET).bin

fuse:
	avrdude -c usbtiny -p $(TARGET_P) -U lfuse:w:$(LFUSE):m -U hfuse:w:$(HFUSE):m

asm: $(SRC) $(INC)
	$(CC) $(CDEFINES) -mmcu=$(MMCU) $(CFLAGS) -S -o $(TARGET).s $<

clean:
	rm -rf $(TARGET).hex $(TARGET).bin $(TARGET).s
