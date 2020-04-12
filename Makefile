# Makefile to automate the usage of avra and avrdude in the programming of the atmega328p

BUILD_DIR=./build/
AVR_CONF=../avrdude.conf
USB_MODEM := $(shell ls /dev/tty.usb*)

build:
	mkdir build
	avr-gcc -mmcu=atmega328p main.c -o main.out
	avr-objcopy -j .text -j .data -O ihex main.out main.hex
	mv *.out $(BUILD_DIR)
	mv *.hex $(BUILD_DIR)

run:
	avrdude -C $(AVR_CONF) -v -p atmega328p -c arduino -P $(USB_MODEM) -b 115200 -D -U flash:w:$(BUILD_DIR)main.hex:i

clean:
	rm -rf build/
	rm -f *.obj *.hex

install: clean build run
