target = vw_rf_tx

LIBS = -lWire -lTime -larduino

device=/dev/ttyACM1

CC=avr-gcc
CPP=avr-g++
AR=avr-ar

CFLAGS = -I. -I../arduino_build -L../arduino_build/lib -I/usr/local/include -std=gnu99  -DARDUINO=165 -DF_CPU=16000000UL -Os -mmcu=atmega328p -lm
CPPFLAGS = -I. -I../arduino_build -L../arduino_build/lib -I/usr/local/include -DF_CPU=16000000UL -DARDUINO=165 -Os -mmcu=atmega328p -lm

default: VirtualWire.o
	avr-g++ VirtualWire.o -L/usr/local/lib -I../arduino_build -L../arduino_build/lib -I/usr/local/include -Wall  -DARDUINO=165 -DF_CPU=16000000UL -Os -mmcu=atmega328p -o $(target).elf $(target).c $(LIBS)  
	avr-objcopy -O ihex -R .eeprom $(target).elf $(target).hex

upload:
	avrdude -c arduino -p m328p -b 115200 -P $(device) -U flash:w:$(target).hex
	#avrdude -c avrispmkII -p m328p -b 57600 -P /dev/ttyACM0 -U flash:w:out.hex
	#avrdude -c arduino -p m328p -b 57600 -P /dev/ttyACM0 -U flash:w:out.hex

.c.o: ${HDRS}
	${CC} ${CFLAGS} -c $*.c -lm

.cpp.o: ${HDRS}
	${CPP} ${CPPFLAGS} -c $*.cpp

clean: 
	rm -f $(target).hex      
	rm -f $(target).elf      
	rm -f *.o      

all: default upload

