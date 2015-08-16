# BSD licensed, see http://lastyearswishes.com/static/Makefile for full license
#
# Modify PREFIX for installation 
# and  set ARDUINO_SRC to proper location of the arduino source code

PREFIX=/usr/local
ARDUINO_SRC=/home/whit/arduino/Arduino-1.6.5

HDRS = Arduino.h binary.h Client.h HardwareSerial.h IPAddress.h \
       new.h pins_arduino.h Platform.h Printable.h Print.h \
       Server.h Stream.h Udp.h USBAPI.h USBCore.h USBDesc.h \
       WCharacter.h wiring_private.h WString.h


OBJS = WInterrupts.o wiring_analog.o wiring.o wiring_digital.o \
       wiring_pulse.o wiring_shift.o CDC.o HardwareSerial.o \
       HID.o IPAddress.o main.o new.o Print.o Stream.o \
       Tone.o USBCore.o WMath.o WString.o


#may need to adjust -mmcu if you have an older atmega168
#may also need to adjust F_CPU if your clock isn't set to 16Mhz
CFLAGS = -I./ -std=gnu99  -DARDUINO=165 -DF_CPU=16000000UL -Os -mmcu=atmega328p -lm
CPPFLAGS = -I./ -DF_CPU=16000000UL -DARDUINO=165 -Os -mmcu=atmega328p -lm

CC=avr-gcc
CPP=avr-gcc
AR=avr-ar

#------ Standard Arduino Library ------#
default: all

all: libarduino.a libWire.a libTime.a 

libarduino.a:   ${OBJS}
	${AR} crs libarduino.a $(OBJS)

libTime.a: libraries/Time/Time.h
	cp libraries/Time/Time.h .
	${CPP} ${CPPFLAGS} -c libraries/Time/DateStrings.cpp -Ilibraries/Time
	${CPP} ${CPPFLAGS} -c libraries/Time/Time.cpp -Ilibraries/Time
	${AR} crs libTime.a Time.o DateStrings.o


libWire.a: $(ARDUINO_SRC)/hardware/arduino/avr/libraries/Wire/Wire.h
	cp $(ARDUINO_SRC)/hardware/arduino/avr/libraries/Wire/Wire.h .
	cp $(ARDUINO_SRC)/hardware/arduino/avr/libraries/Wire/utility/twi.h .
	${CPP} ${CPPFLAGS} -c $(ARDUINO_SRC)/hardware/arduino/avr/libraries/Wire/Wire.cpp -I$(ARDUINO_SRC)/hardware/arduino/avr/libraries/Wire/utility
	${CC} ${CFLAGS} -c $(ARDUINO_SRC)/hardware/arduino/avr/libraries/Wire/utility/twi.c -I$(ARDUINO_SRC)/libraries/Wire/utility
	${AR} crs libWire.a Wire.o twi.o
	

.c.o: ${HDRS}
	${CC} ${CFLAGS} -c $*.c -lm

.cpp.o: ${HDRS}
	${CPP} ${CPPFLAGS} -c $*.cpp

clean:
	rm -f ${OBJS} 
	rm -f *.a 
	rm -f *.o 

install: libarduino.a
	mkdir -p ${PREFIX}/lib
	mkdir -p ${PREFIX}/include
	cp *.h ${PREFIX}/include
	cp *.a ${PREFIX}/lib

