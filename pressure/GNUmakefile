target = pressure

LIBS = -lWire -lTime -larduino

default:
	avr-g++ -L/usr/local/lib -I/usr/local/include -Wall -DF_CPU=16000000UL -Os -mmcu=atmega328p -o $(target).elf $(target).c $(LIBS) 
	avr-objcopy -O ihex -R .eeprom $(target).elf $(target).hex

upload:
	avrdude -c arduino -p m328p -b 115200 -P /dev/ttyACM0 -U flash:w:$(target).hex
	#avrdude -c avrispmkII -p m328p -b 57600 -P /dev/ttyACM0 -U flash:w:out.hex
	#avrdude -c arduino -p m328p -b 57600 -P /dev/ttyACM0 -U flash:w:out.hex

clean: 
	rm $(target).hex      
	rm $(target).elf      

all: default upload

