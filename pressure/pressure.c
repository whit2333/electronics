/** Pressure sensor  and data logger.
 *  First, configure the serial port:
 *  $stty -F /dev/ttyACM0 cs8 9600 ignbrk -brkint -icrnl -imaxbel -opost -onlcr -isig -icanon -iexten -echo -echoe -echok -echoctl -echoke noflsh -ixon -crtscts 
 *
 *  Second, send it a time syncroniztion
 *  $TZ_adjust=-8;  echo T$(($(date +%s)+60*60*$TZ_adjust)) > /dev/ttyACM0
 *
 */
#include <Arduino.h>
#include <Wire.h>
#include <Time.h>

#define BMP085_ADDRESS 0x77  // I2C address of BMP085

#define TIME_MSG_LEN  11   // time sync to PC is HEADER followed by Unix time_t as ten ASCII digits
#define TIME_HEADER  'T'   // Header tag for serial time sync message
#define TIME_REQUEST  7    // ASCII bell character requests a time sync message 

const unsigned char OSS = 0;  // Oversampling Setting

// Calibration values
int ac1;
int ac2; 
int ac3; 
unsigned int ac4;
unsigned int ac5;
unsigned int ac6;
int b1; 
int b2;
int mb;
int mc;
int md;

// b5 is calculated in bmp085GetTemperature(...), this variable is also used in bmp085GetPressure(...)
// so ...Temperature(...) must be called before ...Pressure(...).
long b5; 

short temperature;
long pressure;

void            setup();
void            loop();
void            bmp085Calibration();
short           bmp085GetTemperature(unsigned int ut);
long            bmp085GetPressure(unsigned long up);
char            bmp085Read(unsigned char address);
int             bmp085ReadInt(unsigned char address);
unsigned int    bmp085ReadUT();
unsigned long   bmp085ReadUP();
void digitalClockDisplay();
void processSyncMessage();
void printDigits(int digits);

int main(void) {
   init();
   setup();
   for (;;) {
      loop();
   }
}

void setup()
{
  Serial.begin(9600);
  //setTime(1,1,1,2,3,2013); 
  Wire.begin();
  bmp085Calibration();
}

void loop()
{
   if(Serial.available() )
     {
          processSyncMessage();
      }
  if(timeStatus() == timeNotSet)
       Serial.println("waiting for sync message");
  else    
         digitalClockDisplay();  
 
  //digitalClockDisplay();  
  Serial.print(now());
  Serial.print(" ");
  temperature = bmp085GetTemperature(bmp085ReadUT());
  pressure = bmp085GetPressure(bmp085ReadUP());
//  Serial.print("Temperature: ");
  Serial.print(float(temperature)*0.1, DEC);
  Serial.print(" C ");
//  Serial.print("Pressure: ");
  Serial.print(float(pressure)/1000.0, DEC);
  Serial.print(" kPa" );
  Serial.println();
  
 delay(1000);
}

// Stores all of the bmp085's calibration values into global variables
// Calibration values are required to calculate temp and pressure
// This function should be called at the beginning of the program
void bmp085Calibration()
{
  ac1 = bmp085ReadInt(0xAA);
  ac2 = bmp085ReadInt(0xAC);
  ac3 = bmp085ReadInt(0xAE);
  ac4 = bmp085ReadInt(0xB0);
  ac5 = bmp085ReadInt(0xB2);
  ac6 = bmp085ReadInt(0xB4);
  b1 = bmp085ReadInt(0xB6);
  b2 = bmp085ReadInt(0xB8);
  mb = bmp085ReadInt(0xBA);
  mc = bmp085ReadInt(0xBC);
  md = bmp085ReadInt(0xBE);
}

// Calculate temperature given ut.
// Value returned will be in units of 0.1 deg C
short bmp085GetTemperature(unsigned int ut)
{
  long x1, x2;
  
  x1 = (((long)ut - (long)ac6)*(long)ac5) >> 15;
  x2 = ((long)mc << 11)/(x1 + md);
  b5 = x1 + x2;

  return ((b5 + 8)>>4);  
}

// Calculate pressure given up
// calibration values must be known
// b5 is also required so bmp085GetTemperature(...) must be called first.
// Value returned will be pressure in units of Pa.
long bmp085GetPressure(unsigned long up)
{
  long x1, x2, x3, b3, b6, p;
  unsigned long b4, b7;
  
  b6 = b5 - 4000;
  // Calculate B3
  x1 = (b2 * (b6 * b6)>>12)>>11;
  x2 = (ac2 * b6)>>11;
  x3 = x1 + x2;
  b3 = (((((long)ac1)*4 + x3)<<OSS) + 2)>>2;
  
  // Calculate B4
  x1 = (ac3 * b6)>>13;
  x2 = (b1 * ((b6 * b6)>>12))>>16;
  x3 = ((x1 + x2) + 2)>>2;
  b4 = (ac4 * (unsigned long)(x3 + 32768))>>15;
  
  b7 = ((unsigned long)(up - b3) * (50000>>OSS));
  if (b7 < 0x80000000)
    p = (b7<<1)/b4;
  else
    p = (b7/b4)<<1;
    
  x1 = (p>>8) * (p>>8);
  x1 = (x1 * 3038)>>16;
  x2 = (-7357 * p)>>16;
  p += (x1 + x2   + 3791)>>4;
  
  return p;
}

// Read 1 byte from the BMP085 at 'address'
char bmp085Read(unsigned char address)
{
  unsigned char data;
  
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(address);
  //Wire.send(address);
  Wire.endTransmission();
  
  Wire.requestFrom(BMP085_ADDRESS, 1);
  while(!Wire.available())
    ;
    
  return Wire.read();
}

// Read 2 bytes from the BMP085
// First byte will be from 'address'
// Second byte will be from 'address'+1
int bmp085ReadInt(unsigned char address)
{
  unsigned char msb, lsb;
  
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();
  
  Wire.requestFrom(BMP085_ADDRESS, 2);
  while(Wire.available()<2)
    ;
  msb = Wire.read();
  lsb = Wire.read();
  
  return (int) msb<<8 | lsb;
}

// Read the uncompensated temperature value
unsigned int bmp085ReadUT()
{
  unsigned int ut;
  
  // Write 0x2E into Register 0xF4
  // This requests a temperature reading
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x2E);
  Wire.endTransmission();
  
  // Wait at least 4.5ms
  delay(5);
  
  // Read two bytes from registers 0xF6 and 0xF7
  ut = bmp085ReadInt(0xF6);
  return ut;
}

// Read the uncompensated pressure value
unsigned long bmp085ReadUP()
{
  unsigned char msb, lsb, xlsb;
  unsigned long up = 0;
  
  // Write 0x34+(OSS<<6) into register 0xF4
  // Request a pressure reading w/ oversampling setting
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x34 + (OSS<<6));
  Wire.endTransmission();
  
  // Wait for conversion, delay time dependent on OSS
  delay(2 + (3<<OSS));
  
  // Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF6);
  Wire.endTransmission();
  Wire.requestFrom(BMP085_ADDRESS, 3);
  
  // Wait for data to become available
  while(Wire.available() < 3)
    ;
  msb = Wire.read();
  lsb = Wire.read();
  xlsb = Wire.read();
  
  up = (((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8-OSS);
  
  return up;
}


/*
int ledPin =  13;    // LED connected to digital pin 13

void setup() {
   pinMode(ledPin, OUTPUT);
}

   void loop() {
   digitalWrite(ledPin, HIGH);   // set the LED on
   delay(500);                   // wait for half a second
   digitalWrite(ledPin, LOW);    // set the LED off
   delay(500);                   // wait for half a second
}
*/
void digitalClockDisplay(){
   // digital clock display of the time
   Serial.print(hour());
   printDigits(minute());
   printDigits(second());
   Serial.print(" ");
   Serial.print(day());
   Serial.print("-");
   Serial.print(month());
   Serial.print("-");
   Serial.print(year());
   Serial.print(" ");
}

void printDigits(int digits){
   // utility function for digital clock display: prints preceding colon and leading 0
   Serial.print(":");
   if(digits < 10)
      Serial.print('0');
   Serial.print(digits);
}

void processSyncMessage() {
   // if time sync available from serial port, update time and return true
   while(Serial.available() >=  TIME_MSG_LEN ){  // time message consists of header & 10 ASCII digits
      char c = Serial.read() ;
      Serial.print(c);  
      if( c == TIME_HEADER ) {      
         time_t pctime = 0;
         for(int i=0; i < TIME_MSG_LEN -1; i++){  
            c = Serial.read();          
            if( c >= '0' && c <= '9'){  
               pctime = (10 * pctime) + (c - '0') ; // convert digits to a number    
            }
         }  
         setTime(pctime);   // Sync Arduino clock to the time received on the serial port
      }  
   }
}
