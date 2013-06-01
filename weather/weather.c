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
#define TIME_MSG_LEN  11    // time sync to PC is HEADER followed by Unix time_t as ten ASCII digits
#define TIME_HEADER  'T'    // Header tag for serial time sync message
#define TIME_REQUEST  7     // ASCII bell character requests a time sync message 

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

const unsigned int sensorReadTimeoutMillis = 1000;
const int serialBaudRate = 9600;
const int sensorReadIntervalMs = 6000;
int millisSinceLastRead = 0;
int sensorPin = 2;


// Microseconds since last state transition. Used to check timing, which is required
// to read bits and helpful for error checking in other states.
volatile long lastTransitionMicros = 0;

// This counter tracks how many signal line changes have happened in this
// sensor run, so we can record each pulse length in the right part of the timings
// array. We also use this for sanity checking; if we didn't get 86 pulses, something
// went wrong.
volatile int signalLineChanges;

// The timings array stores intervals between pulse changes so we can decode them
// once the sensor finishes sending the pulse sequence.
int timings[88];

// If an interrupt needs to report an error, we store the message
// here and print it once the interrupt has ended.
//char errorMsgBuf[256];

// and a flag to store error state during sensor read/processing
boolean errorFlag = false;

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

boolean readSensor(int * temperature_decidegrees_c, int * rel_humidity_decipercent) ;
void flagError() ;
void checkPreBitLowPulse(int pulseMicros, int timingIndex) ;
boolean requestSensorRead() ;
void sensorLineChange();
void initState() ;
void debugPrintTimings() ;
void analyseTimings(int * temperature_decidegrees_c, int * rel_humidity_decipercent) ;
int readnbits(int * timingsIdx, int nbits) ;
int shiftNextBit(int oldValue, int pulseMicros, int timingIndex) ;
  
  
  
int main(void) {
   init();
   setup();
   for (;;) {
      loop();
   }
}

void setup()
{
  //setTime(1,1,1,2,3,2013); 
  // INPUT mode is the default, but being explicit never hurts
  pinMode(sensorPin, INPUT); 
  // Pull the sensor pin high using the Arduino's built-in 20k 
  // pull-up resistor. The sensor expects it to be pulled high
  // for the first 2s and kept high as its default state.
  digitalWrite(sensorPin, HIGH);
  // The sensor expects no communication for the first 2s, so delay
  // entry to the polling loop to give it tons of time to warm up.
  // Now ensure the serial port is ready
  delay(1000);
  Serial.begin(serialBaudRate);

  Wire.begin();
  bmp085Calibration();

  //Serial.println("Sensor initialized, Ready for first reading");

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
  
 
  
  Serial.print(now());
  Serial.print(" ");
  temperature = bmp085GetTemperature(bmp085ReadUT());
  pressure = bmp085GetPressure(bmp085ReadUP());
//  Serial.print("Temperature: ");
  Serial.print(float(temperature)*0.1, DEC);
  Serial.print(" C ");
//  Serial.print("Pressure: ");
  Serial.print(float(pressure)/1000.0, DEC);
  Serial.print(" kPa " );

  int temp;
  int humidity;
  boolean success = readSensor(&temp, &humidity);
  //Serial.println("done READING"); 
  if (success) {
    char buf[32];
    //sprintf(buf, "Reading: %i.%i degrees C at %i.%i relative humidity", temperature/10, abs(temperature%10), humidity/10, humidity%10);
    sprintf(buf, "%i.%i C %i.%i %%RH ", temp/10, abs(temp%10), humidity/10, humidity%10);
    Serial.print(buf);
  }
     //temp = (float)temp/10.0;
     //temp = temp*1.8+32.0;
     //Serial.println((float)temperature);
     //Serial.println((float)humidity/10.0);
  float temp3=(float(analogRead(0))*5.0)/1024.0;
  temp3=(temp3-0.5)*100;//*1.8 + 32;
  Serial.print(" ");
  Serial.print(temp3);

  Serial.println("");
  
  delay(sensorReadIntervalMs);
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


//
// Capture sensor data, returning true on success and false on failure.
//
// On false return the arguments are **NOT** modified.
//
// Temperature is returned in decidegrees centigrade, ie degrees C times ten.
// Divide by ten to get the real temperature. Eg a return of 325 means a
// temperature of 32.4 degrees C. Negative temperatures may be returned,
// so make sure to use the absolute of the remainder when extracting the
// fractional part, eg:
//
//  int wholePart = temperature/10;
//  int fractionalPart = abs(temperature%10);
//
// Humidity is returned in percentage relative humidity times ten. Divide by ten
// to get the real relative humidity. Eg a return of 631 means 63.1% humidity.
//
boolean readSensor(int * temperature_decidegrees_c, int * rel_humidity_decipercent) {
  initState();
  // Install the state transition interrupts so we see our own request pulses
  // and all the sensor replies,
  attachInterrupt(sensorPin - 2, sensorLineChange, CHANGE);
  // Send a sensor read request pulse (7000ms LOW) and check that the sensor replies with 80ms low
  if (!requestSensorRead()) {
    flagError();
  } else {
    // collect the sensor line pulses then stop capturing interrupts on the line
    // once we either time out or get all the sensor pulses we expect.
    const long startMillis = millis();
    do {
      delay(100);
    } while ( signalLineChanges < 86 && ( (millis() - startMillis) < sensorReadTimeoutMillis));
    detachInterrupt(sensorPin - 2);
    if (signalLineChanges != 86) {
      //sprintf(errorMsgBuf, "*** MISSED INTERRUPTS ***: Expected 86 line changes, saw %i", signalLineChanges);
      flagError();
    } else {
      //debugPrintTimings();  // XXX DEBUG
      // TODO: memory barrier to ensure visibility of timings array
      // Feed the collected pulse sequence through the state machine to analyze it.
      analyseTimings(temperature_decidegrees_c, rel_humidity_decipercent);
    }
  }
  return !errorFlag;
}
  
// Interrupt service routine to capture timing between signal pulses and record
// them. The pulse sequence will be analysed once it's all been captured so we don't
// have to worry so much about execution time.
void sensorLineChange() {
  const long pulseMicros = micros() - lastTransitionMicros;
  lastTransitionMicros = micros();
  timings[signalLineChanges] = pulseMicros;
  signalLineChanges++;
}

// Clear all the sensor reader state before we start reading
void initState() {
  detachInterrupt(sensorPin - 2);
  for (int i = 0; i < 86; i++) { timings[i] = 0; }
  errorFlag = false;
  lastTransitionMicros = micros();
  signalLineChanges = 0;
  // TODO: memory barrier to guarantee visibility of timings array
}

// DEBUG routine: dump timings array to serial
void debugPrintTimings() { // XXX DEBUG
  for (int i = 0; i < 86; i++) { // XXX DEBUG
    if (i%10==0) { Serial.print("\n\t"); }
    char buf[24];
    sprintf(buf, i%2==0 ? "H[%02i]: %-3i  " : "L[%02i]: %-3i  ", i, timings[i]);
    Serial.print(buf);
  } // XXX DEBUG
  Serial.print("\n"); // XXX DEBUG
}

void analyseTimings(int * temperature_decidegrees_c, int * rel_humidity_decipercent) {
  // The pulse sequence is:
  // (Processed by requestSensorRead()):
  //   LOW (1000 - 10000us) - our sensor request
  //   HIGH (20-40us) - our pull high after sensor request
  //   LOW (80us) - Sensor pulls down to ACK request
  //   HIGH (80us) - Sensor pulls up to ACK request
  // (Processed as results here):
  //   16 bits of humidity data, 8 bits integral and 8 bits fractional part
  //   16 bits of temperature data, 8 bits integral and 8 bits fractional part
  //   8 bits of checksum
  // each bit of which is sent as:
  //   LOW (50us): Expect bit pulse
  //   -- THEN EITHER --
  //   HIGH(18-30us): 0 bit
  //   -- OR --
  //   HIGH(55-85us): 1 bi
  // Start at the 6th entry of the timings array, the first pre-bit low pulse
  int timingsIdx = 5;
  int humid16 = readnbits(&timingsIdx, 16);
  if (errorFlag) {
    Serial.println("Failed to capture humidity data");
    return;
  }
  int temp16 = readnbits(&timingsIdx, 16);
  if (errorFlag) {
    Serial.println("Failed to capture temperature data");
    return;
  }
  int checksum8 = readnbits(&timingsIdx, 8);
  if (errorFlag) {
    Serial.println("Failed to capture checksum");
    return;
  }
  // Verify the checksum. "Checksum" is too good a word to describe this
  // but it'll probably help catch the odd bit error.
  byte cs = (byte)(humid16>>8) + (byte)(humid16&0xFF) + (byte)(temp16>>8) + (byte)(temp16&0xFF);
  if (cs != checksum8) {
    //sprintf(errorMsgBuf, "Checksum mismatch, bad sensor read");
    flagError();
  }
  // If bit 16 is set in the temperature, temperature is negative.
  if ( temp16 & (1<<15) ) {
    temp16 = -(temp16 & (~(1<<15)));
  }
  if (!errorFlag) {
    *temperature_decidegrees_c = temp16;
    *rel_humidity_decipercent = humid16;
  }
}

int readnbits(int * timingsIdx, int nbits) {
  const int * t = timings + *timingsIdx;
  const int * tStop = t + nbits*2;
  int result = 0;
  char buf[12];
  while (t != tStop) {
    checkPreBitLowPulse( *(t++), (*timingsIdx)++ );
    result = shiftNextBit( result, *(t++), (*timingsIdx)++ );
  }
  return result;
}


int shiftNextBit(int oldValue, int pulseMicros, int timingIndex) {
  // The datasheet says 0 pulses are 22-26us and 1 pulses are 70us. The datasheet is a lie, there's
  // lots more variation than that. I've observed 0s from 12 <= x <= 40  for example.  
  if (pulseMicros > 10 && pulseMicros < 40) {
    return (oldValue<<1) | 0;
  } else if (pulseMicros > 60 && pulseMicros < 85) {
    return (oldValue<<1) | 1;
  } else {
    //sprintf(errorMsgBuf, "Bad bit pulse length: %i us at timing idx %i", pulseMicros, timingIndex);
    flagError();
    return 0xFFFFFFFF;
  }
}

// Ensure the passed pulse length is within an acceptable range for a pre-bit-transmission
// low pulse. It should be 45-55 us, but the datasheet is full of lies and we seem to need a tolerance
// around 35-75us to get it to actually work.
void checkPreBitLowPulse(int pulseMicros, int timingIndex) {
  if (pulseMicros <= 35 || pulseMicros >= 75) {
    //sprintf(errorMsgBuf, "Low pulse before bit transmit (%i us) outside 45-70us tolerance at timing idx %i", pulseMicros, timingIndex);
    flagError();
  }
}

// Signal the sensor to request data.
// At entry the line should be pulled high, and it'll be pulled high
// on return.
boolean requestSensorRead() {
  // Make sure we're pulled HIGH to start with
  if (digitalRead(sensorPin) != HIGH) {
    //sprintf(errorMsgBuf, "Line not HIGH at entry to requestSensorRead()");
    flagError();
    return false;
  }
  // Pulse the line low for 1-10ms (we use 7ms)
  // to request that the sensor take a reading
  digitalWrite(sensorPin, LOW);
  pinMode(sensorPin, OUTPUT);
  delayMicroseconds(7000);
  
  // Push the pin explicitly HIGH. This shouldn't really be necessary, but appears to
  // produce a more clean and definite response, plus the datasheet says we should push
  // it high for 20-40us. Without this the sensor readings tend to be unreliable.
  digitalWrite(sensorPin, HIGH);
  delayMicroseconds(30);
  
  // Go back to input mode and restore the pull-up so we can receive the sensor's
  // acknowledgement LOW pulse. The pin will remain HIGH because of the 20k pull-up
  // resistor activated by the Arduino when an OUTPUT pin is set HIGH.
  pinMode(sensorPin, INPUT);
  
  // Then wait for the sensor to pull it low. The sensor will respond
  // after between 20 and 100 microseconds by pulling the line low.  
  // If it hasn't done so within 200 microseconds assume it didn't see our
  // pulse and enter error state.
  int pulseLength = pulseIn(sensorPin, LOW, 200);
  if (pulseLength == 0) {
    //sprintf(errorMsgBuf, "Sensor read failed: Sensor never pulled line LOW after initial request");
    flagError();
    return false;
  }
  // The sensor has pulled the line low. Wait for it to return to HIGH.
  delayMicroseconds(5);
  pulseLength = pulseIn(sensorPin, HIGH, 200);
  if (pulseLength == 0) {  
    //sprintf(errorMsgBuf, "Sensor read failed: Sensor didn't go back HIGH after LOW response to read request");
    flagError();
    return false;
  }
  return true;
}

void flagError() {
  // On error restore the sensor pin to pulled up with high impedance
  pinMode(sensorPin, INPUT);
  digitalWrite(sensorPin, HIGH);
  // Flag error state
  errorFlag = true;
  // then write the error msg
  //Serial.println(errorMsgBuf);
}


