/** Pressure sensor  and data logger.
 *  First, configure the serial port:
 *  $stty -F /dev/ttyACM0 cs8 9600 ignbrk -brkint -icrnl -imaxbel -opost -onlcr -isig -icanon -iexten -echo -echoe -echok -echoctl -echoke noflsh -ixon -crtscts 
 *
 *  Second, send it a time syncroniztion
 *  $TZ_adjust=-8;  echo T$(($(date +%s)+60*60*$TZ_adjust)) > /dev/ttyACM0
 *
 */
#include <Arduino.h>
#include <Time.h>
#include <math.h>

#define TIME_MSG_LEN  11   // time sync to PC is HEADER followed by Unix time_t as ten ASCII digits
#define TIME_HEADER  'T'   // Header tag for serial time sync message
#define TIME_REQUEST  7    // ASCII bell character requests a time sync message 

const unsigned char OSS = 0;  // Oversampling Setting

float alcohol_baseline = 0.0;
bool  subtract_baseline = true; /// Set to subtract baseline measurment from reading done in setup

void            setup();
void            loop();
void digitalClockDisplay();
void processSyncMessage();
void printDigits(int digits);
float           GetAlcoholResistance();
float           GetAlcoholDensity(float);


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
  
   Serial.println("Taking baseline measurement. Please wait...");
   alcohol_baseline  = GetAlcoholDensity(GetAlcoholResistance());
   Serial.print(" baseline reading is ");
   Serial.print(float(alcohol_baseline), DEC);
   Serial.print(" mg/L \n");


  Serial.println("Setup Complete");
}

void loop()
{

   
     if(Serial.available() )
     {
          processSyncMessage();
      }
  //if(timeStatus() == timeNotSet)
  //     Serial.println("waiting for sync message");
  //else    
  //       digitalClockDisplay();  
 
  //digitalClockDisplay();  
  Serial.print(now());
  Serial.print(" ");

   float al = GetAlcoholDensity(GetAlcoholResistance());
   if(subtract_baseline) al = al - alcohol_baseline;
   Serial.print(float(al), DEC);
   Serial.print(" mg/L ");
   Serial.print(float(al*0.21000));
   Serial.print(" %BAC\n");
  delay(300);
}

/* Returns resistance */
float           GetAlcoholResistance(){
   float Rs =10.0 * ( (1024.0/float(analogRead(1))) -1.0 );
   return(Rs); 
}

float           GetAlcoholDensity(float Rs) {
   float A = 0.5374;
   float B = -0.6818;
   float R0 = 38.0;
   return( pow( Rs/(R0*A) , (1.0/B) ) ); 
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
