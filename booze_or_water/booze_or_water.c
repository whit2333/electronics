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

//______________________________________________________________________________
int                 alcohol_channel   = 1;
const unsigned char OSS               = 0;  // Oversampling Setting
float               alcohol_baseline  = 0.0;
bool                subtract_baseline = true; /// Set to subtract baseline measurment from reading done in setup
volatile int        state             = LOW;
volatile int        nbaseline         = 0;    /// Number loops since last baseline calculation
volatile bool       measure_alcohol   = false;

//______________________________________________________________________________
void            setup();
void            loop();
void            booze_or_water_interrupt();
void            booze_or_water();
void            blowing_lights();
void            flashing_lights(int t);
void            water_lights();
void            booze_lights();
float           read_alcohol();
void            print_alcohol();
void            print_result(float al);
void            calculate_baseline();
void            digitalClockDisplay();
void            processSyncMessage();
void            printDigits(int digits);
float           GetAlcoholResistance();
float           GetAlcoholDensity(float);

//______________________________________________________________________________
int main(void) {
   init();
   setup();
   for (;;) {
      loop();
   }
}

//______________________________________________________________________________
void setup()
{
   Serial.begin(9600);

   // elwire driver  
   //pinMode(2, OUTPUT);  // channel A  
   //pinMode(3, OUTPUT);  // channel B  
   //pinMode(4, OUTPUT);  // channel c  

   pinMode(5, OUTPUT);  // channel D 
   pinMode(6, OUTPUT);  // channel E
   pinMode(7, OUTPUT);  // channel F  - booze 
   pinMode(8, OUTPUT);  // channel G  - water
   pinMode(9, OUTPUT);  // channel H  

   digitalWrite(5, LOW);   // turn the EL channel off 
   digitalWrite(6, LOW);   // turn the EL channel off 
   digitalWrite(7, LOW);   // turn the EL channel off 
   digitalWrite(8, LOW);   // turn the EL channel off 
   digitalWrite(9, LOW);   // turn the EL channel off 

   // We also have two status LEDs, pin 10 on the Escudo, 
   // and pin 13 on the Arduino itself
   pinMode(10, OUTPUT);     
   pinMode(13, OUTPUT);    

   // Interrupt for taking a measurement. 
   // Interrupt is on digital channel 2
   attachInterrupt(0, booze_or_water_interrupt, FALLING);

   //setTime(1,1,1,2,3,2013); 
  
   Serial.println("Setup Complete");
}

//______________________________________________________________________________
void loop()
{
   // elwire
   //digitalWrite(9, HIGH);   // turn the EL channel on
   //digitalWrite(8, LOW);   // turn the EL channel on
   //digitalWrite(7, LOW);   // turn the EL channel on

   flashing_lights(400);

   if(Serial.available() )
   {
      processSyncMessage();
   }
   //if(timeStatus() == timeNotSet)
   //     Serial.println("waiting for sync message");
   //else    
   //       digitalClockDisplay();  
   //digitalClockDisplay();  

   //delay(1200);
   if( measure_alcohol ) {
      digitalWrite(9, HIGH);   // turn the EL channel on
      delay(100);
      digitalWrite(9, LOW);   // turn the EL channel on
      delay(100);
      digitalWrite(9, HIGH);   // turn the EL channel on
      delay(100);
      digitalWrite(9, LOW);   // turn the EL channel on
      delay(100);
      digitalWrite(9, HIGH);   // turn the EL channel on
      delay(100);
      digitalWrite(9, LOW);   // turn the EL channel on
      delay(100);
      booze_or_water();
      measure_alcohol = false;
   }

}

//______________________________________________________________________________
void booze_or_water_interrupt(){
   measure_alcohol = true;
}

//______________________________________________________________________________
void booze_or_water(){

   float first  = read_alcohol();
   float max = 0.0;
   float avg = first;
   float current = 0;
   int i = 0;
   int Nread = 10;
   for(i = 1;i<Nread;i++)
   {
      blowing_lights();
      current = read_alcohol();
      if(current > max ) max = current;
      avg += current; 
   }
   avg = avg/float(Nread);
   blowing_lights();
   float last  = read_alcohol();

   Serial.print("First: ");
   print_result(first);
   Serial.print("\n");
   
   Serial.print("Last: ");
   print_result(last);
   Serial.print("\n");
   
   Serial.print("Average: ");
   print_result(avg);
   Serial.print("\n");

   if( (last > first) && (avg > 2.0*first) ) 
   {
      Serial.print(" water 0 \n");
      water_lights(); 
   } 
   else if( avg*0.21 > 0.2 ) 
   {
      Serial.print(" water 1 \n");
      water_lights();
   } 
   else booze_lights();

   print_alcohol();
}

//______________________________________________________________________________
void print_result(float al){ 
   Serial.print(float(al), DEC);
   Serial.print(" mg/L ");
   Serial.print(float(al*0.21000));
   Serial.print(" %BAC ");
}
//______________________________________________________________________________
void booze_lights(){
   digitalWrite(8, LOW);   
   digitalWrite(7, HIGH);  
   delay(200);
   int i;
   for(i=0;i<10;i++){
      digitalWrite(7, LOW);   
      delay(200);
      digitalWrite(7, HIGH);  
      delay(200);
      digitalWrite(7, LOW);   
      delay(200);
      digitalWrite(7, HIGH);  
      delay(1500);
   }
}
//______________________________________________________________________________
void water_lights(){
   digitalWrite(7, LOW);   
   digitalWrite(8, HIGH);  
   delay(200);
   int i;
   for(i=0;i<10;i++){
      digitalWrite(8, LOW);   
      delay(200);
      digitalWrite(8, HIGH);  
      delay(200);
      digitalWrite(8, LOW);   
      delay(200);
      digitalWrite(8, HIGH);  
      delay(1500);
   }
}
//______________________________________________________________________________
void flashing_lights(int t){
   int i;
   for(i=9;i>=5;i--){
      digitalWrite(i, HIGH);  
      delay(t);
      digitalWrite(i, LOW);   
      delay(t);
   }
}

//______________________________________________________________________________
void blowing_lights(){
   flashing_lights(100); 
   flashing_lights(100); 
   //flashing_lights(300); 
   //flashing_lights(200); 
   //digitalWrite(9, LOW);   // turn the EL channel on
   //digitalWrite(8, HIGH);   // turn the EL channel on
   //digitalWrite(7, LOW);   // turn the EL channel on
   //delay(300);
   //digitalWrite(9, HIGH);   // turn the EL channel on
   //digitalWrite(8, LOW);   // turn the EL channel on
   //digitalWrite(7, HIGH);   // turn the EL channel on
   //delay(300);
   //digitalWrite(9, LOW);   // turn the EL channel on
   //digitalWrite(8, HIGH);   // turn the EL channel on
   //digitalWrite(7, LOW);   // turn the EL channel on
   //delay(200);
   //digitalWrite(9, HIGH);   // turn the EL channel on
   //digitalWrite(8, LOW);   // turn the EL channel on
   //digitalWrite(7, HIGH);   // turn the EL channel on
   //delay(200);
   //digitalWrite(9, LOW);   // turn the EL channel on
   //digitalWrite(8, HIGH);   // turn the EL channel on
   //digitalWrite(7, LOW);   // turn the EL channel on
   //delay(100);
   //digitalWrite(9, HIGH);   // turn the EL channel on
   //digitalWrite(8, LOW);   // turn the EL channel on
   //digitalWrite(7, HIGH);   // turn the EL channel on
   //delay(100);
   //digitalWrite(9, LOW);   // turn the EL channel on
   //digitalWrite(8, LOW);   // turn the EL channel on
   //digitalWrite(7, LOW);   // turn the EL channel on

}

//______________________________________________________________________________
void calculate_baseline(){
   Serial.println("Taking baseline measurement. Please wait...");
   alcohol_baseline  = GetAlcoholDensity(GetAlcoholResistance());
   Serial.print(" baseline reading is ");
   Serial.print(float(alcohol_baseline), DEC);
   Serial.print(" mg/L \n");
   nbaseline = 0;
}

//______________________________________________________________________________
float read_alcohol(){
   float al = GetAlcoholDensity(GetAlcoholResistance());
   if(subtract_baseline) al = al - alcohol_baseline;
   return(al);
}

//______________________________________________________________________________
void print_alcohol(){
   float al = read_alcohol();
   Serial.print(now());
   Serial.print(" ");
   Serial.print(float(al), DEC);
   Serial.print(" mg/L ");
   Serial.print(float(al*0.21000),DEC);
   Serial.print(" %BAC\n");
}

//______________________________________________________________________________
float           GetAlcoholResistance(){
   // Returns the resistance
   float Rs =10.0 * ( (1024.0/float(analogRead(alcohol_channel))) -1.0 );
   return(Rs); 
}

//______________________________________________________________________________
float           GetAlcoholDensity(float Rs) {
   // Returns the the calculated alcohol density
   float A = 0.5374;
   float B = -0.6818;
   float R0 = 10.0;
   return( pow( Rs/(R0*A) , (1.0/B) ) ); 
}

//______________________________________________________________________________
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

//______________________________________________________________________________
void printDigits(int digits){
   // utility function for digital clock display: prints preceding colon and leading 0
   Serial.print(":");
   if(digits < 10)
      Serial.print('0');
   Serial.print(digits);
}

//______________________________________________________________________________
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
//______________________________________________________________________________

//______________________________________________________________________________
