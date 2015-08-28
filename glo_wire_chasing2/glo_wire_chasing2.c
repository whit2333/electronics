#include <Arduino.h>

// Test sketch for El Escudo Dos
// Turn each EL channel (A-H) on in sequence and repeat
// Mike Grusin, SparkFun Electronics

int speeds[10] = { 20, 25, 30, 35, 40, 30, 40, 50, 60, 80};
int stat1 = 0;
int count1 = 0;

void setup() {                
   // The EL channels are on pins 2 through 9
   // Initialize the pins as outputs
   pinMode(2, OUTPUT);  // channel A  
   pinMode(3, OUTPUT);  // channel B   
   pinMode(4, OUTPUT);  // channel C
   pinMode(5, OUTPUT);  // channel D    
   pinMode(6, OUTPUT);  // channel E
   pinMode(7, OUTPUT);  // channel F
   pinMode(8, OUTPUT);  // channel G
   pinMode(9, OUTPUT);  // channel H
   // We also have two status LEDs, pin 10 on the Escudo, 
   // and pin 13 on the Arduino itself
   pinMode(10, OUTPUT);     
   pinMode(13, OUTPUT);    
   digitalWrite(10, LOW);   // blink both status LEDs
   digitalWrite(13, LOW);
   digitalWrite(8, HIGH);
   digitalWrite(9, HIGH);
}

void grow(int dtime){

   int x;
   //int dtime = 200;
   for (x=2; x<=9; x++)
   {
      digitalWrite(x, HIGH);   // turn the EL channel on
      delay(dtime);              // wait for 1/10 second
   }
   for (x=9; x>=2; x--)
   {
      //digitalWrite(x, HIGH);   // turn the EL channel on
      digitalWrite(x, LOW);    // turn the EL channel off
      delay(dtime);              // wait for 1/10 second

   }
   for (x=9; x>=2; x--)
   {
      digitalWrite(x, HIGH);   // turn the EL channel on
      //digitalWrite(x, LOW);    // turn the EL channel off
      delay(dtime);              // wait for 1/10 second

   }
   for (x=2; x<=9; x++)
   {
      digitalWrite(x, LOW);   // turn the EL channel on
      delay(dtime);              // wait for 1/10 second
   }
}

void chase_wire(int time, int repeat) 
{
   int x;
   int i;

   stat1 = !stat1;
   digitalWrite(8, stat1);
   digitalWrite(9, stat1);
   for( i=0;i<=repeat;i++) {
      // Step through all eight EL channels (pins 2 through 9)
      for (x=2; x<=4; x++)
      {
         digitalWrite(x  , HIGH);
         digitalWrite(x+3, HIGH);
         delay(time);
         digitalWrite(x  , LOW);
         digitalWrite(x+3, LOW);
      }
   }
}

void loop() 
{
   int x = 1;
   int status = 0;
   int time_length = 300;
   int rep;

   grow(120);
   grow(100);
   grow(80);
   grow(70);
   grow(60);
   grow(50);
   grow(50);

   for (x=0; x<10; x++){
      rep = time_length/(speeds[x]);
      chase_wire(speeds[x],rep);
   }

   for (x=9; x>=0; x--){
      rep = time_length/(speeds[x]);
      chase_wire(speeds[x],rep);
   }

   digitalWrite(10, status);   // blink both status LEDs
   digitalWrite(13, status);
   status = !status; 

   //count1++;
   //grow(200);

   //if( count1 == 10 ) {
   //   count1 = 0;
   //}
}

int main(void) {
   init();
   setup();
   for (;;) {
      loop();
   }
}
