#include <SPI.h>
volatile int process = 0;
byte i=0, variable=0;
byte c,x;

void setup (void) {
   Serial.begin (115200);
   pinMode(MISO, OUTPUT); // have to send on master in so it set as output
   SPCR |= _BV(SPE); // turn on SPI in slave mode
   process = false;
   SPI.attachInterrupt(); // turn on interrupt
}

ISR (SPI_STC_vect){ // SPI interrupt routine
   c = SPDR; // read byte from SPI Data Register
   SPDR = i;
   process ++;
   variable++;
}

void loop (void) {
   if(process==1)
   {
      x = c<<8;
   }
   if(process==2)
   {
      process = 0;
      x = x|c;
      if (variable == 1)
      {
        Serial.print("\nAccX = ");
        Serial.print(x);
      }
      else if (variable == 2)
      {
        Serial.print("  AccY = ");
        Serial.print(x);
      }
      else if (variable == 3)
      {
        Serial.print("  AccZ = ");
        Serial.print(x);
      }
      else if (variable == 4)
      {
        Serial.print("  Temp = ");
        Serial.print(x);
      }
      else if (variable == 5)
      {
        Serial.print("  GyroX = ");
        Serial.print(x);
      }
      else if (variable == 6)
      {
        Serial.print("  GyroY = ");
        Serial.print(x);
      }
      else if (variable == 7)
      {
        variable = 0;
        Serial.print("  GyroZ = ");
        Serial.print(x);
      }
   }
}

