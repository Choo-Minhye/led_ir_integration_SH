#include <IRremote.h>

IRsend irSender;    //Left Transmitter: pin 3
IRsend1 irSender1;  //Center Transmitter: pin 9
IRsend2 irSender2;  //Right Transmitter: pin 10

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  irSender.sendNEC(0xFF629D, 32); 
  delay(25);
  irSender1.sendNEC(0xFFE21D, 32);
  delay(25);
  irSender2.sendNEC(0xFFA25D, 32);
  delay(25);
} 
