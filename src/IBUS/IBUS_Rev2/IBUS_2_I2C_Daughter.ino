//Derived from slave_sender, IBUS2PWM example sketches and the refrence below
//Refrence: https://thewanderingengineer.com/2015/05/06/sending-16-bit-and-32-bit-numbers-with-arduino-i2c/

//REV2: Altered to run two channels

//This code is intedned to run on an arduino NANO. It reads the status of IBUS channel 1 as an interger, converts it into an array of two bytes, then sends it on i2c address 1 when requested
#include <Wire.h>
#include <IBusBM.h>

IBusBM IBus; //setup IBus object

 
void setup()
{
  Wire.begin(1); //startup i2c client on address 1
  Wire.onRequest(requestEvent); //on i2c request run the request event function
  IBus.begin(Serial);    // iBUS connected to Serial0 - change to Serial1 or Serial2 port when required
 }
 
void loop()
{
  delay(100); //do nothing
}
 
void requestEvent()
{
  int16_t ch1 = IBus.readChannel(0); // get latest value from remote control channel 1 and write it to a 16 bit interger
  int16_t ch2 = IBus.readChannel(1); // get latest value from remote control channel 2 and write it to a 16 bit interger
  byte myArray[4]; //setup a byte array with four entries
   
  myArray[0] = (ch1 >> 8) & 0xFF; //set array entry 0 to the first byte of the ch1
  myArray[1] = ch1 & 0xFF; //set array entry 1 to the second byte of the ch1

  myArray[2] = (ch2 >> 8) & 0xFF; //set array entry 2 to the first byte of the ch2
  myArray[3] = ch2 & 0xFF; //set array entry 3 to the second byte of the ch2
  Wire.write(myArray, 4); //send the array as 4 bytes
}
