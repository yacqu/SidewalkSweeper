//Refrence: https://thewanderingengineer.com/2015/05/06/sending-16-bit-and-32-bit-numbers-with-arduino-i2c/

//This code is intedned to run on an arduino DUE. It reads the byte array that it requests from address 1 of the i2c port, converts the array to an interger then prints it to the plotter

//REV2: Altered to run two channels

#include <Wire.h>
 
void setup()
{
  Wire.begin(); //startup i2c
  Serial.begin(115200); //startup serial terminal
}
 
void loop()
{
  delay(1); //delay 1ms (this can be lenthened)
   
  int16_t ch1,ch2; //create a 16 bit interger to store the recieved data
  byte a,b,c,d; //create 4 byte variables in order to store the raw i2c message
  
  Wire.requestFrom(1,4); //request 2 bytes of data from address 1
   
  a = Wire.read(); //write the first byte to a
  b = Wire.read(); //write the second byte to b
  c = Wire.read(); //write the third byte to c
  d = Wire.read(); //write the fourth byte to d

  //convert the two bytes to an interger
  ch1 = a;
  ch1 = ch1 << 8 | b;

  ch2 = c;
  ch2 = ch2 << 8 | d;

  //print the intergers to serial plotter
  Serial.print(ch1); //the first variable for plotting
  Serial.print(","); //add seperator
  Serial.println(ch2);
}
