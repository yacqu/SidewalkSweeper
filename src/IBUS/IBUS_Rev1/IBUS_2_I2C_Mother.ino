//Refrence: https://thewanderingengineer.com/2015/05/06/sending-16-bit-and-32-bit-numbers-with-arduino-i2c/

//This code is intedned to run on an arduino DUE. It reads the byte array that it requests from address 1 of the i2c port, converts the array to an interger then prints it to the terminal

#include <Wire.h>
 
void setup()
{
  Wire.begin(); //startup i2c
  Serial.begin(115200); //startup serial terminal
}
 
void loop()
{
  delay(1); //delay 1ms (this can be lenthened)
   
  int16_t bigNum; //create a 16 bit interger to store the recieved data
  byte a,b; //create 2 byte variables in order to store the raw i2c message
  
  Wire.requestFrom(1,2); //request 2 bytes of data from address 1
   
  a = Wire.read(); //write the first byte to a
  b = Wire.read(); //write the second byte to b

  //convert the two bytes to an interger
  bigNum = a;
  bigNum = bigNum << 8 | b;

  //print the interger
  Serial.print(bigNum);
  Serial.print("\n");
}
