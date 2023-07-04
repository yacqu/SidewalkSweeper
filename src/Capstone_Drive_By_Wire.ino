#include <Servo.h>
#include <AccelStepper.h>
//TO DO: optimize the size of these variables (for example ints for connections could be Bytes)
//TO DO: optionally make pwm levels an array so as to condense the four functions and streamline post processing

// Define stepper pin connections
const int dirPin = 52;
const int stepPin = 50;
// Creates an instance
AccelStepper myStepper(1, stepPin, dirPin);

//Brush Motor Connections
int in1 = 35;
int in2 = 33;
int en = 37;

//Left Encoder COnnections
int encLA = 10;
int encLB = 11;

//Right Encoder Connections
int encRA = 12;
int encRB = 13;

//Setup Variables for left encoders
int counterL = 0; 
int aStateL;
int aLastStateL;  

int countLA = 0;
int countLB = 0;

//Setup Variables for right encoders
int counterR = 0; 
int aStateR;
int aLastStateR;  

int countRA = 0;
int countRB = 0;

//define remote channels
int stick_ch1 = 2; // Left/Right
int stick_ch2 = 3; // FWD/Back
int stick_ch3 = 4; // Brush Motor (Pin TBD)
int stick_ch4 = 5; // Stepper Angle

//create variables to store radio channels pulsewidths
volatile unsigned long ch1Pulsewidth;
volatile unsigned long ch2Pulsewidth;
volatile unsigned long ch3Pulsewidth;
volatile unsigned long ch4Pulsewidth;

// Define Joystick Zones/Bounds
int joystick_low = 995;
int joystick_neutral = 1490;
int joystick_high = 1990;
int deadzone = 25;

//setup motor control lines as servos
Servo left_motor;  // create servo object to control left motor
Servo right_motor;  // create servo object to control right motor

//define "Servo" pins for left and right motors
int left_motor_pin = 6;
int right_motor_pin = 7;

void setup() {
  Serial.begin (115200); //open serial terminal (Debug)
  Serial.print("STARTUP!");

  //attach the servos entity to the correct pin and set to neutral (90 "degrees")
  left_motor.attach(left_motor_pin);
  right_motor.attach(right_motor_pin);
  left_motor.write(90);
  right_motor.write(90);
  delay(5000); //wait for motor controller bootup (this allows it to "zero" the pwm signal
  
  // Set all the brush motor control pins to outputs
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(en, OUTPUT);

  //set encoder lines as inputs
  pinMode(encLA, INPUT);
  pinMode(encLB, INPUT);
  pinMode(encRA, INPUT);
  pinMode(encRB, INPUT);

  //intialize encoder input/interrupt pins
  pinMode(stick_ch1, INPUT);              // Set the input pin
  attachInterrupt(digitalPinToInterrupt(stick_ch1), Readch1Pulsewidth, CHANGE);   // Run the calcPulsewidth function on signal CHANGE
  pinMode(stick_ch2, INPUT);              // Set the input pin
  attachInterrupt(digitalPinToInterrupt(stick_ch2), Readch2Pulsewidth, CHANGE);   // Run the calcPulsewidth function on signal CHANGE
  pinMode(stick_ch3, INPUT);              // Set the input pin
  attachInterrupt(digitalPinToInterrupt(stick_ch3), Readch3Pulsewidth, CHANGE);   // Run the calcPulsewidth function on signal CHANGE
  pinMode(stick_ch4, INPUT);              // Set the input pin
  attachInterrupt(digitalPinToInterrupt(stick_ch4), Readch4Pulsewidth, CHANGE);   // Run the calcPulsewidth function on signal CHANGE
  
  // Turn off brush motor - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(en, LOW);

  //Get initial state of encoders
  aLastStateL = digitalRead(encLA);
  aLastStateR = digitalRead(encRA);

  //Setup Stepper
  myStepper.setMaxSpeed(5000);
  myStepper.setAcceleration(5000);
  myStepper.setSpeed(0);
}

void loop() {
  int ch1 = readpwm(stick_ch1); //read Left/Right Channel
  int ch2 = readpwm(stick_ch2); //read Fwd/Back Channel
  int ch3 = readpwm(stick_ch3); //read Brush Motor Channel
  int ch4 = readpwm(stick_ch4); //read Stepper angle Channel

  myStepper.setSpeed(ch4*2); //set the stepper speed
  myStepper.runSpeed(); //update the stepper speed
  Serial.println(ch4);


  //print the post-processed PWM signals to console (DEBUG)
//  Serial.print(ch1);
//  Serial.print(",");
//  Serial.println(ch2);
//  
  //setup variables to hold motor power level (PLACEHOLDER VALUES)
  int pwrL = 0;
  int pwrR = 0;
  
  pwrL = ch2+ch1; //apply pwr and side bias to Left motor
  pwrR = ch2-ch1; //apply pwr and side bias to Right motor

  //constrain pwr levels and reverse
  pwrL = -1*constrain(pwrL, -255, 255);
  pwrR = -1*constrain(pwrR, -255, 255);

  pwrL = map(-1*pwrL,-255,255,20,160);
  pwrR = map(-1*pwrR,-255,255,20,160);
  
  left_motor.write(pwrL);
  right_motor.write(pwrR);
  
  brush_run(ch3); //send power level to brush motor
  //motor_run("R", pwrR); //send power level to right motor

  //print motor power to console (DEBUG)
  //Serial.print(pwrL);
  //Serial.print(",");
  //Serial.println(pwrR);
}

//setup  function to send power level to motors
//Takes char variable to select right or left motor. Takes int variable to determine motor power level
//Interprets these inputs then runs sends signals to motor controller
void brush_run(int pwr){
  //set direction
  if(pwr > 0){
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else{
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }

  //set the power level
  analogWrite(en, abs(pwr));
}

//setup function to read radio PWM lines (This is SUBOPTIMAL)
//Takes input of stick to check. Outputs the stick status as an interger
//reads the stick using pulsein then applies the deadzones to output a value between +/- 255
//TO DO: optimize the size of these variables
int readpwm(int stick){
  int duration = 0;
  if(stick == stick_ch1){//write the appropriate pulsewidth of the appropriate channel
    duration = ch1Pulsewidth; //read joystick 1 raw pwm signal
  }
  else if(stick == stick_ch2){
    duration = ch2Pulsewidth; //read joystick 2 raw pwm signal
  }
  else if(stick == stick_ch3){
    duration = ch3Pulsewidth; //read joystick 3 raw pwm signal
  }
  else if(stick == stick_ch4){
    duration = ch4Pulsewidth; //read joystick 4 raw pwm signal
  }
  
  int sign = 0; //create variable to store joystick direction

  //check direction of joystick
  if(duration < joystick_neutral){
    sign = -1;
  }
  if(duration > joystick_neutral){
    sign = 1;
  }
  
  duration = abs(map(duration, joystick_low, joystick_high, -255, 255)); //map the signal to range +/-255

  //apply deadzones to joystick 
  if(duration < deadzone){
    duration = 0;
  }

  duration =  duration*sign;
  return duration;
}


//setup functions to run on interrupt pins
//make function to increment left encodere
void incLA(){
  if(digitalRead(encLB) == HIGH){
    counterL++;
  }
  else{
    counterL--;
  }
}

//make function to increment right encoder
void incRA(){
  if(digitalRead(encRB) == HIGH){
    counterR++;
  }
  else{
    counterR--;
  }
}

//make function to find pulsewidth on radio channel 1
void Readch1Pulsewidth() //Source:https://forum.arduino.cc/t/pwm-reading-with-due/896453/5
{
  static unsigned long ch1StartTime;   // Start time variable
  
  if (digitalRead(stick_ch1) == HIGH)    // If the change was a RISING edge
  {
    ch1StartTime = micros();           // Store the start time (in microseconds)
  }
  else                                   // If the change was a FALLING edge
  {        
    ch1Pulsewidth = micros() - ch1StartTime;    // Calculate the pulsewidth
  }
}

//make function to find pulsewidth on radio channel 2
void Readch2Pulsewidth() //Source:https://forum.arduino.cc/t/pwm-reading-with-due/896453/5
{
  static unsigned long ch2StartTime;   // Start time variable
  
  if (digitalRead(stick_ch2) == HIGH)    // If the change was a RISING edge
  {
    ch2StartTime = micros();           // Store the start time (in microseconds)
  }
  else                                   // If the change was a FALLING edge
  {        
    ch2Pulsewidth = micros() - ch2StartTime;    // Calculate the pulsewidth
  }
}

//make function to find pulsewidth on radio channel 3
void Readch3Pulsewidth() //Source:https://forum.arduino.cc/t/pwm-reading-with-due/896453/5
{
  static unsigned long ch3StartTime;   // Start time variable
  
  if (digitalRead(stick_ch3) == HIGH)    // If the change was a RISING edge
  {
    ch3StartTime = micros();           // Store the start time (in microseconds)
  }
  else                                   // If the change was a FALLING edge
  {        
    ch3Pulsewidth = micros() - ch3StartTime;    // Calculate the pulsewidth
  }
}

//make function to find pulsewidth on radio channel 4
void Readch4Pulsewidth() //Source:https://forum.arduino.cc/t/pwm-reading-with-due/896453/5
{
  static unsigned long ch4StartTime;   // Start time variable
  
  if (digitalRead(stick_ch4) == HIGH)    // If the change was a RISING edge
  {
    ch4StartTime = micros();           // Store the start time (in microseconds)
  }
  else                                   // If the change was a FALLING edge
  {        
    ch4Pulsewidth = micros() - ch4StartTime;    // Calculate the pulsewidth
  }
}
