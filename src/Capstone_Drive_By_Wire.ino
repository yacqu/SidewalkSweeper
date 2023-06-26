//TO DO: optimize the size of these variables (for example ints for connections could be Bytes)

//TO DO: Use interrupts for reading PWM. Pulsein is not functional on a DUE
// Left Motor Connections
int inR1 = 5;
int inR2 = 6;
int enR = 4;

//Right Motor Connections
int inL1 = 7;
int inL2 = 8;
int enL = 9;

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

//create variables to store radio channels pulsewidths
volatile unsigned long ch1Pulsewidth;
volatile unsigned long ch2Pulsewidth;

// Define Joystick Zones/Bounds
int joystick_low = 995;
int joystick_neutral = 1490;
int joystick_high = 1990;
int deadzone = 25;

// adding stepper motor code
#include <AccelStepper.h>

// Define pin connections
const int dirPin = 52;  // white wire
const int stepPin = 50;  // blue wire

// Define motor interface type
#define motorInterfaceType 1

// Creates an instance
AccelStepper myStepper(motorInterfaceType, stepPin, dirPin);

void setup() {
  Serial.begin (115200); //open serial terminal (Debug)
  Serial.print("STARTUP!");
  
  // Set all the motor control pins to outputs
  pinMode(10, INPUT_PULLUP);
  pinMode(inL1, OUTPUT);
  pinMode(inL2, OUTPUT);
  pinMode(enL, OUTPUT);
  pinMode(encLA, INPUT);
  pinMode(encLB, INPUT);

  pinMode(inR1, OUTPUT);
  pinMode(inR2, OUTPUT);
  pinMode(enR, OUTPUT);
  pinMode(encRA, INPUT);
  pinMode(encRB, INPUT);

  //intialize encoder input/interrupt pins
  pinMode(stick_ch1, INPUT);              // Set the input pin
  attachInterrupt(digitalPinToInterrupt(stick_ch1), Readch1Pulsewidth, CHANGE);   // Run the calcPulsewidth function on signal CHANGE
  pinMode(stick_ch2, INPUT);              // Set the input pin
  attachInterrupt(digitalPinToInterrupt(stick_ch2), Readch2Pulsewidth, CHANGE);   // Run the calcPulsewidth function on signal CHANGE
  
  // Turn off motors - Initial state
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, LOW);
  digitalWrite(enL, LOW);

  digitalWrite(inR1, LOW);
  digitalWrite(inR2, LOW);
  digitalWrite(enR, LOW);

  //Get initial state of encoders
  aLastStateL = digitalRead(encLA);
  aLastStateR = digitalRead(encRA);

  //setup joystick pins as input
  pinMode(stick_ch1, INPUT);
  pinMode(stick_ch2, INPUT);
}

void loop() {
  int ch1 = readpwm(stick_ch1); //read Left/Right Channel
  int ch2 = readpwm(stick_ch2); //read Fwd/Back Channel

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
  
  motor_run("L", pwrL); //send power level to left motor
  motor_run("R", pwrR); //send power level to right motor

  // run the stepper motor with no user input
  // TODO: interface with the remotor controller
  runStepper()

  //print motor power to console (DEBUG)
  Serial.print(pwrL);
  Serial.print(",");
  Serial.println(pwrR);
}

//setup  function to send power level to motors
//Takes char variable to select right or left motor. Takes int variable to determine motor power level
//Interprets these inputs then runs sends signals to motor controller
void motor_run(String motor, int pwr){
  byte in1,in2,en; //setup local pin variables

  //check which motor to run and write it to the local pin variables
  if(motor == "L"){
    in1 = inL1;
    in2 = inL2;
    en = enL;
  }
  else if(motor == "R"){
    in1 = inR1;
    in2 = inR2;
    en = enR;
  }


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


void runStepper() {
    // set the maximum speed, acceleration factor,
  // initial speed and the target position
  
  myStepper.setMaxSpeed(1000);
  myStepper.setAcceleration(50);
  myStepper.setSpeed(600);
  myStepper.moveTo(400);
  
  // Change direction once the motor reaches target position
  if (myStepper.distanceToGo() == 0) 
    myStepper.moveTo(-myStepper.currentPosition());
  // Move the motor one step
  myStepper.run();
  
  while (digitalRead(10) == LOW) {
  
  // Do nothing 
  }
}

