#include <Servo.h>
#include <AccelStepper.h>
//TO DO: optimize the size of these variables (for example ints for connections could be Bytes)

// Define stepper pin connections
const int dirPin = 52;
const int stepPin = 50;
const int stepen = 48; //set stpper enable pin
// Creates an instance
AccelStepper myStepper(1, stepPin, dirPin);

//Brush Motor Connections
int in1 = 35;
int in2 = 33;
int en = 37;

//Left Encoder Connections
int encLA = 10;
int encLB = 11;

//Right Encoder Connections
int encRA = 13;
int encRB = 12;

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

int motor_const = 188; //define encoder counts per revolution (found empirically)

//define remote channels
int stick_ch1 = 2; // Left/Right
int stick_ch2 = 3; // FWD/Back
int stick_ch3 = 4; // Autonomous Trigger
int stick_ch4 = 5; // Stepper Angle
int stick_ch5 = 6; // Brush Motor
int stick_ch6 = 27; //speed limiter pin

//create variables to store radio channels pulsewidths
volatile unsigned long ch1Pulsewidth;
volatile unsigned long ch2Pulsewidth;
volatile unsigned long ch3Pulsewidth;
volatile unsigned long ch4Pulsewidth;
volatile unsigned long ch5Pulsewidth;
volatile unsigned long ch6Pulsewidth;

int ch3_last = 0;

// Define Joystick Zones/Bounds
int joystick_low = 995;
int joystick_neutral = 1490;
int joystick_high = 1990;
int deadzone = 25;

//setup motor control lines as servos
Servo left_motor;  // create servo object to control left motor
Servo right_motor;  // create servo object to control right motor

//define "Servo" pins for left and right motors
int left_motor_pin = 8;
int right_motor_pin = 9;

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
  pinMode(stick_ch5, INPUT);              // Set the input pin
  attachInterrupt(digitalPinToInterrupt(stick_ch5), Readch5Pulsewidth, CHANGE);   // Run the calcPulsewidth function on signal CHANGE
  pinMode(stick_ch6, INPUT);              // Set the input pin
  attachInterrupt(digitalPinToInterrupt(stick_ch6), Readch6Pulsewidth, CHANGE);   // Run the calcPulsewidth function on signal CHANGE

  //attach interrupt functions to encoder pins
  attachInterrupt(digitalPinToInterrupt(encLA), incLA, RISING);
  attachInterrupt(digitalPinToInterrupt(encRA), incRA, RISING);
  
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
  pinMode(stepen, OUTPUT);

  Serial.print("Setup Complete");
}

void loop() {
  int ch1 = readpwm(stick_ch1); //read Left/Right Channel
  int ch2 = readpwm(stick_ch2); //read Fwd/Back Channel
  int ch3 = readpwm(stick_ch3); //read Autonomous Trigger Channel
  int ch4 = readpwm(stick_ch4); //read Stepper angle Channel
  int ch5 = readpwm(stick_ch5); //read Brush motor Channel
  int ch6 = readpwm(stick_ch6); //read Brush motor Channel

  //calculate the speed limit factor
  float fwd_speed_limit = map(ch6,-270,270,8,100)*0.01; //map only works with intergers so we multiply our desired range by 100 (50-100) then divide it at the end to the desired a decimal (0.5-1.0)
  float rev_speed_limit = map(ch6,-270,270,30,100)*0.01;
  
  if(ch3 > 0){  //check for autonomous trigger
    if(ch3_last == 0){ //I don't know why this didn't work as an AND statement with the previous line
      ch3_last = 1; //set the state tracking variable so it only triggers once per switch on
      Serial.println("Autonomous Triggered!");
      forward(4); //drive forward 4 feet
      turn(180); //turn 180 degrees
      forward(4); //drive forward 4 feet
      turn(180); //turn 180 degrees
      Serial.println("Autonomous Complete!");
    }
  }
  else{
    ch3_last = 0; //reset the state tracking variable
  }

  //print encoder statuses (DEBUG)
  //Serial.print(counterL);
  //Serial.print(",");
  //Serial.println(counterR);

  if(abs((ch4-joystick_neutral)) > deadzone){ //if the signal varies from  neutral by more than the deadzone, send a speed
    digitalWrite(stepen,HIGH); //enable the stepper
    myStepper.setSpeed(ch4*2); //set the stepper speed
    myStepper.runSpeed(); //update the stepper speed
  }
  else{ //otherwise send nothing
    digitalWrite(stepen,LOW); //disable the stepper
    myStepper.setSpeed(0); //set the stepper speed
    myStepper.runSpeed(); //update the stepper speed
  }
  
  //Serial.println(ch4);
  
  //print the post-processed PWM signals to console (DEBUG)
//  Serial.print(ch1);
//  Serial.print(",");
//  Serial.println(ch2);
  
  //setup variables to hold motor power level (PLACEHOLDER VALUES)
  int pwrL = 0;
  int pwrR = 0;
  
  pwrR = ch2-ch1; //apply pwr and side bias to Left motor
  pwrL = ch2+ch1; //apply pwr and side bias to Right motor

  //constrain pwr levels and reverse
  pwrR = constrain(pwrR, -255, 255);
  pwrL = constrain(pwrL, -255, 255);

  //Apply power limits, nees if statement so as to apply different coefficients for fwd and reverse
  if(pwrR > 0){
    pwrR = int(pwrR*fwd_speed_limit);
  }
  else{
    pwrR = int(pwrR*rev_speed_limit);
  }
  if(pwrL > 0){
    pwrL = int(pwrL*fwd_speed_limit);
  }
  else{
    pwrL = int(pwrL*rev_speed_limit);
  }

  //remap power levels to motor controller range
  pwrR = map(pwrR,-255,255,20,160);
  pwrL = map(pwrL,-255,255,20,160);
  
  left_motor.write(pwrL);
  right_motor.write(pwrR);
  
  brush_run(ch5); //send power level to brush motor
  
  //motor_run("R", pwrR); //send power level to right motor

  //print motor power to console (DEBUG)
  Serial.print(pwrL);
  Serial.print(",");
  Serial.println(pwrR);
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

//create function to drive forward at a set distance (Dictated by argument)
void forward(int dist){
  if(readpwm(stick_ch3) > 0){ //do the follwoing if autonomous is still enabled
    counterL = 0;
    counterR = 0;
  
    //Serial.println(dist); //DEBUG
    float target = dist * motor_const *12 /(3.1415 * 8); //calculate the encoder turns (explained in next line)
    // multiply the desired length * pi * wheel diameter converted to feet, this gives desired number of rotations. Multiply this by motor constant to get encoder counts required for the distance
    //Serial.println(target); //DEBUG
  
    while(((counterL + counterR)/2) < target){ //while the average encoder counts are less than the target, do the following
       if(readpwm(stick_ch3) < 0){ //check that autoonomous is still enabled
        right_motor.write(90);
        left_motor.write(90);
        break;
       }
       if(counterL == counterR){ //if the encoders are equal (going straight), set to equal speed
        left_motor.write(110);
        right_motor.write(110);
       }
       else if(counterL > counterR){ //if left side is higher (turning right), slow down left side
        left_motor.write(100);
        right_motor.write(110);
       }
       else if(counterL < counterR){ //if right side is higher (turning left), slow down right side
        left_motor.write(110);
        right_motor.write(100);
       }
    }
  }      
  //stop motors if autonomous is still enabled
  if(readpwm(stick_ch3) > 0){
    left_motor.write(20);
    right_motor.write(20);
    delay(350);
    left_motor.write(90);
    right_motor.write(90);
    delay(500);
  }
  else{ //if autonomous is disabled, reset motors
    right_motor.write(90);
    left_motor.write(90);
  }
}

//create function to turn right at a set angle (Dictated by argument)
void turn(int angle){
  if(readpwm(stick_ch3) > 0){
    //reset counters
    counterL = 0;
    counterR = 0;
      
    right_motor.write(50);
    while(abs(counterL-counterR) < 2.17*angle){
      if(readpwm(stick_ch3) < 0){ //check that autoonomous is still enabled
        right_motor.write(90);
        left_motor.write(90);
        break;
       }
      left_motor.write(115);
    }
    
    //stop motors if autonomous is still enabled
    if(readpwm(stick_ch3) > 0){
      right_motor.write(100);
      delay(100);
      left_motor.write(20);
      right_motor.write(20);
      delay(300);
      right_motor.write(90);
      left_motor.write(90);
      delay(500);
    }
  }
  else{ //if autonomous is disabled, reset motors
    right_motor.write(90);
    left_motor.write(90);
  }
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
  else if(stick == stick_ch5){
    duration = ch5Pulsewidth; //read joystick 4 raw pwm signal
  }
  else if(stick == stick_ch6){
    duration = ch6Pulsewidth; //read joystick 4 raw pwm signal
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

//make function to find pulsewidth on radio channel 5
void Readch5Pulsewidth() //Source:https://forum.arduino.cc/t/pwm-reading-with-due/896453/5
{
  static unsigned long ch5StartTime;   // Start time variable
  
  if (digitalRead(stick_ch5) == HIGH)    // If the change was a RISING edge
  {
    ch5StartTime = micros();           // Store the start time (in microseconds)
  }
  else                                   // If the change was a FALLING edge
  {        
    ch5Pulsewidth = micros() - ch5StartTime;    // Calculate the pulsewidth
  }
}

//make function to find pulsewidth on radio channel 6
void Readch6Pulsewidth() //Source:https://forum.arduino.cc/t/pwm-reading-with-due/896453/5
{
  static unsigned long ch6StartTime;   // Start time variable
  
  if (digitalRead(stick_ch6) == HIGH)    // If the change was a RISING edge
  {
    ch6StartTime = micros();           // Store the start time (in microseconds)
  }
  else                                   // If the change was a FALLING edge
  {        
    ch6Pulsewidth = micros() - ch6StartTime;    // Calculate the pulsewidth
  }
}
