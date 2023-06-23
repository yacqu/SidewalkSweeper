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

// Define Joystick Zones/Bounds
int joystick_low = 995;
int joystick_neutral = 1490;
int joystick_high = 1990;
int deadzone = 25;

void setup() {
  Serial.begin (115200); //open serial terminal (Debug)
  Serial.print("STARTUP!");
  
  // Set all the motor control pins to outputs
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

  //intialize interrupt pins
  //attachInterrupt(digitalPinToInterrupt(encLA), incLA, RISING);
  //attachInterrupt(digitalPinToInterrupt(encLB), incLB, RISING);

  //attachInterrupt(digitalPinToInterrupt(encRA), incRA, RISING);
  //attachInterrupt(digitalPinToInterrupt(encRB), incRB, RISING);
  
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
  //int ch2 = readpwm(stick_ch2); //read Fwd/Back Channel
  int ch2 = 0;
  
  //Serial.print("pwrL: ");
  //Serial.println(ch1);
  //Serial.print("pwrR: ");
  //Serial.println(ch2);
  
  //setup variables to hold motor power level (PLACEHOLDER VALUES)
  int pwrL = 0;
  int pwrR = 0;

  /*
  int side_sign
  if(ch1 >= 0){
    side_sign = 1;
  }
  else if(ch1 < 0){
    side_sign = -1;
  }
  float side = map(abs(ch1), 0, 255, 1, 0.5); //remap absolute value of the ch1 signal to 1-0.5
  side = side_sign*side; //reattach the sign to side. (So, if its at neutral it should = 1
  */
  
  pwrL = ch1+ch2; //apply pwr and side bias to Left motor
  pwrR = ch1-ch2; //apply pwr and side bias to Right motor

  //constrain pwr levels
  pwrL = constrain(pwrL, 0, 255);
  pwrR = constrain(pwrR, 0, 255);
  
  motor_run("L", pwrL); //send power level to left motor
  motor_run("R", pwrR); //send power level to right motor

  /*
  Serial.print("pwrL: ");
  Serial.println(pwrL);
  Serial.print("pwrR: ");
  Serial.println(pwrR);*/
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
  int duration = pulseIn(stick, HIGH); //read joystick raw pwm signal
  int sign; //create variable to store joystick direction

  Serial.println(duration); //problems start here
  
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
void incLA(){
  if(digitalRead(encLB) == HIGH){
    counterL++;
  }
  else{
    counterL--;
  }
}


void incRA(){
  if(digitalRead(encRB) == HIGH){
    counterR++;
  }
  else{
    counterR--;
  }
}
