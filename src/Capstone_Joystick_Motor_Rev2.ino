//declare joystick pins (v denoting vertical and h denoting horizontal)
int joystickv = 3;
int joystickh = 2;

//setup variables to hold joystick status (v denoting vertical and h denoting horizontal)
int durationv;
int durationh;

//setup variables to hold desire motor power (L and R denoting left and right respectively)
int pwrL;
int pwrR;

// Motor connections
int in1 = 8;
int in2 = 7;
int en1 = 9;

int in1b = 6;
int in2b = 4;
int en1b = 5;

// Define Joystick Zones/Bounds
int joystick_low = 980;
int joystick_neutral = 1490;
int joystick_high = 2000;
int deadzone = 30;

void setup() {
  Serial.begin(9600); //open serial line to for debugging
  
  //set joystick pins as input
  pinMode(joystickv, INPUT);
  pinMode(joystickh, INPUT);

  // Set all the motor control pins to outputs
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(en1, OUTPUT);

  pinMode(in1b, OUTPUT);
  pinMode(in2b, OUTPUT);
  pinMode(en1b, OUTPUT);
}

void loop() {
  //get raw PWM signal
  durationv = pulseIn(joystickv, HIGH);
  durationh = pulseIn(joystickh, HIGH);

  /*
  //debug, display raw PWM signals
  Serial.println();
  Serial.print("raw_vert: ");
  Serial.println(durationv);
  Serial.print("raw_horz: ");
  Serial.println(durationh);
  */

  //convert PWM signal to range from -255 to 255
  durationv = map(durationv, joystick_low, joystick_high, -255, 255); //on my controller I have Ch1 inverted, thus the weird mapping
  durationh = map(durationh, joystick_low, joystick_high, 255, -255);

  //convert joystick signals to motor power
  //average accordingly
  pwrL = (durationv+durationh)/2;
  pwrR = (durationv-durationh)/2;
  
  //the motors only get full power if the joystick is in a corner, so we will double the signal and "lop off the top" so that when the stick is forward it is able to go at full power
  //yes, I know this is shoddy
  pwrL = constrain(2*pwrL,-255,255);
  pwrR = constrain(2*pwrR,-255,255);

  //debug, display prescribed motor power
  Serial.print("PwrL: ");
  Serial.println(pwrL);
  Serial.print("PwrR: ");
  Serial.println(pwrR);
  
  /*
  //debug, display conditioned PWM signals
  Serial.print("vert: ");
  Serial.println(durationv);
  Serial.print("horz: ");
  Serial.println(durationh);
  */
  
  //set the rotation direction of right motor
  if(pwrR > deadzone){
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else if(pwrR < -1*deadzone){
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else{
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }

  //set the rotation speed of right motor
  analogWrite(en1, abs(pwrR));


  //set the rotation direction of left motor
  if(pwrL > deadzone){
    digitalWrite(in1b, LOW);
    digitalWrite(in2b, HIGH);
  }
  else if(pwrL < -1*deadzone){
    digitalWrite(in1b, HIGH);
    digitalWrite(in2b, LOW);
  }
  else{
    digitalWrite(in1b, LOW);
    digitalWrite(in2b, LOW);
  }

  //set the rotation speed of left motor
  analogWrite(en1b, abs(pwrL));
  
}
