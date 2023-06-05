int pin = 2;
int pinb = 3;
unsigned long duration;
unsigned long durationb;

// Motor connections
int in1 = 8;
int in2 = 7;
int en1 = 9;

int in1b = 6;
int in2b = 4;
int en1b = 5;

// Define Joystick Zones/Bounds
int joystick_low = 995;
int joystick_neutral = 1490;
int joystick_high = 1990;
int deadzone = 25;

int pwr = 0;
int pwrb = 0;

void setup() {
  Serial.begin(9600);
  pinMode(pin, INPUT);
  pinMode(pinb, INPUT);

  // Set all the motor control pins to outputs
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(en1, OUTPUT);

  pinMode(in1b, OUTPUT);
  pinMode(in2b, OUTPUT);
  pinMode(en1b, OUTPUT);
}

void loop() {
  duration = pulseIn(pin, HIGH);
  durationb = pulseIn(pinb, HIGH);

  //set the rotation direction
  if(duration > joystick_neutral+deadzone){
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else if(duration < joystick_neutral-deadzone){
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else{
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }

  //set the rotation speed
  pwr = abs(map(duration, joystick_low, joystick_high, -255, 255));
  analogWrite(en1, pwr);


  //set the rotation direction
  if(durationb > joystick_neutral+deadzone){
    digitalWrite(in1b, LOW);
    digitalWrite(in2b, HIGH);
  }
  else if(durationb < joystick_neutral-deadzone){
    digitalWrite(in1b, HIGH);
    digitalWrite(in2b, LOW);
  }
  else{
    digitalWrite(in1b, LOW);
    digitalWrite(in2b, LOW);
  }

  //set the rotation speed
  pwrb = abs(map(durationb, joystick_low, joystick_high, -255, 255));
  analogWrite(en1b, pwrb);
  
  Serial.println(duration);
}
