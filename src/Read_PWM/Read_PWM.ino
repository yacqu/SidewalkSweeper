//Refrence: https://www.theboredrobot.com/post/reading-values-from-an-rc-receiver-using-arduino

//define the pins and variables
#define RCPina 2
#define RCPinb 3
volatile long StartTimea = 0;
volatile long CurrentTimea = 0;
volatile long Pulsesa = 0;
int PulseWidtha = 0;

volatile long StartTimeb = 0;
volatile long CurrentTimeb = 0;
volatile long Pulsesb = 0;
int PulseWidthb = 0;

void setup() {
  //set up the serial monitor, pin mode, and external interrupt.
  Serial.begin(9600);
  pinMode(RCPina, INPUT_PULLUP);
  pinMode(RCPinb, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RCPina),PulseTimera,CHANGE);
  attachInterrupt(digitalPinToInterrupt(RCPinb),PulseTimerb,CHANGE);
}

void loop() {
  //only save pulse lengths that are less than 2000 microseconds
  if (Pulsesa < 2000){
    PulseWidtha = Pulsesa;
  }  
  Serial.print(PulseWidtha);

  if (Pulsesb < 2000){
    PulseWidthb = Pulsesb;
  }  
  Serial.print(",");
  Serial.println(PulseWidthb);
}


void PulseTimera(){
  //measure the time between interrupts
  CurrentTimea = micros();
  if (CurrentTimea > StartTimea){
    Pulsesa = CurrentTimea - StartTimea;
    StartTimea = CurrentTimea;
  }
}


void PulseTimerb(){
  //measure the time between interrupts
  CurrentTimeb = micros();
  if (CurrentTimeb > StartTimeb){
    Pulsesb = CurrentTimeb - StartTimeb;
    StartTimeb = CurrentTimeb;
  }
}
