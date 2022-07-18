#include <arduino-timer.h>

// Define input and output pins for cascading communication
const int inSignal = 2;   //D2 INT0 interrupt pin
const int outSignal = 3;  //D3
const int motorPin = 5;   //PWM pin for ERM vibration motor
//bool demoSwitch = false;       // on off switch for demonstration

#define DUTYCYCLE20  map(20, 0, 100, 0, 255)

auto timer = timer_create_default();

void setup() 
{
  pinMode(inSignal, INPUT);
  pinMode(outSignal, OUTPUT);
  pinMode(motorPin, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(inSignal), turnOnMotor, RISING);

  digitalWrite(outSignal, LOW);
  analogWrite(motorPin, 0);

  Serial.begin(9600);
}

void turnOnMotor()
{
  timer.in(500, turnOffMotor);         //set timer [ms]
  analogWrite(motorPin, DUTYCYCLE20);  //turn on motor to 20% duty cycle
  digitalWrite(outSignal, LOW);        //reset outSignal to get ready for next trigger
}

void turnOffMotor()
{
  analogWrite(motorPin, 0);
  digitalWrite(outSignal, HIGH);
}

void loop() 
{
  timer.tick();
  //Check for serial input to turn motor on/off
  if (Serial.available()) 
  {
    Serial.read();
    turnOnMotor();
  }
}
