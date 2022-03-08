//Import libraries:
#include <Stepper.h>
#include <Servo.h>
#include <Arduino.h>
#include "BasicStepperDriver.h"
#define MOTOR_STEPS 200
#define RPM 30
#define MICROSTEPS 1

//pin map
const int dir1Pin = 2;
const int dir2Pin = 3;
const int sleepPin = 4;
const int step1Pin = 5;
const int step2Pin = 6;
const int servo1Pin = 8;
const int stepsPerRev = 200;
const int I1 = 9; //current limit
const int I2 = 10;
const int servo2Pin = 11;
const int servo3Pin = 12;
const int servo4Pin = 13;

//dynamic vars
int delta = 0;

//Create objects
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

BasicStepperDriver stepper1(MOTOR_STEPS, dir1Pin, step1Pin);
BasicStepperDriver stepper2(MOTOR_STEPS, dir2Pin, step2Pin);

//Set up variables
float turnConst;
float ticksPermm;
int pulseWidthMicros = 20;  // microseconds
int millisbetweenSteps = 5; // milliseconds - or try 1000 for slower steps

long watchdog;
long sleepDelay;

const byte numChars = 8; //max array size for incoming serial data
char receivedChars[numChars]; //buffer to receive serial chars
float val = 0.0;
uint8_t dir = 0;
boolean newData = false;


/*******************Servo functions*******************/
void attachServos() {

  if (!servo1.attached()) {
    servo1.attach(servo1Pin);
  }
  if (!servo2.attached()) {
    servo2.attach(servo2Pin);
  }
  if (!servo3.attached()) {
    servo3.attach(servo3Pin);
  } if (!servo4.attached()) {
    servo4.attach(servo4Pin);
  }
}
void detachServos() {
  if (servo1.attached()) {
    servo1.detach();
  }
  if (servo2.attached()) {
    servo2.detach();
  }
  if (servo3.attached()) {
    servo3.detach();
  }
  if (servo4.attached()) {
    servo4.detach();
  }
}
void servoTurn() {
  attachServos();
  servo1.write(50);
  servo2.write(50);
  servo3.write(95);
  servo4.write(95);
  delay(250);
}
void servoStraight() {
  attachServos();
  servo1.write(5);
  servo2.write(5);
  servo3.write(140);
  servo4.write(140);
  delay(250);
}

/*******************Stepper config functions*******************/

void dirFwd() {
  digitalWrite(dir1Pin, HIGH);
  digitalWrite(dir2Pin, LOW);
}
void dirRight() {
  digitalWrite(dir1Pin, HIGH);
  digitalWrite(dir2Pin, HIGH);
}
void dirLeft() {
  digitalWrite(dir1Pin, LOW);
  digitalWrite(dir2Pin, LOW);
}

/*******************Communication functions*******************/
void parseData() {

  // split the data into its parts

  char * strtokIndx; // this is used by strtok() as an index

  strtokIndx = strtok(receivedChars, ",");     // get the first part - the string
  //Serial.println(strtokIndx);
  dir = atoi(strtokIndx);
  //Serial.println(dir);

  strtokIndx = strtok(NULL, ",");
  val = atof(strtokIndx);     // convert this part to a float
  newData = true;

}
void recvWithEndMarker() {
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;

  // if (Serial.available() > 0) {
  while (Serial1.available() > 0 && newData == false) {
    rc = Serial1.read();

    if (rc != endMarker) {
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= numChars) {
        ndx = numChars - 1;
      }
    }
    else {
      receivedChars[ndx] = '\0'; // terminate the string
      ndx = 0;
      parseData();
    }
  }
}


/*******************Drive functions*******************/

void drive() {

  //set drive parameters
  if (dir == 'w') {
    servoStraight();
    dirFwd();
    delta = int(val * ticksPermm);
    //    return true;
  }
  else if (dir == 'a' || dir == 'd') {
    servoTurn();
    delta = int(val * turnConst);

    if (dir == 'a') {
      //Serial.println("turning left");
      dirLeft();
    }
    else if (dir == 'd') {
      dirRight();
    }
  }
  else { //ignore invalid cmds
    goToSleep();
    return;
  }

  for (int i = 0; i < delta; i++) {
    digitalWrite(step1Pin, HIGH);
    digitalWrite(step2Pin, HIGH);
    delayMicroseconds(pulseWidthMicros);
    digitalWrite(step1Pin, LOW);
    digitalWrite(step2Pin, LOW);
    delay(millisbetweenSteps);
  }
}

void goToSleep() {
  Serial1.println('r');
  digitalWrite(sleepPin, LOW);
  detachServos();
  watchdog = millis();
}
void wakeUp() {
  digitalWrite(sleepPin, HIGH);
  attachServos();
  watchdog = millis();
}
/*******************Setup functions*******************/
void setGeometryConsts() {
  float wheelDiam = 65;
  float wheelCircum = wheelDiam * PI;
  float wheelbaseRadius = 175;

  ticksPermm = MOTOR_STEPS / wheelCircum;
  turnConst = ticksPermm * wheelbaseRadius;
}
void setup() {

  //Power saving 
  for (int i = 14; i <= 19; i++){
    pinMode(i, OUTPUT);
    digitalWrite(i, LOW);
  }
  // disable ADC
  ADCSRA = 0;  

  Serial1.begin(9600);
  setGeometryConsts();

  watchdog = millis();
  sleepDelay = millis();

  attachServos();
  servoStraight();
  while (false) {
    servoTurn();
    delay(3000);
    servoStraight();
    delay(3000);
  }

  pinMode(dir1Pin , OUTPUT);
  pinMode(dir2Pin , OUTPUT);
  pinMode(sleepPin , OUTPUT);
  pinMode(step1Pin , OUTPUT);
  pinMode(step2Pin , OUTPUT);
  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);
  servo3.attach(servo3Pin);
  servo4.attach(servo4Pin);

  /* Stepper driver current limit configs
   * I1   I2  Current Limit
   * Z     Z     0.5 A
   * Low   Z     1 A
   * Z     Low   1.5 A
   * Low   Low   2 A
   * 
   * For finer control, an analog voltage or a PWM can be supplied to the I1 pin while I2 is driven low:
   */

  //1A limit:
  pinMode(I1, OUTPUT);
  pinMode(I2, INPUT);
  digitalWrite(I1, LOW);
  
//  // custom limit:
//  pinMode(I1, OUTPUT);
//  pinMode(I2, OUTPUT);
//  digitalWrite(I1, LOW);
//  float desiredLimit =
//  float maxCurrent = 2;
//  int pwmVal = int((desiredLimit / maxCurrent) * 255);
//  digitalWrite(I1, pwmVal);
//  
  digitalWrite(sleepPin, HIGH);
  stepper1.begin(RPM, MICROSTEPS);
  stepper2.begin(RPM, MICROSTEPS);
  stepper1.enable();
  stepper2.enable();

  goToSleep();

}

void loop() {

  if (millis() - watchdog >= 5000) { //30 second timer
    goToSleep(); //handshake, reset watchdog
  }
  recvWithEndMarker();
  if (newData) {
    //Serial.println(dir);
    //Serial.println(val);
    newData = false;
    wakeUp();

    drive();
    goToSleep();
  }
}
