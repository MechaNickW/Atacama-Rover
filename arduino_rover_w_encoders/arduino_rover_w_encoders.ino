//Import libraries:
#include <ServoTimer2.h>
#include <PID_v2.h>
#include <Encoder.h>

//Create objects
ServoTimer2 servo1;
ServoTimer2 servo2;
Encoder rEnc(3, 6);
Encoder lEnc(2, 5);


//motor shield pin map
int nD2 = 4; //Tri-state disables both outputs of both motor channels when LOW;
int M1DIR = 7; //Motor 1 direction input
int M1PWM = 9; //Motor 1 speed input
int M2DIR = 8; //Motor 2 direction input
int M2PWM = 10; //Motor 2 speed input

// Specify the links and initial tuning parameters for PID
double Kp = .3, Ki = .0, Kd = .03;
PID_v2 lPID(Kp, Ki, Kd, PID::Direct);
PID_v2 rPID(Kp, Ki, Kd, PID::Direct);

//Set up variables
long lSetpoint;
long rSetpoint;
float turnConst;
float ticksPermm;

boolean turn = false;
boolean stopped = false;
const byte numChars = 8; //max array size for incoming serial data
char receivedChars[numChars]; //buffer to receive serial chars
float val = 0.0;
char dir = 0;
boolean newData = false;

boolean moveComplete = false;

void attachServos() {
  if (!servo1.attached()) {
    servo1.attach(11);
  }
  if (!servo2.attached()) {
    servo2.attach(13);
  }
}
void detachServos() {
  if (servo1.attached()) {
    servo1.detach();
  }
  if (servo2.attached()) {
    servo2.detach();
  }
}
void servoTurn() {
  turn = true;
  attachServos();
  servo1.write(1667); // 11
  servo2.write(1200); // 13
  delay(100);
}
void servoStraight() {
  turn = false;
  attachServos();
  servo1.write(768);
  servo2.write(2167);
  delay(100);
}
void dontMove() {
  analogWrite(M1PWM, 0);
  analogWrite(M2PWM, 0);
  detachServos();
}
void recvWithEndMarker() {
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;

  // if (Serial.available() > 0) {
  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

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
void parseData() {

  // split the data into its parts

  char * strtokIndx; // this is used by strtok() as an index

  strtokIndx = strtok(receivedChars, ",");     // get the first part - the string
  dir = receivedChars[0];

  strtokIndx = strtok(NULL, ",");
  val = atof(strtokIndx);     // convert this part to a float
  newData = true;

}
void setEncoderTargets(double lPosition, double rPosition) {
  Serial.println("setting encoder targets");
  Serial.println(dir);
  Serial.println(val);
  attachServos();


  if (dir == 'w') {
    servoStraight();
    int encDelta = int(val * ticksPermm);
    lSetpoint = lPosition - encDelta;
    rSetpoint = rPosition + encDelta;
    lPID.Setpoint(lSetpoint);
    rPID.Setpoint(rSetpoint);
    //    return true;
  }
  else if (dir == 'a' || dir == 'd') {
    servoTurn();
    int encDelta = int(val * turnConst);
    if (dir == 'a') {
      lSetpoint = lPosition + encDelta;
      rSetpoint = rPosition + encDelta;
      lPID.Setpoint(lSetpoint);
      rPID.Setpoint(rSetpoint);
    }
    else if (dir == 'd') {
      lSetpoint = lPosition - encDelta;
      rSetpoint = rPosition - encDelta;
      lPID.Setpoint(lSetpoint);
      rPID.Setpoint(rSetpoint);
    }
  }
}
void setGeometryConsts() {
  float wheelDiam = 70;
  float wheelCircum = wheelDiam * PI;
  float wheelbaseRadius = 135;
  int ticksPerRev = 3840;
  ticksPermm = ticksPerRev / wheelCircum;
  turnConst = ticksPermm * wheelbaseRadius;
}

void setup() {

  Serial.begin(9600);
  lSetpoint = lEnc.read();
  rSetpoint = rEnc.read();
  setGeometryConsts();

  attachServos();
  servoStraight();

  pinMode(M1DIR, OUTPUT);
  pinMode(M1PWM, OUTPUT);
  pinMode(M2DIR, OUTPUT);
  pinMode(M2PWM, OUTPUT);

  digitalWrite(nD2, HIGH);
  digitalWrite(M1DIR, LOW);
  analogWrite(M1PWM, 0);
  digitalWrite(M2DIR, LOW);
  analogWrite(M2PWM, 0);

  lPID.SetOutputLimits(-255, 255);
  rPID.SetOutputLimits(-255, 255);

  lPID.Start(lEnc.read(),  // input
             0,                      // current output
             0);                   // setpoint
  rPID.Start(rEnc.read(),  // input
             0,                      // current output
             0);                   // setpoint

  rPID.SetMode(1);
  lPID.SetMode(1);
  pinMode(nD2, OUTPUT);
  digitalWrite(nD2, HIGH);

}
void goToSleep() {
  //  digitalWrite(nD2, LOW);
  //  detachServos();
  //  Serial.println("going to sleep");
  //  lPID.SetMode(0);
  //  rPID.SetMode(0);
}

void wakeUp() {
  //  digitalWrite(nD2, HIGH);
  //  attachServos();
  //  Serial.println("waking up");
  //  lPID.SetMode(1);
  //  rPID.SetMode(1);
}


void loop() {

  const double lPosition = lEnc.read();
  const double rPosition = rEnc.read();

  recvWithEndMarker();
  if (newData) {
    Serial.println(dir);
    Serial.println(val);
    newData = false;
    //moveComplete = false;
    //wakeUp();
    setEncoderTargets(lPosition, rPosition);
  }
  const double lInput = lEnc.read();
  const double lOutput = lPID.Run(lInput);
  const double rInput = rEnc.read();
  const double rOutput = rPID.Run(rInput);
  
  if (true && millis() % 300 == 0) {
    Serial.print("L: ");
    Serial.print(lSetpoint);
    Serial.print(", ");
    Serial.print(lInput);
    Serial.print(", ");
    Serial.print(lOutput);
    Serial.print(", R: ");
    Serial.print(rSetpoint);
    Serial.print(", ");
    Serial.print(rInput);
    Serial.print(", ");
    Serial.println(rOutput);
  }

  if (rOutput < 0) {
    digitalWrite(M2DIR, LOW);
  }
  else {
    digitalWrite(M2DIR, HIGH);
  }
  if (lOutput < 0) {
    digitalWrite(M1DIR, LOW);
  }
  else {
    digitalWrite(M1DIR, HIGH);
  }
  analogWrite(M1PWM, abs(lOutput));
  analogWrite(M2PWM, abs(rOutput));
}
