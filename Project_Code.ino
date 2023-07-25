/// Final Code for all 3 modes: Manuel control, Line follower and obstacle Avoider
#include <Servo.h>
char TxData;
int trig=7;                   // Trig pin of UltraSonic Sensor is connected to pin 7 of arduino.
int echo=3;                   // Echo pin of UltraSonuc Sensor.
int M1A=9;                    // intialize motor pins
int M1B=10;
int M2A=11;
int M2B=12;
int LSP=13;                   // intialize IRsensor Pins
int RSP=4;
int en1=5;                    // intialize PWM pins
int en2=6;
int LSD;
int RSD;
double distance;             // declare distance variables for ultrasonic sensor
double Left_Distance;
double Right_Distance;
double L = 0;
double R = 0;
Servo s;                     // servo object
bool LineFollowingMode;
bool AvoiderRobotMode;

void setup() {
   Serial.begin(9600);       // set the baud rate for data transmission.

   // Motor pins are output
   pinMode(M1A,OUTPUT);
   pinMode(M1B,OUTPUT);
   pinMode(M2A,OUTPUT);
   pinMode(M2B,OUTPUT);

   // Ultra sonic sensors
   pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  // IR sensor pins are input
  pinMode(LSP,INPUT);
   pinMode(RSP,INPUT);

   // PWM output
   pinMode(en1,OUTPUT);
   pinMode(en2,OUTPUT);

   // servo motor intializiation
   s.attach(2);
  s.write(90);

  Stop();     // intialize motor operation
}

void loop() {
  
  // line follower mode
  if (LineFollowingMode)
  {
   LSD=digitalRead(LSP);             // Read signals from IR sensors
   RSD=digitalRead(RSP); 
   if (LSD == HIGH && RSD == LOW)     // check IR signal values if Left detects black line turn Left.
   {
     Left();
     delay(100);
   }
   else if (LSD == LOW && RSD == HIGH)  // check IR signal values if Right detects black line turn right.
   {
     Right();
     delay(100);
   }
   else if (LSD == LOW && RSD == LOW)  // otherwise move forward.
   {
     Forward();
   }
  }

// object avoider mode.
  if(AvoiderRobotMode)
  {
   distance = getDistanceinCm();        // calculate the distance
  if (distance <= 20) {                  // if robot approaches an object check distance on either side and choose the path with farther distance.
    Stop();
    delay(200);
    Backward();
    delay(300);
    Stop();
    L = leftsee();
    s.write(90);
    delay(800);
    R = rightsee();
    s.write(90);
    if (L < R) {
      Right();
      delay(800);
      Stop();
      delay(200);
    } else if (L > R) {
      Left();
      delay(800);
      Stop();
      delay(200);
    }
  } else {
    Forward();
  } 
  }

  // Check transmitted data over bluetooth.
  if(Serial.available()>0){
    TxData = Serial.read();
    
    if (TxData=='s'){
      Stop();
      LineFollowingMode=false;
      AvoiderRobotMode=false;
      s.write(90);
    }
    else if (TxData=='2'){
     LineFollowingMode=true;
     } 
     else if (TxData=='f'){
     Forward();
     }
    else if (TxData=='b'){
     Backward();
     }
     else if (TxData=='r'){
     Right();
     }
     else if (TxData=='l'){
     Left();
     }
     else if (TxData=='3'){
     AvoiderRobotMode=true;
     }
  }

delay (100);
}

//Motor direction speed and direction control.
void Forward()
{
  
digitalWrite(M1A, LOW); 
digitalWrite(M1B, HIGH);
digitalWrite(M2A, HIGH);
digitalWrite(M2B, LOW);  
analogWrite(en1,80);
analogWrite(en2,80);
}
void Backward() 
{
  
digitalWrite(M1A, HIGH); //back
digitalWrite(M1B, LOW);
digitalWrite(M2A, LOW);
digitalWrite(M2B, HIGH); 
analogWrite(en1,80);
analogWrite(en2,80);
}
void Left() 
{
digitalWrite(M1A, LOW);
digitalWrite(M1B, HIGH);
digitalWrite(M2A, LOW);
digitalWrite(M2B, LOW);
analogWrite(en1,100);
analogWrite(en2,100);
}
void Right()
{
 digitalWrite(M1A, LOW);
digitalWrite(M1B, LOW);
digitalWrite(M2A, HIGH);
digitalWrite(M2B, LOW);
 analogWrite(en1,100);
analogWrite(en2,100);
}
void Stop()
{
digitalWrite(M1A, LOW);
digitalWrite(M1B, LOW);
digitalWrite(M2A, LOW);
digitalWrite(M2B, LOW); 
}

// Ultrasonic sensor distance reading function
double getDistanceinCm() {
  digitalWrite(trig, LOW);
  delayMicroseconds(4);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long TimeMS = pulseIn(echo, HIGH);
  long cm = TimeMS / 29 / 2; //time convert distance
  return cm;
}

// Servo motor control 

double leftsee() {
  s.write(160);
  delay(800);
  Left_Distance = getDistanceinCm();
  return Left_Distance;
}
double rightsee() {
  s.write(20);
  delay(800);
  Right_Distance = getDistanceinCm();
  return Right_Distance;
}

