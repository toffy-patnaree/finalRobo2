
/*
 * Set Board: Tools --> Board --> Arduino Mega2560
 * Set Processor: Tools --> Processor --> ATmega2560
 */

#include <Servo.h>
#include <Ewma.h>
#include "configuration.h"
#include <ArduinoJson.h>

StaticJsonDocument<200> doc;

//=============== Function Name Declaration ===============
double Deadband(double value, double limit);
int OutputToMotor1(int value);
int OutputToMotor2(int value);
void AdjustAngleBase(Servo myServoL, Servo myServoR, int value);
void AdjustAngleRotating(Servo myServo, int value);
Ewma adcFilter3(0.50); // Base
Ewma adcFilter4(0.95); // Rotating

//=============== Parameters Declaration ==================

// Type Servo
Servo myServo1;
Servo myServo2;
Servo myServo3;
Servo myServo4;
// Time
unsigned long previousLoopTime = 0;
unsigned long loopTime = 0;
// Input Signal from Receiver
int input1 = 0;
int input2 = 0;
int input3 = 0;
int input4 = 0;
int input5 = 0;
int input6 = 0;

// filter
float filterInput5 = 0;

// Output Signal to Motor Driver
int out1 = 0;
int out2 = 0;

// Motor's Current
float currentValue1 = 0.0;
float currentValue2 = 0.0;
int currentLimit = 5;

// degree
int degreeBaseL = 90;
int degreeBaseR = 90;
int degreeRotating = 90;

//===================== setup() ========================

void setup()
{
  //===== Set Digital Pin Mode =====
  // Set pinmode to read command signal from Receiver.
  pinMode(CH1, INPUT); // channel 1
  pinMode(CH2, INPUT); // channel 2
  pinMode(CH3, INPUT); // channel 3
  pinMode(CH4, INPUT); // channel 4
  pinMode(CH5, INPUT); // channel 5
  pinMode(CH6, INPUT); // channel 6
  // Set pinmode to read command signal from Test Switch.
  pinMode(buttonPin, INPUT);
  // Set pinmode to write command signal to Motor Driver.
  pinMode(INA1, OUTPUT);
  pinMode(INB1, OUTPUT);
  pinMode(PWM1, OUTPUT);

  pinMode(INA2, OUTPUT);
  pinMode(INB2, OUTPUT);
  pinMode(PWM2, OUTPUT);

  // Assign Servo variable to a servo pin
  myServo1.attach(servo1);
  myServo2.attach(servo2);
  myServo3.attach(servo3);
  myServo4.attach(servo4);

  //===== Initialize Command =====
  // Initialize Motor Driver.
  digitalWrite(INA1, LOW);
  digitalWrite(INB1, LOW);
  analogWrite(PWM1, 0);

  digitalWrite(INA2, LOW);
  digitalWrite(INB2, LOW);
  analogWrite(PWM2, 0);

  // Initialize Servo Motor, Set servo to Mid-point.
  myServo1.write(90); // change to 90 mai?
  myServo2.write(90);
  myServo3.write(90);
  myServo4.write(90); // change to 90 mai?

  // Open Serial port, Set baud rate for serial data transmission.
  Serial.begin(115200); // USB:Rx0,Tx0
  while (!Serial)
    continue;

  // Returns time(us)
  previousLoopTime = micros();

} // End SetUp

//======================= loop() ==========================

void loop()
{

  loopTime = micros() - previousLoopTime;

  // if loop time is more than 10000 microseconds, do the next loop.
  // Limit Maximum feedback loop at 100Hz.
  if (loopTime >= 10000)
  {
    // Set new loop time
    previousLoopTime = micros();

    // Read input signal from receiver. PulseIn signal from Receiver vary between 1000 - 2000.
    // Substract 1500 as the offset to set new signal range from (1000, 2000) to (-500, 500)
    // Also set Deadband limit to the input Signal

    input1 = pulseIn(CH1, HIGH) - 1500; // Channel 1
    input2 = pulseIn(CH2, HIGH) - 1500; // Channel 2
    input3 = pulseIn(CH3, HIGH) - 1500; // Channel 3
    input4 = pulseIn(CH4, HIGH) - 1500; // Channel 4
    input5 = pulseIn(CH5, HIGH) - 1500; // Channel 5
    input6 = pulseIn(CH6, HIGH) - 1500; // Channel 6

    input1 = Deadband(input1, 30); // Channel 1
    input2 = Deadband(input2, 30); // Channel 2
    input3 = Deadband(input3, 30); // Channel 3
    input4 = Deadband(input4, 30); // Channel 4
    input5 = Deadband(input5, 30); // Channel 5
    input6 = Deadband(input6, 30); // Channel 6

    // Filter Signal
    float filterInput3 = adcFilter3.filter(input3);
    float filterInput4 = adcFilter4.filter(input4);

    // Read Motor's Current From Motor Driver
    // The resolution of Arduino analogRead is 5/1024 Volts/Unit. (10-Bit, Signal vary from 0 to 1023 units)
    // The resolution of Current Sensor from POLOLU VNH5019 is 0.14 Volt/Amp.
    // Convert analogRead signal(Volt) to Current(Amp) by multiply (5/1024)/0.14 = 0.035 Amp/Unit.
    currentValue1 = analogRead(CS1) * 0.035; // Motor Driver 1
    currentValue2 = analogRead(CS2) * 0.035; // Motor Driver 2

    // Check Motor Current and Assign Motor Direction
    // Motor can be operated, if motor's current is lower than 5 amp.
    if (currentValue1 < currentLimit)
      out1 = OutputToMotor1(input1);
    else
      out1 = OutputToMotor1(0);
    if (currentValue2 < currentLimit)
      out2 = OutputToMotor2(input2);
    else
      out2 = OutputToMotor2(0);

    // Drive Motor
    // Assign motor's direction by function OutputToMotor.
    // Command PWM Duty Cycle to drive motor.
    analogWrite(PWM1, out1);
    analogWrite(PWM2, out2);

    // Moving Servo

    AdjustAngleRotating(myServo2, filterInput4); // rotating

    if (input5 > 450)
    {
      filterInput5 = 100;
    }
    else if (input5 < -450)
    {
      filterInput5 = 0;
    }
    //myServo3.write(filterInput5); // gripping

    AdjustAngleBase(myServo1, myServo4, filterInput3); // base

    // Print
//    Serial.print("M 1 = ");
//    Serial.print(input1);
//    Serial.print("\t M 2 = ");
//    Serial.print(input2);
//    Serial.print("\t M 3 = ");
//    Serial.print(filterInput3);
//    Serial.print("\t M 4 = ");
//    Serial.print(filterInput4);
//    Serial.print("\t M 5 = ");
//    Serial.print(filterInput5);
//    Serial.print("\t M 6= ");
//    Serial.print(input6);
//    Serial.print("\t LoopTime = ");
//    Serial.println(loopTime);

    // Jetson Nano
    // Check if there is data in serial buffer
    if (Serial.available())
    {
      Serial.println("Serial Check");

      // Deserialize the JSON document
      DeserializationError error = deserializeJson(doc, Serial);
      // Action if parsing succeeds.
      if (!error)
      Serial.println("non Error check");
      {
        int cmd = Serial.read();

        // bottle center size range ok
        if (cmd == 0)
        {
          myServo3.write(100);
          AdjustAngleBase(myServo1, myServo4, 300);
          delay(2000);
          Serial.println("cmd=0 done");
        }

        // bottle center size range >
        else if (cmd == 1)
        {
          // Move backward
          // set PWM for the speed of rotation of motor
          analogWrite(PWM1, 60);
          analogWrite(PWM2, 60);
          // Motor 1 Counterclockwise direction
          digitalWrite(INA1, LOW);
          digitalWrite(INB1, HIGH);
          // Motor 2 clockwise direction
          digitalWrite(INA2, HIGH);
          digitalWrite(INB2, LOW);
        }

        // bottle center size range <
        else if (cmd == 2)
        {
          // Move forward
          // set PWM for the speed of rotation of motor
          analogWrite(PWM1, 60);
          analogWrite(PWM2, 60);
          // Motor 1 Counterclockwise direction
          digitalWrite(INA1, HIGH);
          digitalWrite(INB1, LOW);
          // Motor 2 clockwise direction
          digitalWrite(INA2, LOW);
          digitalWrite(INB2, HIGH);
        }

        // bottle too left
        else if (cmd == 3)
        {
          // set PWM for the speed of rotation of motor
          analogWrite(PWM1, 60);
          analogWrite(PWM2, 60);
          // Motor 3&4 Counterclockwise direction, so it'll move to the left
          digitalWrite(INA2, LOW);
          digitalWrite(INB2, HIGH);
          digitalWrite(INA1, LOW);
          digitalWrite(INB1, HIGH);
          delay(1000);
        }

        // bottle too right
        else if (cmd == 4)
        {
          // set PWM for the speed of rotation of motor
          analogWrite(PWM1, 60);
          analogWrite(PWM2, 60);
          // Motor 3&4 clockwise direction, so it'll move to the right
          digitalWrite(INA1, HIGH);
          digitalWrite(INB1, LOW);
          digitalWrite(INB2, LOW);
          digitalWrite(INA2, HIGH);
          delay(1000);
        }

        // bottle too high
        else if (cmd == 5)
        {
          AdjustAngleBase(myServo1, myServo4, -300);
          delay(1000);
        }

        // bottle too low
        else
        {
          AdjustAngleBase(myServo1, myServo4, 300);
          delay(1000);
        }
      }
    }
    // If there is no serial sending to arduino
    else
    {

      digitalWrite(INA3, HIGH);
      digitalWrite(INB3, HIGH);
      digitalWrite(INA4, HIGH);
      digitalWrite(INB4, HIGH);
    }
    delay(100);

  } // End if
} // End loop

//=============== Function Declaration ===============

//===== double Deadband(double value,double limit) =====
//===== Set Dead Band =====
// If the input signal from receiver is in the band limit, set input signal to 0.0.
double Deadband(double value, double limit)
{
  double temp = 0.0;
  if (value >= limit)
    temp = value - limit;
  else if (value <= -limit)
    temp = value + limit;
  else
    temp = 0.0;
  return temp;
}

//===== int OutputToMotor(int value) ======
//===== Assign Motor's Direction and Scale Down Input Signal =====
// value must be positive and scaled down to fit 8-Bit PWM Range.

// Motor 1
int OutputToMotor1(int value)
{
  int temp = 0;
  if (value >= 0)
  {
    // CW
    digitalWrite(INA1, LOW);
    digitalWrite(INB1, HIGH);
    temp = map(value, 0, 500, 0, 255);
  }
  else
  {
    // CCW
    digitalWrite(INA1, HIGH);
    digitalWrite(INB1, LOW);
    temp = map(-value, 0, 500, 0, 255);
  }
  return temp;
}

// Motor 2
int OutputToMotor2(int value)
{
  int temp = 0;
  if (value >= 0)
  {
    digitalWrite(INA2, LOW);
    digitalWrite(INB2, HIGH);
    temp = map(value, 0, 500, 0, 255);
  }
  else
  {
    digitalWrite(INA2, HIGH);
    digitalWrite(INB2, LOW);
    temp = map(-value, 0, 500, 0, 255);
  }
  return temp;
}

// AdjustAngleServo
void AdjustAngleBase(Servo myServoL, Servo myServoR, int value)
{
  //    degreeBase = map(value,-350,350,0,180);
  //    myServo.write(degreeBase);
  //

  if (value > 200)
  {
    degreeBaseL++;
    degreeBaseR--;
  }
  else if (value < -200)
  {
    degreeBaseL--;
    degreeBaseR++;
  }

  myServoL.write(degreeBaseL);
  myServoR.write(degreeBaseR);
}

void AdjustAngleRotating(Servo myServo, int value)
{
  //    degree = map(value,-350,350,0,180);
  //    myServo.write(degree);

  if (value > 300)
  {
    degreeRotating++;
  }
  else if (value < -300)
  {
    degreeRotating--;
  }
  myServo.write(degreeRotating);
}
