/*
The MIT License (MIT)

Copyright (c) 2016 Chiang Mai Maker Club

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "CurieImu.h"

//#define ACCELEROMETER_SENSITIVITY 8192.0
#define GYROSCOPE_SENSITIVITY 16.348

#define M_PI 3.14159265359
#define dt (10.0/1000.0)             // 100hz = 10ms

#define RMotor_offset 60 // The offset of the Motor
#define LMotor_offset 50 // The offset of the Motor

// walkin
static float kp = 70.00f;
static float ki = 2.500f;
static float kd = 6000.00f;

uint32_t _prev = millis();

static uint32_t lastTime = millis();

int TN1 = 4;
int TN2 = 3;
int ENA = 5;
int TN3 = 8;
int TN4 = 7;
int ENB = 6;

static float error = 0;  // Proportion
static float errSum = 0;
static float dErr = 0;

struct Motor {
  float LOutput;
  float ROutput;
} motor_t;


//http://www.pieter-jan.com
static void ComplementaryFilter(short accData[3], short gyrData[3], float *pitch, float *roll)
{
  float pitchAcc, rollAcc;

  // Integrate the gyroscope data -> int(angularSpeed) = angle
  *pitch += ((float)gyrData[0] / GYROSCOPE_SENSITIVITY) * dt; // Angle around the X-axis
  *roll -= ((float)gyrData[1] / GYROSCOPE_SENSITIVITY) * dt;    // Angle around the Y-axis

  // Compensate for drift with accelerometer data if !bullshit
  // Sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
  int forceMagnitudeApprox = abs(accData[0]) + abs(accData[1]) + abs(accData[2]);
  if (forceMagnitudeApprox > 8192 && forceMagnitudeApprox < 32768)
  {
    // Turning around the X axis results in a vector on the Y-axis
    pitchAcc = atan2f((float)accData[1], (float)accData[2]) * 180 / M_PI;
    *pitch = *pitch * 0.98 + pitchAcc * 0.02;

    // Turning around the Y axis results in a vector on the X-axis
    rollAcc = atan2f((float)accData[0], (float)accData[2]) * 180 / M_PI;
    *roll = *roll * 0.98 + rollAcc * 0.02;
  }
}

void setup() {
  Serial.begin(115200); // initialize Serial communication
  //  while (!Serial);    // wait for the serial port to open

  // initialize device
  Serial.println("Initializing IMU device...");

  CurieImu.initialize();
  CurieImu.setFullScaleAccelRange(BMI160_ACCEL_RANGE_2G);
  CurieImu.setFullScaleGyroRange(BMI160_GYRO_RANGE_2000);
  //  CurieImu.setGyroRate(BMI160_GYRO_RATE_100HZ);
  //  CurieImu.setFullScaleGyroRange(BMI160_GYRO_RANGE_2000);
  CurieImu.setGyroRate(BMI160_GYRO_RATE_3200HZ);
  //  CurieImu.setGyroDLPFMode(BMI160_DLPF_MODE_NORM);

  Serial.print("full scale accel \t");
  Serial.print(CurieImu.getFullScaleAccelRange());
  Serial.print("full scale gyro \t");
  Serial.println(CurieImu.getFullScaleGyroRange());

  // verify connection
  Serial.println("Testing device connections...");
  if (CurieImu.testConnection()) {
    Serial.println("CurieImu connection successful");
  } else {
    Serial.println("CurieImu connection failed");
  }

  // use the code below to calibrate accel/gyro offset values
  Serial.println("Internal sensor offsets BEFORE calibration...");
  Serial.print(CurieImu.getXAccelOffset());
  Serial.print("\t"); // -76
  Serial.print(CurieImu.getYAccelOffset());
  Serial.print("\t"); // -235
  Serial.print(CurieImu.getZAccelOffset());
  Serial.print("\t"); // 168
  Serial.print(CurieImu.getXGyroOffset());
  Serial.print("\t"); // 0
  Serial.print(CurieImu.getYGyroOffset());
  Serial.print("\t"); // 0
  Serial.println(CurieImu.getZGyroOffset());

  pinMode(TN1, OUTPUT);
  pinMode(TN2, OUTPUT);
  pinMode(TN3, OUTPUT);
  pinMode(TN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  Serial.println("About to calibrate. Make sure your board is stable and upright");
  delay(10);

  // The board must be resting in a horizontal position for
  // the following calibration procedure to work correctly!
  Serial.print("Starting Gyroscope calibration...");
  CurieImu.autoCalibrateGyroOffset();
  Serial.println(" Done");
  Serial.print("Starting Acceleration calibration...");
  CurieImu.autoCalibrateXAccelOffset(0);
  CurieImu.autoCalibrateYAccelOffset(0);
  CurieImu.autoCalibrateZAccelOffset(1);
  Serial.println(" Done");

  Serial.println("Internal sensor offsets AFTER calibration...");
  Serial.print(CurieImu.getXAccelOffset());
  Serial.print("\t"); // -76
  Serial.print(CurieImu.getYAccelOffset());
  Serial.print("\t"); // -2359
  Serial.print(CurieImu.getZAccelOffset());
  Serial.print("\t"); // 1688
  Serial.print(CurieImu.getXGyroOffset());
  Serial.print("\t"); // 0
  Serial.print(CurieImu.getYGyroOffset());
  Serial.print("\t"); // 0
  Serial.println(CurieImu.getZGyroOffset());

  Serial.println("Enabling Gyroscope/Acceleration offset compensation");
  CurieImu.setGyroOffsetEnabled(true);
  CurieImu.setAccelOffsetEnabled(true);
  _prev = millis();
}


//cmmakerclub.com
void myPID(float filtered_angle, Motor *motor) {
  static float lastErr = 0;
  uint32_t timeChange = (millis() - lastTime);
  static float Output;
  lastTime = millis();

  error = filtered_angle;  // Proportion
  errSum += error * timeChange;  // Integration
  dErr = (error - lastErr) / timeChange;  // Differentiation
  Output = kp * error + ki * errSum + kd * dErr;
  lastErr = error;

  motor->LOutput = Output;
  motor->ROutput = Output;

}

static void PWMControl(float LOutput, float ROutput)
{
  if (LOutput > 0)
  {
    digitalWrite(TN1, 0);
    digitalWrite(TN2, 1);
  }
  else if (LOutput < 0)
  {
    digitalWrite(TN1, 1);
    digitalWrite(TN2, 0);
  }
  else
  {
    digitalWrite(ENA, 0);
  }

  if (ROutput > 0)
  {
    digitalWrite(TN3, 1);
    digitalWrite(TN4, 0);
  }
  else if (ROutput < 0)
  {
    digitalWrite(TN3, 0);
    digitalWrite(TN4, 1);
  }
  else
  {
    digitalWrite(ENB, 0);
  }

  analogWrite(ENA, min(255, (abs(LOutput) + LMotor_offset)));
  analogWrite(ENB, min(255, (abs(ROutput) + RMotor_offset)));
}

void readIMUSensor(float *Angle_Filtered) {
  static float roll;
  static float pitch;
  // put your main code here, to run repeatedly:
  short accData[3];
  short gyrData[3];

  CurieImu.getMotion6(&accData[0], &accData[1], &accData[2], &gyrData[0], &gyrData[1], &gyrData[2]);
  ComplementaryFilter(accData, gyrData, &pitch, &roll);

  *Angle_Filtered = pitch;
  Serial.println(String("Angle Filter = ") + String(pitch));
}


void loop() {
  static Motor motor;
  static float Angle_Filtered;
  if ( millis() - _prev > dt * 1000 ) {
    _prev = millis();
    readIMUSensor(&Angle_Filtered);
    //   If angle > 45 or < -45 then stop the robot
    if (abs(Angle_Filtered) < 45)
    {
      myPID(Angle_Filtered, &motor);
      PWMControl(motor.LOutput, motor.ROutput);
    }
    else
    {
      //TODO:// set zero when robot down.
      //      Output = error = errSum = dErr = 0;
      Serial.println("STOP");
      analogWrite(ENA, 0);  // Stop the wheels
      analogWrite(ENB, 0);  // Stop the wheels
    }
  }
}
