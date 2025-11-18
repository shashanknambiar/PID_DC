#include <Arduino.h>
#include <util/atomic.h>
#include <AFMotor.h>
#include "QuadEncoder.h"
#include "VelocityControl.h"
#include "PositionControl.h"

// Encoder pins
#define ENCODER_A 21
#define ENCODER_B 20

// Predifination
void SetMotor(float u);
void CheckMode();
void PrintConstants(float Kp, float Ki, float Kd, float Tau, float K);

// Operation Mode
enum OperationMode
{
  SPIN,
  POS,
  TUNE
};
OperationMode opMode = POS;
OperationMode prevMode = opMode;

// Tuner
#define TUNE_BTN 51

// Logging
bool LogToCSVON = false;
bool SerialCurve = true;
void LogToCSV(unsigned long timeMs, float rpm, float setRpm, int pwmVal);
unsigned long lastLog = 0;
bool printHeaders = true;

// Encoder variables
long actual;
long posActual;
float targetRPM;
float targetPos;

//Motor pins
#define MOTOR_L 9 //Rotate the shaft left
#define MOTOR_R 8 //Rotate the shaft right

QuadEncoder qEncoder;
VelocityControl velControl;
PositionControl posControl(-150, 150);

void setup()
{
  Serial.begin(115200);
  qEncoder.Setup(ENCODER_A, ENCODER_B);
  velControl.Initialize();
  posControl.Initialize();
  pinMode(A15, INPUT);
  pinMode(TUNE_BTN, INPUT_PULLUP);
  //Motor pins
  pinMode(MOTOR_L, OUTPUT);
  pinMode(MOTOR_R, OUTPUT);
  sei();              // enable global interrupts
}

void loop()
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    actual = qEncoder.GetRPM();
    posActual = qEncoder.GetAngle();
  }

  switch (opMode)
  {
  case SPIN:
  {
    targetRPM = map(analogRead(A15), 0, 1023, -350, 350);
    float pwm = velControl.Calculate(targetRPM, actual);
    SetMotor(pwm);
    if (LogToCSVON)
      LogToCSV(millis(), actual, targetRPM, pwm);
    CheckMode();
    if (SerialCurve)
    {
      Serial.print("Actual:");
      Serial.print(actual);
      Serial.print("\tTarget:");
      Serial.println(targetRPM);
    }
    break;
  }
  case POS:
  {
    targetPos = 180;//map(analogRead(A15), 0, 1023, 0,  350);
    targetRPM = posControl.Calculate(targetPos, posActual);
    float pwm = velControl.Calculate(targetRPM, actual);
    SetMotor(pwm);
    if (LogToCSVON)
      LogToCSV(millis(), actual, targetRPM, pwm);
    CheckMode();
    if (SerialCurve)
    {
      Serial.print("Actual:");
      Serial.print(posActual);
      Serial.print("\tTarget:");
      Serial.println(targetPos);
    }
    break;
  }
  case TUNE:
  {
    float pwm = velControl.Tune(actual);
    SetMotor(pwm);
    if (velControl.GetTunerState() == VelocityControl::DONE)
    {

      float kp, ki, kd, tau, k;
      velControl.GetConstantsEEPROM(kp, ki, kd, tau, k);
      PrintConstants(kp, ki, kd, tau, k);
      Serial.println("Tuning position loop");
      posControl.Tune(tau, k);
      posControl.GetConstantsEEPROM(kp, ki, kd, tau, k);
      PrintConstants(kp, ki, kd, tau, k);
      Serial.print("Tuning completed. Switching to ");
      prevMode == SPIN ? Serial.println("SPIN") : Serial.println("POS"); 
      opMode = prevMode;
    }
    break;
  }
  default:
    opMode = SPIN;
    break;
  }

  // Serial.print("\tPWM");
  //  Serial.println(pwm);
}

void SetMotor(float u)
{
  int pwm = constrain(u, -255, 255);
  if (pwm < 0)
  {
   analogWrite(MOTOR_L, abs(pwm));
   analogWrite(MOTOR_R, 0);
  }
  else if (pwm > 0)
  {
   analogWrite(MOTOR_L, 0);
   analogWrite(MOTOR_R, abs(pwm));
  }
  else
  {
   analogWrite(MOTOR_L, 0);
   analogWrite(MOTOR_R, 0);
  }

}

void LogToCSV(unsigned long timeMs, float rpm, float setRpm, int pwmVal)
{
  if (printHeaders)
  {
    Serial.println("Time,Set RPM, Measured RPM, PWM");
    printHeaders = false;
  }

  if (timeMs - lastLog > 300) // Send logs every 60ms
  {
    Serial.print(timeMs);
    Serial.print(',');
    Serial.print(rpm);
    Serial.print(',');
    Serial.print(setRpm);
    Serial.print(',');
    Serial.println(pwmVal);
    lastLog = timeMs;
  }
}

void PrintConstants(float Kp, float Ki, float Kd, float Tau, float K)
{
  Serial.print("Kp: ");
  Serial.print(Kp);
  Serial.print(" Ki: ");
  Serial.print(Ki);
  Serial.print(" Kd: ");
  Serial.print(Kd);
  Serial.print(" Tau: ");
  Serial.print(Tau);
  Serial.print(" K: ");
  Serial.println(K);
}

void CheckMode()
{
  if (digitalRead(TUNE_BTN) == LOW)
  {
    prevMode = opMode;
    opMode = TUNE;
    Serial.println("Mode changed to TUNE");
  }
}
