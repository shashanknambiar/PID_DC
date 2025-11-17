#include <Arduino.h>
#include <util/atomic.h>
#include <AFMotor.h>
#include "QuadEncoder.h"
#include "VelocityControl.h"

// Encoder pins
#define ENCODER_A 21
#define ENCODER_B 20

// Predifination
void SetMotor(float u);
void CheckMode();
void  PrintConstants(float Kp, float Ki, float Kd, float Tau, float K);

// Operation Mode
enum OperationMode
{
  SPIN,
  TUNE
};
OperationMode opMode = SPIN;

// Tuner
#define TUNE_BTN 51

// Logging
bool LogToCSVON = false;
void LogToCSV(unsigned long timeMs, float rpm, float setRpm, int pwmVal);
unsigned long lastLog = 0;
bool printHeaders = true;

// Encoder variables
long actual;
float targetRPM;

// Initliaze motor 3 on adafruit shield
AF_DCMotor motor(3);

QuadEncoder qEncoder;
VelocityControl velControl;

void setup()
{
  Serial.begin(115200);
  qEncoder.Setup(ENCODER_A, ENCODER_B);
  velControl.Initialize();
  pinMode(A15, INPUT);
  pinMode(TUNE_BTN, INPUT_PULLUP);  
  motor.run(FORWARD); // Motor off initially
  sei();              // enable global interrupts
}

void loop()
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    actual = qEncoder.GetRPM();
  }

  switch (opMode)
  {
  case SPIN:
  {
    targetRPM = map(analogRead(A15), 0, 1023, -150, 150);
    float pwm = velControl.Calculate(targetRPM, actual);
    SetMotor(pwm);
    if(LogToCSVON)
      LogToCSV(millis(), actual, targetRPM, pwm);
    CheckMode();
    break;
  }
  case TUNE:
  {
    float pwm = velControl.Tune(actual);
    SetMotor(pwm);
    if(velControl.GetTunerState() == VelocityControl::DONE)
    {
      float kp, ki, kd, tau, k;
      velControl.GetConstantsEEPROM(kp, ki, kd, tau, k);
      PrintConstants(kp, ki, kd, tau, k);
      Serial.println("Tuning completed. Switching to SPIN");
      opMode = SPIN;
    }
    break;
  }
  default:
    opMode = SPIN;
    break;
  }

  Serial.print("Actual:");
  Serial.print(actual);
  Serial.print("\tTarget:");
  Serial.println(targetRPM);
  //Serial.print("\tPWM");
  // Serial.println(pwm);
}

void SetMotor(float u)
{
  int pwm = constrain(u, -255, 255);
  if (pwm < 0)
    motor.run(BACKWARD);
  else if (pwm > 0)
    motor.run(FORWARD);
  else
    motor.run(RELEASE);
  motor.setSpeed(abs(pwm));
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
  if(digitalRead(TUNE_BTN) == LOW)
  {
    opMode = TUNE;
    Serial.println("Mode changed to TUNE");
  }
  else 
  {
    opMode = SPIN;
  }
}
