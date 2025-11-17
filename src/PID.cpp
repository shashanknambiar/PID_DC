#include "PID.h"
#include <Arduino.h>

PID::PID(float Kp, float Ki, float Kd, float max, float min, float sampleTimeMs)
            : Kp(Kp), Ki(Ki), Kd(Kd), Max(max), Min(min), sampleTimeMs(sampleTimeMs)
{

}
void PID::GetConstants(float& kp,float& ki, float& kd)
{
    kp = Kp; ki = Ki; kd = Kd;
}

void PID::SetConstants(float kp, float ki, float kd)
{
    Kp = kp; Ki = ki; Kd = kd;
}

void PID::SetOutputLimits(float _min, float _max){
    Min = _min; Max = _max;
}

float PID::Calculate(float setPoint, float measurement)
{
    unsigned long now = millis();
    unsigned long dtMs = now - lastTime;
    if (lastTime == 0) { // first call
      lastTime = now;
      lastMeasurement = measurement;
      return 0.0f;
    }
    if (dtMs < sampleTimeMs) return lastOutput;

    float dt = dtMs / 1000.0f;
    float error = setPoint - measurement;

    //Proportional 
    float P = Kp * error;

    //Integral with anti windup
    // --- Integral with anti-windup ---
    if (Ki != 0.0f) {
        Integral += error * dt;

        float maxInt = Max / Ki;
        float minInt = Min / Ki;

        if (Integral > maxInt) Integral = maxInt;
        if (Integral < minInt) Integral = minInt;
    }
     float I = Ki * Integral;
    //Derivative
    float derivative = (measurement - lastMeasurement) / dt;
    float D = -Kd * derivative;
    
    float output = P + I + D;

    // clamp final output
    if (output > Max) output = Max;
    if (output < Min) output = Min;

    // save state
    lastOutput = output;
    lastError = error;
    lastMeasurement = measurement;
    lastTime = now;

    return output;
}
