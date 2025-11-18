#include "PositionControl.h"
#include "PID.h"
#include "PIDControlConfig.h"
#include <Arduino.h>

PositionControl::PositionControl(int MinOut, int MaxOut): _PID(1, 0 , 0, MaxOut, MinOut, 60)
{

}

void PositionControl::Initialize()
{
    PIDLoopConfig posConfig;
    _PIDControlConfig.loadPosition(posConfig, PIDPositionDefaults);
    Serial.println("++++++++Position control initializing with following parameters++++++++");
    Serial.print("Kp: ");
    Serial.print(posConfig.kp);
    Serial.print(" Ki: ");
    Serial.print(posConfig.ki);
    Serial.print(" Kd: ");
    Serial.print(posConfig.kd);
    Serial.print(" Tau: ");
    Serial.print(posConfig.timeConstant);
    Serial.print(" K: ");
    Serial.println(posConfig.steadyStateGain);
}

void PositionControl::SetConstants(float kP, float kI, float kD)
{
    _PID.SetConstants(kP, kI, kD);
}

float PositionControl::Calculate(int setpoint, int measured)
{
    return _PID.Calculate(setpoint, measured);
}

void PositionControl::GetConstants(float &kp, float &ki, float &kd)
{
    _PID.GetConstants(kp, ki, kd);
}

void PositionControl::GetConstantsEEPROM(float &kp, float &ki, float &kd, float& tau, float& k)
{
    PIDLoopConfig velocityConfig;
    _PIDControlConfig.loadPosition(velocityConfig, PIDPositionDefaults);
    kp = velocityConfig.kp;
    ki = velocityConfig.ki;
    kd = velocityConfig.kp;
    tau = velocityConfig.timeConstant;
    k = velocityConfig.steadyStateGain;
}

int PositionControl::Tune(float Tau_v, float K_v)
{
    // Kp = alpha^2 / (tau_v * Kv)
    float Kp = (_Alpha * _Alpha) / (Tau_v * K_v);
    // choose default Ti = 5*tau_v -> Ki = Kp / Ti
    float Ti = 5.0f * Tau_v;
    float Ki = (Ti > 0.0f) ? (Kp / Ti) : 0.0f;
    // store current (applied) gains (you may scale these down initially)
    _PID.SetConstants(Kp, Ki, 0);
    PIDLoopConfig newConfig = {Kp, Ki, 0, 0, 0};
    _PIDControlConfig.savePosition(newConfig);
    return 0;
}
