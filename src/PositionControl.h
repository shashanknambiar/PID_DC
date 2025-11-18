#ifndef POSITION_CONTROL_H
#define POSITION_CONTROL_H

#include "PID.h"
#include "PIDControlConfig.h"

class PositionControl
{
public:
    PositionControl(int MinOut, int MaxOut);
    void Initialize();
    void GetConstants(float &Kp, float &Ki, float &Kd);
    void GetConstantsEEPROM(float &Kp, float &Ki, float &Kd, float &Tau, float &K);
    void SetConstants(float kP, float kD, float kI);
    float Calculate(int target, int actual);
    int Tune(float Tau, float K); //Modal based tuning
    float _Alpha = 0.15;

private:
    PID _PID;
    PIDControlConfig _PIDControlConfig;
    const PIDLoopConfig PIDPositionDefaults = {1, 0, 0};
};

#endif