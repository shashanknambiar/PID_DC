#ifndef VELOCITY_CONTROL_H
#define VELOCITY_CONTROL_H

#include "PID.h"
#include "PIDControlConfig.h"

class VelocityControl
{
    public:
        enum TuneMode { IDLE, BASELINE, STEP, DONE };
        VelocityControl(float Kp,float Ki,float Kd);
        VelocityControl();
        void Initialize();
        void GetConstants(float& Kp, float& Ki, float& Kd);
        void GetConstantsEEPROM(float& Kp, float& Ki, float& Kd, float& Tau, float& K);
        void SetConstants(float kP, float kD, float kI);
        float Calculate(int target, int actual);
        int Tune(int measuredRPM);
        TuneMode GetTunerState();
        ~ VelocityControl();
    private:
        bool SaveConstants();
        PID _PID;
        PIDControlConfig _PIDControlConfig;
        const PIDLoopConfig PIDVelocityDefaults = {0.86, 2.17, 0.03};
        //Step response auto tuner
        TuneMode TunerState;
        unsigned long baselineStart = 0, tuneStartTime = 0, lastSampledAt = 0;
        unsigned long baselineDuration = 1500; //1.5 seconds at Low
        unsigned int index = 0;
        const int pwmLow = 70, pwmHigh = 120;
        float* samples = nullptr; 
        const unsigned int MAX_SAMPLES = 600;
        void ResetSamples();
        void ComputeAndSet();
};      

#endif
