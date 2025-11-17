#ifndef PID_H
#define PID_H

class PIDParameters
{
    public:
    PIDParameters();
    PIDParameters(float Kp, float Ki, float Kd);
    float Kp;
    float Ki;
    float Kd;
};

class PID
{
    public:
    PID(float Kp, float Ki, float Kd, float max, float min, float dt);
    void SetOutputLimits(float min, float max);
    void SetConstants(float Kp, float Ki, float Kd);
    bool Tune(float& Kp,float& Ki, float& Kd);
    void GetConstants(float& Kp,float& Ki, float& Kd);
    float Calculate(float setPoint, float measurement);
    private:    
    float Kp, Ki, Kd, Max, Min, sampleTimeMs, lastMeasurement, lastTime, lastOutput, lastError;
    float Integral;
};
#endif