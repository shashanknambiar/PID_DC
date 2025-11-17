
#include "VelocityControl.h"
#include "PIDControlConfig.h"
#include <Arduino.h>

VelocityControl::VelocityControl(float Kp, float Ki, float Kd) : _PID(Kp, Ki, Kd, 255, -255, 30)
{
    TunerState = IDLE;
    samples = new float[MAX_SAMPLES];
}

VelocityControl::VelocityControl(): _PID(1, 0, 0, 255, -255, 30)
{
    TunerState = IDLE;
    samples = new float[MAX_SAMPLES];
}

void VelocityControl::Initialize()
{
    PIDLoopConfig velocityConfig;
    _PIDControlConfig.loadVelocity(velocityConfig, PIDVelocityDefaults);
    _PID.SetConstants(velocityConfig.kp, velocityConfig.ki, velocityConfig.kd);
    Serial.println("++++++++Velocity control initializing with following parameters++++++++");
    Serial.print("Kp: ");
    Serial.print(velocityConfig.kp);
    Serial.print(" Ki: ");
    Serial.print(velocityConfig.ki);
    Serial.print(" Kd: ");
    Serial.print(velocityConfig.kd);
    Serial.print(" Tau: ");
    Serial.print(velocityConfig.timeConstant);
    Serial.print(" K: ");
    Serial.println(velocityConfig.steadyStateGain);
}

void VelocityControl::SetConstants(float kP, float kI, float kD)
{
    _PID.SetConstants(kP, kI, kD);
}

float VelocityControl::Calculate(int setpoint, int measured)
{
    return _PID.Calculate(setpoint, measured);
}

void VelocityControl::GetConstants(float &kp, float &ki, float &kd)
{
    _PID.GetConstants(kp, ki, kd);
}

void VelocityControl::GetConstantsEEPROM(float &kp, float &ki, float &kd, float& tau, float& k)
{
    PIDLoopConfig velocityConfig;
    _PIDControlConfig.loadVelocity(velocityConfig, PIDVelocityDefaults);
    kp = velocityConfig.kp;
    ki = velocityConfig.ki;
    kd = velocityConfig.kp;
    tau = velocityConfig.timeConstant;
    k = velocityConfig.steadyStateGain;
}

int VelocityControl::Tune(int measuredRPM)
{
    switch (TunerState)
    {
    case IDLE:
    case DONE:
        TunerState = BASELINE;
        baselineStart = millis();
        tuneStartTime = 0;
        lastSampledAt = 0;
        index = 0;
        ResetSamples();
        return pwmLow;
    case BASELINE:
    {
        if (millis() - baselineStart >= baselineDuration)
        {
            TunerState = STEP;
            tuneStartTime = millis();
            return pwmHigh;
        }
        return pwmLow;
    }
    case STEP:
    {
        if (index < MAX_SAMPLES)
        {
            if (millis() - lastSampledAt >= 10) // sample every 10ms
            {
                
                samples[index++] = measuredRPM;
                lastSampledAt = millis();
            }
        }
        else
        {
            ComputeAndSet();
            TunerState = DONE;
            return 0;
        }
        return pwmHigh;
    }
    default:
        TunerState = IDLE;
        break;
    }

    return 0;
}

void VelocityControl::ComputeAndSet()
{
    float r0 = samples[0];
    float rEnd = samples[index - 1];

    float deltaRPM = rEnd - r0;
    float deltaPWM = pwmHigh - pwmLow;

    // System gain
    float K = deltaRPM / deltaPWM;

    // Time constant τ (63.2% rise)
    float target = r0 + 0.632f * deltaRPM;
    float tau = 0;

    for (uint16_t i = 0; i < index; i++)
    {
        if (samples[i] >= target)
        {
            tau = (i * 10.0f) / 1000.0f; // samples × 10ms
            break;
        }
    }
    // IMC tuning (λ = τ)
    float lambda = tau;
    float Kp, Ki, Kd;
    if (K > 0 && tau > 0)
    {
        Kp = tau / (K * lambda);
        Ki = 1.0f / (tau + lambda);
        Kd = (tau * lambda) / 2.0f;
    }
    else
    {
        Kp = Ki = Kd = 0; // fallback
    }
    _PID.SetConstants(Kp, Ki, Kd);
    PIDLoopConfig newConfig = {Kp, Ki, Kd, tau, K};
    _PIDControlConfig.saveVelocity(newConfig);
}

VelocityControl::TuneMode VelocityControl::GetTunerState()
{
    return TunerState;
}

VelocityControl::~VelocityControl()
{
    delete[] samples;
    samples = nullptr;
}

void VelocityControl::ResetSamples()
{
    delete[] samples;
    samples = new float [MAX_SAMPLES];
}


