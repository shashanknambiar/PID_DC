#ifndef QUADENCODER_H
#define QUADENCODER_H

#include <Arduino.h>

class QuadEncoder
{
public:
    /*
     * Timer 1 Overflow ISR will be occupied when this setup is completed
     */
    void Setup(int EncoderPinA, int EncoderPinB);

    int GetTicks();
    float GetAngle();
    float GetRPM();
    static void CalculateRPM();
private:
    static void UpdateEncoderA();
    static void UpdateEncoderB();
    static volatile int InstantTick;
    static volatile int PrevTick;
    static volatile float RPM;
    static volatile float lastRawRPM [3];
    static volatile float rpmEma;
    static volatile int emaIdx;
    static volatile float emaAlpha; 
    
    static uint8_t pinA;
    static uint8_t pinB;

    static constexpr int TICKS_PER_REV = 1440;
};

#endif