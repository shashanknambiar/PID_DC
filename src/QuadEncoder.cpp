#include "QuadEncoder.h"

// Static variable definitions
volatile int QuadEncoder::InstantTick = 0;
volatile int QuadEncoder::PrevTick = 0;
uint8_t QuadEncoder::pinA = 0;
uint8_t QuadEncoder::pinB = 0;
volatile float QuadEncoder::RPM = 0;
volatile float QuadEncoder::lastRawRPM [3] = {0};
volatile float QuadEncoder::rpmEma = 0.0f;
volatile int QuadEncoder::emaIdx = 0;
volatile float QuadEncoder::emaAlpha = 0.18f;
// Public Methods
void QuadEncoder::Setup(int EncoderPinA, int EncoderPinB)
{
    pinA = EncoderPinA;
    pinB = EncoderPinB;

    pinMode(pinA, INPUT);
    pinMode(pinB, INPUT);

    attachInterrupt(digitalPinToInterrupt(pinA), QuadEncoder::UpdateEncoderA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pinB), QuadEncoder::UpdateEncoderB, CHANGE);

    // Enable timer1, raise interrupt every 5ms to calculate RPM
    TCCR1A = 0;          // Init Timer1A
    TCCR1B = 0;          // Init Timer1B
    TCCR1B |= B00000010; // Prescaler = 8
    TCNT1 = 25535;       // Timer Preloading
    TIMSK1 |= B00000001; // Enable Timer Overflow Interrupt
}

int QuadEncoder::GetTicks()
{
    return InstantTick;
}

float QuadEncoder::GetAngle()
{
    long ticks = InstantTick;
    long wrapped = ticks;

    while (wrapped >= TICKS_PER_REV)
        wrapped -= TICKS_PER_REV;
    while (wrapped < 0)
        wrapped += TICKS_PER_REV;

    return wrapped * (360.0f / TICKS_PER_REV);
}

float QuadEncoder::GetRPM()
{
    return RPM;
}

void QuadEncoder::CalculateRPM()
{
    //1440 Ticks per revolution
    //ISR trigerred every 20ms
    int delta = InstantTick - PrevTick;
    PrevTick = InstantTick;
    //RPM = (delta / 1440.0f) * (60.0f / 0.020f);
    float rawRPM = (delta / 1440.0f) * (60.0f / 0.020f);
    // median-of-3
    lastRawRPM[emaIdx] = rawRPM;
    emaIdx = (emaIdx + 1) % 3;
    float a = lastRawRPM[0], b = lastRawRPM[1], c = lastRawRPM[2];
    float med = a;
    if ((a <= b && b <= c) || (c <= b && b <= a)) med = b;
    else if ((b <= a && a <= c) || (c <= a && a <= b)) med = a;
    else med = c;

    // EMA on the median
    rpmEma = emaAlpha * med + (1.0f - emaAlpha) * rpmEma;
    RPM = rpmEma;
}

// ISR Handlers
void QuadEncoder::UpdateEncoderA()
{
    int a = digitalRead(pinA);
    int b = digitalRead(pinB);

    if (a == b)
        InstantTick++;
    else
        InstantTick--;
}

void QuadEncoder::UpdateEncoderB()
{
    int a = digitalRead(pinA);
    int b = digitalRead(pinB);

    if (a != b)
        InstantTick++;
    else
        InstantTick--;
}

ISR(TIMER1_OVF_vect)
{
    //Calculate RPM
    TCNT1 = 25535;
    QuadEncoder::CalculateRPM();
}
