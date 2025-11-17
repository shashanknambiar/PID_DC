#ifndef PID_CONTROL_CONFIG_H
#define PID_CONTROL_CONFIG_H

#include <Arduino.h>

struct PIDLoopConfig {
    float kp;
    float ki;
    float kd;
    float timeConstant;
    float steadyStateGain;
    uint32_t timestamp;    // millis() of tuning
};

class PIDControlConfig {
public:
    PIDControlConfig();

    bool loadVelocity(PIDLoopConfig &cfg, const PIDLoopConfig& defaults);
    bool loadPosition(PIDLoopConfig &cfg, const PIDLoopConfig& defaults);

    void saveVelocity(const PIDLoopConfig &cfg);
    void savePosition(const PIDLoopConfig &cfg);

private:
    uint16_t computeCRC(const PIDLoopConfig &cfg);
    bool loadBlock(int addr, PIDLoopConfig &cfg);
    void saveBlock(int addr, const PIDLoopConfig &cfg);

private:
    static constexpr int BLOCK_SIZE = sizeof(PIDLoopConfig) + sizeof(uint16_t);

    static constexpr int ADDR_VELOCITY = 0;
    static constexpr int ADDR_POSITION = ADDR_VELOCITY + BLOCK_SIZE;
};

#endif
