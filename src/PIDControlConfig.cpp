#include "PIDControlConfig.h"
#include "EEPROM.h"

PIDControlConfig::PIDControlConfig() {}

// CRC-16 (Modbus polynomial)
uint16_t PIDControlConfig::computeCRC(const PIDLoopConfig &cfg) {
    const uint8_t *data = reinterpret_cast<const uint8_t*>(&cfg);
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < sizeof(PIDLoopConfig); i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
        }
    }
    return crc;
}

bool PIDControlConfig::loadBlock(int addr, PIDLoopConfig &cfg) {
    EEPROM.get(addr, cfg);

    uint16_t storedCRC;
    EEPROM.get(addr + sizeof(PIDLoopConfig), storedCRC);

    uint16_t calcCRC = computeCRC(cfg);

    return storedCRC == calcCRC;
}

void PIDControlConfig::saveBlock(int addr, const PIDLoopConfig &cfg) {
    EEPROM.put(addr, cfg);

    uint16_t crc = computeCRC(cfg);
    EEPROM.put(addr + sizeof(PIDLoopConfig), crc);
}

bool PIDControlConfig::loadVelocity(PIDLoopConfig &cfg, const PIDLoopConfig& defaults) {
    if(loadBlock(ADDR_VELOCITY, cfg)) return true;
    
    cfg = defaults;
    return false;
}

bool PIDControlConfig::loadPosition(PIDLoopConfig &cfg, const PIDLoopConfig& defaults) {
    if(loadBlock(ADDR_POSITION, cfg)) return true;

    cfg = defaults;
    return false;
}

void PIDControlConfig::saveVelocity(const PIDLoopConfig &cfg) {
    saveBlock(ADDR_VELOCITY, cfg);
}

void PIDControlConfig::savePosition(const PIDLoopConfig &cfg) {
    saveBlock(ADDR_POSITION, cfg);
}