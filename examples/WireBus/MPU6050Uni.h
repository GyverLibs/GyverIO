#pragma once
#include <GyverIO.h>

#define MPU_ADDR 0x68
#define MPU_PWR_MGMT_1 0x6B
#define MPU_TEMP_OUT_H 0x41

template <class WireClass>
class MPU6050Uni : public WireBus<WireClass> {
   public:
    MPU6050Uni(WireClass& wire, uint8_t addr = 0x68) : WireBus<WireClass>(wire, addr) {}

    bool begin() {
        return this->writeReg(MPU_PWR_MGMT_1, 1);
    }

    float readTemp() {
        uint16_t raw = 0;
        this->readRegBE16(MPU_TEMP_OUT_H, raw);
        return int16_t(raw) / 340.0f + 36.53f;
    }
};