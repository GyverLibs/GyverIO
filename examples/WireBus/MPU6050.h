#pragma once
#include <Wire.h>

#include "MPU6050Uni.h"

class MPU6050 : public MPU6050Uni<TwoWire> {
   public:
    MPU6050(uint8_t addr = 0x68, TwoWire& wire = Wire) : MPU6050Uni<TwoWire>(wire, addr) {}
};