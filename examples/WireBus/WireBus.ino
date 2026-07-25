#include <Arduino.h>

// Arduino Wire
#include "MPU6050.h"
MPU6050 mpu;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    mpu.begin();
}

void loop() {
    Serial.println(mpu.readTemp());
    delay(100);
}

// Soft Wire
// #include "MPU6050Uni.h"
// SoftWire swire;
// MPU6050Uni<SoftWire> mpu(swire);

// void setup() {
//     Serial.begin(115200);
//     swire.begin(A4, A5);
//     mpu.begin();
// }

// void loop() {
//     Serial.println(mpu.readTemp());
//     delay(100);
// }
