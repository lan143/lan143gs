/**
 * MIT License
 *
 * Copyright (c) 2021 Kravchenko Artyom
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "MPU6050Driver.h"

#define BUFFER_SIZE 100

MPU6050Driver::MPU6050Driver() {
    _mpu = new MPU6050(IMU_ADDR);
}

void MPU6050Driver::driverInit() {
    _mpu->initialize();
    _mpu->setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
    _mpu->setFullScaleGyroRange(MPU6050_GYRO_FS_2000);

    gyroScale = 1.0f / 16.4f;
    accScale = 512 * 4;
    
    _mpu->setXAccelOffset(0);
    _mpu->setYAccelOffset(0);
    _mpu->setZAccelOffset(0);
    _mpu->setXGyroOffset(0);
    _mpu->setYGyroOffset(0);
    _mpu->setZGyroOffset(0);

    Serial.println(_mpu->testConnection() ? "MPU6050 OK" : "MPU6050 FAIL");
}

void MPU6050Driver::updateData() {
    int16_t accX, accY, accZ, gyroX, gyroY, gyroZ;
    _mpu->getMotion6(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);

    accADCRaw[X] = accX;
    accADCRaw[Y] = accY;
    accADCRaw[Z] = accZ;

    gyroADCRaw[X] = gyroX;
    gyroADCRaw[Y] = gyroY;
    gyroADCRaw[Z] = gyroZ;
}
