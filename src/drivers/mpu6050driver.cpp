/**
 * MIT License
 *
 * Copyright (c) 2020 Kravchenko Artyom
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

#include "mpu6050driver.h"

Mpu6050Driver::Mpu6050Driver() {
    _mpu = new MPU6050(IMU_ADDR);
}

void Mpu6050Driver::init() {
    _mpu->initialize();
    _mpu->setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
    _mpu->setFullScaleGyroRange(MPU6050_GYRO_FS_1000);

    Serial.println(_mpu->testConnection() ? "MPU6050 OK" : "MPU6050 FAIL");
}

imuData_t Mpu6050Driver::getData() {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    _mpu->getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    imuData_t data;
    data.accX = (float)ax / 32768 * 8;
    data.accY = (float)ay / 32768 * 8;
    data.accZ = (float)az / 32768 * 8;
    data.gyroX = (float)gx / 32768 * 1000;
    data.gyroY = (float)gy / 32768 * 1000;
    data.gyroZ = (float)gz / 32768 * 1000;

    return data;
}