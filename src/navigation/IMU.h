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

#ifndef H_IMU_H
#define H_IMU_H

#include "../common/axis.h"
#include "../drivers/AccGyroDriver.h"
#include "../drivers/CompassDriver.h"
#include "../common/quaternion.h"

typedef struct imuRuntimeConfig_s {
    float dcm_kp_acc;
    float dcm_ki_acc;
    float dcm_kp_mag;
    float dcm_ki_mag;
} imuRuntimeConfig_t;

typedef union {
    int16_t raw[XYZ_AXIS_COUNT];
    struct {
        // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
        int16_t roll;
        int16_t pitch;
        int16_t yaw;
    } values;
} attitudeEulerAngles_t;

class IMU {
public:
    IMU();
    void init();

    attitudeEulerAngles_t getAttitudeData(unsigned long currentTime);

protected:
    void mahonyAHRSupdate(
        float dt,
        const fpVector3_t * gyroBF,
        const fpVector3_t * accBF,
        const fpVector3_t * magBF,
        bool useCOG,
        float courseOverGround,
        float accWScaler,
        float magWScaler
    );
    attitudeEulerAngles_t getEulerAngles(void);
    void setMagneticDeclination(float declinationDeg);
    void computeRotationMatrix(void);
    void checkAndResetOrientationQuaternion(const fpQuaternion_t * quat, const fpVector3_t * accBF);
    bool validateQuaternion(const fpQuaternion_t * quat);
    void resetOrientationQuaternion(const fpVector3_t * accBF);

    bool useFastGains(void);
    float getPGainScaleFactor(void);
    float calculateAccelerometerWeight(const float dT, fpVector3_t imuMeasuredAccelBF);

    void gyroGetMeasuredRotationRate(fpVector3_t *measuredRotationRate);
    void accGetMeasuredAcceleration(fpVector3_t *measuredAcc);

protected:
    AccGyroDriver* _accGyro;
    CompassDriver* _compass;

    imuRuntimeConfig_t _imuRuntimeConfig;

    unsigned long _previousIMUUpdateTime;

    fpQuaternion_t _orientation;
    float _rMat[3][3];
    fpVector3_t _vCorrectedMagNorth;
};

#endif
