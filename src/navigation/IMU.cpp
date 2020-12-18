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

#include "IMU.h"
#include "../common/maths.h"
#include "../factories/IMUDriverFactory.h"
#include "../factories/CompassDriverFactory.h"

#define SPIN_RATE_LIMIT             20
#define MAX_ACC_SQ_NEARNESS         25      // 25% or G^2, accepted acceleration of (0.87 - 1.12G)

IMU::IMU() {
    _imu = ImuDriverFactory::build();
    _compass = CompassDriverFactory::build();
}

void IMU::init() {
    _imu->init();
    _compass->init();

    _imuRuntimeConfig.dcm_kp_acc = 2500;             // 0.25 * 10000
    _imuRuntimeConfig.dcm_ki_acc = 50;               // 0.005 * 10000
    _imuRuntimeConfig.dcm_kp_mag = 10000;            // 1.00 * 10000
    _imuRuntimeConfig.dcm_ki_mag = 0;                // 0.00 * 10000;

    // Create magnetic declination matrix
    const int deg = _compass->getConfig()->mag_declination / 100;
    const int min = _compass->getConfig()->mag_declination % 100;
    setMagneticDeclination(deg + min / 60.0f);

    quaternionInitUnit(&_orientation);
    computeRotationMatrix();
}

attitudeEulerAngles_t IMU::getAttitudeData(unsigned long currentTime) {
    float dT = (currentTime - _previousIMUUpdateTime) * 1e-3;
    _previousIMUUpdateTime = currentTime;

    fpVector3_t imuMeasuredAccelBF;
    fpVector3_t imuMeasuredRotationBF;
    fpVector3_t measuredMagBF;
    imuData_t imuData = _imu->getData();
    bool magIsReady = _compass->isReady() && _compass->isCalibated();

    if (magIsReady) {
        compassData_t compassData = _compass->getData();    
        measuredMagBF.v[X] = compassData.x;
        measuredMagBF.v[Y] = compassData.y;
        measuredMagBF.v[Z] = compassData.z;
    }

    imuMeasuredAccelBF.v[X] = imuData.accX;
    imuMeasuredAccelBF.v[Y] = imuData.accY;
    imuMeasuredAccelBF.v[Z] = imuData.accZ;

    imuMeasuredRotationBF.v[X] = imuData.gyroX;
    imuMeasuredRotationBF.v[Y] = imuData.gyroY;
    imuMeasuredRotationBF.v[Z] = imuData.gyroZ;

    gyroGetMeasuredRotationRate(&imuMeasuredRotationBF);    // Calculate gyro rate in body frame in rad/s
    accGetMeasuredAcceleration(&imuMeasuredAccelBF);        // Calculate accel in body frame in cm/s/s

    const float magWeight = getPGainScaleFactor() * 1.0f;
    const float accWeight = getPGainScaleFactor() * calculateAccelerometerWeight(dT, imuMeasuredAccelBF);
    const bool useAcc = (accWeight > 0.001f);

    mahonyAHRSupdate(dT, &imuMeasuredRotationBF,
                        useAcc ? &imuMeasuredAccelBF : NULL,
                        magIsReady ? &measuredMagBF : NULL,
                        false, 0,
                        accWeight,
                        magWeight);

    return getEulerAngles();
}

void IMU::mahonyAHRSupdate(
    float dt,
    const fpVector3_t * gyroBF,
    const fpVector3_t * accBF,
    const fpVector3_t * magBF,
    bool useCOG,
    float courseOverGround,
    float accWScaler,
    float magWScaler
) {
    fpVector3_t vGyroDriftEstimate = { 0 };

    fpQuaternion_t prevOrientation = _orientation;
    fpVector3_t vRotation = *gyroBF;

    /* Calculate general spin rate (rad/s) */
    const float spin_rate_sq = vectorNormSquared(&vRotation);

    /* Step 1: Yaw correction */
    // Use measured magnetic field vector
    if (magBF || useCOG) {
        static const fpVector3_t vForward = { .v = { 1.0f, 0.0f, 0.0f } };

        fpVector3_t vErr = { .v = { 0.0f, 0.0f, 0.0f } };

        if (magBF && vectorNormSquared(magBF) > 0.01f) {
            fpVector3_t vMag;

            // For magnetometer correction we make an assumption that magnetic field is perpendicular to gravity (ignore Z-component in EF).
            // This way magnetic field will only affect heading and wont mess roll/pitch angles

            // (hx; hy; 0) - measured mag field vector in EF (assuming Z-component is zero)
            // This should yield direction to magnetic North (1; 0; 0)
            quaternionRotateVectorInv(&vMag, magBF, &_orientation);    // BF -> EF

            // Ignore magnetic inclination
            vMag.z = 0.0f;

            // We zeroed out vMag.z -  make sure the whole vector didn't go to zero
            if (vectorNormSquared(&vMag) > 0.01f) {
                // Normalize to unit vector
                vectorNormalize(&vMag, &vMag);

                // Reference mag field vector heading is Magnetic North in EF. We compute that by rotating True North vector by declination and assuming Z-component is zero
                // magnetometer error is cross product between estimated magnetic north and measured magnetic north (calculated in EF)
                vectorCrossProduct(&vErr, &vMag, &_vCorrectedMagNorth);

                // Rotate error back into body frame
                quaternionRotateVector(&vErr, &vErr, &_orientation);
            }
        } else if (useCOG) {
            fpVector3_t vHeadingEF;

            // Use raw heading error (from GPS or whatever else)
            while (courseOverGround >  M_PIf) courseOverGround -= (2.0f * M_PIf);
            while (courseOverGround < -M_PIf) courseOverGround += (2.0f * M_PIf);

            // William Premerlani and Paul Bizard, Direction Cosine Matrix IMU - Eqn. 22-23
            // (Rxx; Ryx) - measured (estimated) heading vector (EF)
            // (-cos(COG), sin(COG)) - reference heading vector (EF)

            // Compute heading vector in EF from scalar CoG
            fpVector3_t vCoG = { .v = { -cos_approx(courseOverGround), sin_approx(courseOverGround), 0.0f } };

            // Rotate Forward vector from BF to EF - will yield Heading vector in Earth frame
            quaternionRotateVectorInv(&vHeadingEF, &vForward, &_orientation);
            vHeadingEF.z = 0.0f;

            // We zeroed out vHeadingEF.z -  make sure the whole vector didn't go to zero
            if (vectorNormSquared(&vHeadingEF) > 0.01f) {
                // Normalize to unit vector
                vectorNormalize(&vHeadingEF, &vHeadingEF);

                // error is cross product between reference heading and estimated heading (calculated in EF)
                vectorCrossProduct(&vErr, &vCoG, &vHeadingEF);

                // Rotate error back into body frame
                quaternionRotateVector(&vErr, &vErr, &_orientation);
            }
        }

        // Compute and apply integral feedback if enabled
        if (_imuRuntimeConfig.dcm_ki_mag > 0.0f) {
            // Stop integrating if spinning beyond the certain limit
            if (spin_rate_sq < sq(DEGREES_TO_RADIANS(SPIN_RATE_LIMIT))) {
                fpVector3_t vTmp;

                // integral error scaled by Ki
                vectorScale(&vTmp, &vErr, _imuRuntimeConfig.dcm_ki_mag * dt);
                vectorAdd(&vGyroDriftEstimate, &vGyroDriftEstimate, &vTmp);
            }
        }

        // Calculate kP gain and apply proportional feedback
        vectorScale(&vErr, &vErr, _imuRuntimeConfig.dcm_kp_mag * magWScaler);
        vectorAdd(&vRotation, &vRotation, &vErr);
    }


    /* Step 2: Roll and pitch correction -  use measured acceleration vector */
    if (accBF) {
        static const fpVector3_t vGravity = { .v = { 0.0f, 0.0f, 1.0f } };
        fpVector3_t vEstGravity, vAcc, vErr;

        // Calculate estimated gravity vector in body frame
        quaternionRotateVector(&vEstGravity, &vGravity, &_orientation);    // EF -> BF

        // Error is sum of cross product between estimated direction and measured direction of gravity
        vectorNormalize(&vAcc, accBF);
        vectorCrossProduct(&vErr, &vAcc, &vEstGravity);

        // Compute and apply integral feedback if enabled
        if (_imuRuntimeConfig.dcm_ki_acc > 0.0f) {
            // Stop integrating if spinning beyond the certain limit
            if (spin_rate_sq < sq(DEGREES_TO_RADIANS(SPIN_RATE_LIMIT))) {
                fpVector3_t vTmp;

                // integral error scaled by Ki
                vectorScale(&vTmp, &vErr, _imuRuntimeConfig.dcm_ki_acc * dt);
                vectorAdd(&vGyroDriftEstimate, &vGyroDriftEstimate, &vTmp);
            }
        }

        // Calculate kP gain and apply proportional feedback
        vectorScale(&vErr, &vErr, _imuRuntimeConfig.dcm_kp_acc * accWScaler);
        vectorAdd(&vRotation, &vRotation, &vErr);
    }

    // Apply gyro drift correction
    vectorAdd(&vRotation, &vRotation, &vGyroDriftEstimate);

    // Integrate rate of change of quaternion
    fpVector3_t vTheta;
    fpQuaternion_t deltaQ;

    vectorScale(&vTheta, &vRotation, 0.5f * dt);
    quaternionInitFromVector(&deltaQ, &vTheta);
    const float thetaMagnitudeSq = vectorNormSquared(&vTheta);

    // If calculated rotation is zero - don't update quaternion
    if (thetaMagnitudeSq >= 1e-20) {
        // Calculate quaternion delta:
        // Theta is a axis/angle rotation. Direction of a vector is axis, magnitude is angle/2.
        // Proper quaternion from axis/angle involves computing sin/cos, but the formula becomes numerically unstable as Theta approaches zero.
        // For near-zero cases we use the first 3 terms of the Taylor series expansion for sin/cos. We check if fourth term is less than machine precision -
        // then we can safely use the "low angle" approximated version without loss of accuracy.
        if (thetaMagnitudeSq < sqrtf(24.0f * 1e-6f)) {
            quaternionScale(&deltaQ, &deltaQ, 1.0f - thetaMagnitudeSq / 6.0f);
            deltaQ.q0 = 1.0f - thetaMagnitudeSq / 2.0f;
        }
        else {
            const float thetaMagnitude = sqrtf(thetaMagnitudeSq);
            quaternionScale(&deltaQ, &deltaQ, sin_approx(thetaMagnitude) / thetaMagnitude);
            deltaQ.q0 = cos_approx(thetaMagnitude);
        }

        // Calculate final orientation and renormalize
        quaternionMultiply(&_orientation, &_orientation, &deltaQ);
        quaternionNormalize(&_orientation, &_orientation);
    }

    // Check for invalid quaternion and reset to previous known good one
    checkAndResetOrientationQuaternion(&prevOrientation, accBF);

    // Pre-compute rotation matrix from quaternion
    computeRotationMatrix();
}

attitudeEulerAngles_t IMU::getEulerAngles(void)
{
    attitudeEulerAngles_t attitude;

    /* Compute pitch/roll angles */
    attitude.values.roll = RADIANS_TO_DECIDEGREES(atan2_approx(_rMat[2][1], _rMat[2][2]));
    attitude.values.pitch = RADIANS_TO_DECIDEGREES((0.5f * M_PIf) - acos_approx(-_rMat[2][0]));
    attitude.values.yaw = RADIANS_TO_DECIDEGREES(-atan2_approx(_rMat[1][0], _rMat[0][0]));

    if (attitude.values.yaw < 0) {
        attitude.values.yaw += 3600;
    }

    return attitude;
}

void IMU::computeRotationMatrix(void)
{
    float q1q1 = _orientation.q1 * _orientation.q1;
    float q2q2 = _orientation.q2 * _orientation.q2;
    float q3q3 = _orientation.q3 * _orientation.q3;

    float q0q1 = _orientation.q0 * _orientation.q1;
    float q0q2 = _orientation.q0 * _orientation.q2;
    float q0q3 = _orientation.q0 * _orientation.q3;
    float q1q2 = _orientation.q1 * _orientation.q2;
    float q1q3 = _orientation.q1 * _orientation.q3;
    float q2q3 = _orientation.q2 * _orientation.q3;

    _rMat[0][0] = 1.0f - 2.0f * q2q2 - 2.0f * q3q3;
    _rMat[0][1] = 2.0f * (q1q2 + -q0q3);
    _rMat[0][2] = 2.0f * (q1q3 - -q0q2);

    _rMat[1][0] = 2.0f * (q1q2 - -q0q3);
    _rMat[1][1] = 1.0f - 2.0f * q1q1 - 2.0f * q3q3;
    _rMat[1][2] = 2.0f * (q2q3 + -q0q1);

    _rMat[2][0] = 2.0f * (q1q3 + -q0q2);
    _rMat[2][1] = 2.0f * (q2q3 - -q0q1);
    _rMat[2][2] = 1.0f - 2.0f * q1q1 - 2.0f * q2q2;
}

void IMU::setMagneticDeclination(float declinationDeg)
{
    const float declinationRad = -DEGREES_TO_RADIANS(declinationDeg);
    _vCorrectedMagNorth.x = cos_approx(declinationRad);
    _vCorrectedMagNorth.y = sin_approx(declinationRad);
    _vCorrectedMagNorth.z = 0;
}

void IMU::checkAndResetOrientationQuaternion(const fpQuaternion_t * quat, const fpVector3_t * accBF)
{
    // Check if some calculation in IMU update yield NAN or zero quaternion
    if (validateQuaternion(&_orientation)) {
        return;
    }

    // Orientation is invalid. We need to reset it
    if (validateQuaternion(quat)) {
        // Previous quaternion valid. Reset to it
        _orientation = *quat;
    } else {
        // No valid reference. Best guess from accelerometer
        resetOrientationQuaternion(accBF);
    }
}

bool IMU::validateQuaternion(const fpQuaternion_t * quat)
{
    const float check = fabs(quat->q0) + fabs(quat->q1) + fabs(quat->q2) + fabs(quat->q3);

    if (!isnan(check) && !isinf(check)) {
        return true;
    }

    const float normSq = quaternionNormSqared(&_orientation);
    if (normSq > (1.0f - 1e-6f) && normSq < (1.0f + 1e-6f)) {
        return true;
    }

    return false;
}

void IMU::resetOrientationQuaternion(const fpVector3_t * accBF)
{
    const float accNorm = sqrtf(vectorNormSquared(accBF));

    _orientation.q0 = accBF->z + accNorm;
    _orientation.q1 = accBF->y;
    _orientation.q2 = -accBF->x;
    _orientation.q3 = 0.0f;

    quaternionNormalize(&_orientation, &_orientation);
}

bool IMU::useFastGains(void) {
    return millis() < 20000;
}

float IMU::getPGainScaleFactor(void) {
    if (useFastGains()) {
        return 10.0f;
    } else {
        return 1.0f;
    }
}

float IMU::calculateAccelerometerWeight(const float dT, fpVector3_t imuMeasuredAccelBF) {
    // If centrifugal test passed - do the usual "nearness" style check
    float accMagnitudeSq = 0;

    for (int axis = 0; axis < 3; axis++) {
        accMagnitudeSq += imuMeasuredAccelBF.v[axis] * imuMeasuredAccelBF.v[axis];
    }

    // Magnitude^2 in percent of G^2
    const float nearness = ABS(100 - (accMagnitudeSq * 100));
    const float accWeight_Nearness = (nearness > MAX_ACC_SQ_NEARNESS) ? 0.0f : 1.0f;

    return accWeight_Nearness;
}

void IMU::gyroGetMeasuredRotationRate(fpVector3_t *measuredRotationRate)
{
    for (int axis = 0; axis < 3; axis++) {
        measuredRotationRate->v[axis] = DEGREES_TO_RADIANS(measuredRotationRate->v[axis]);
    }
}

void IMU::accGetMeasuredAcceleration(fpVector3_t *measuredAcc)
{
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        measuredAcc->v[axis] *= GRAVITY_CMSS;
    }
}