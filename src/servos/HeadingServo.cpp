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

#include "HeadingServo.h"

HeadingServo::HeadingServo() {
    // TODO: Make servo calibration and store and load this values from config
    _centerValue = 1489;
    _startMovementOffset = 54;

    _servo = new Servo();
    _servo->attach(SERVO_HEADING);
    _servo->writeMicroseconds(_centerValue);

    // TODO: load coeffs from config
    _pid = new Pid(0.1f, 0.0f, 0.0f);
}

void HeadingServo::update(int input, int setpoint, float dT) {
    int16_t error = input - setpoint;

    if (error <= -1800) {
        error += 3600;
    }

    if (error >= 1800) {
        error -= 3600;
    }

    int influence = _pid->compute(abs(error), 0, dT);

    if (error < 0) {
        influence = _centerValue + _startMovementOffset - influence;
    } else if (error > 0) {
        influence = _centerValue - _startMovementOffset + influence;
    } else {
        influence = _centerValue;
    }

    if (influence > 2000) {
        influence = 2000;
    }

    if (influence < 1000) {
        influence = 1000;
    }

    _servo->writeMicroseconds(influence);
}
