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

#include "Arduino.h"
#include "AngleServo.h"

AngleServo::AngleServo() {
    // TODO: load from config
    int centerValue = 1250;

    _servo = new Servo();
    _servo->attach(SERVO_ANGLE);
    _servo->writeMicroseconds(centerValue);
    _prev = centerValue;

    // TODO: load coeffs from config
    _pid = new Pid(0.1f, 0.0000005f, 0.005f);
}

void AngleServo::update(int input, int setpoint, float dT) {
    int influence = _pid->compute(abs(input), setpoint, dT);

    if (setpoint < input) {
        influence *= -1;
    }

    int newVal = _prev + influence;

    if (newVal > 2000) {
        newVal = 2000;
    }

    if (newVal < 1000) {
        newVal = 1000;
    }

    _servo->writeMicroseconds(newVal);
    _prev = newVal;
}