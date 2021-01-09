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

#include "alignment.h"

void applySensorAlignment(int32_t * dest, int32_t * src, SensorAlign_e rotation)
{
    // Create a copy so we could use the same buffer for src & dest
    const int32_t x = src[X];
    const int32_t y = src[Y];
    const int32_t z = src[Z];

    switch (rotation) {
    default:
    case CW0_DEG:
        dest[X] = x;
        dest[Y] = y;
        dest[Z] = z;
        break;
    case CW90_DEG:
        dest[X] = y;
        dest[Y] = -x;
        dest[Z] = z;
        break;
    case CW180_DEG:
        dest[X] = -x;
        dest[Y] = -y;
        dest[Z] = z;
        break;
    case CW270_DEG:
        dest[X] = -y;
        dest[Y] = x;
        dest[Z] = z;
        break;
    case CW0_DEG_FLIP:
        dest[X] = -x;
        dest[Y] = y;
        dest[Z] = -z;
        break;
    case CW90_DEG_FLIP:
        dest[X] = y;
        dest[Y] = x;
        dest[Z] = -z;
        break;
    case CW180_DEG_FLIP:
        dest[X] = x;
        dest[Y] = -y;
        dest[Z] = -z;
        break;
    case CW270_DEG_FLIP:
        dest[X] = -y;
        dest[Y] = -x;
        dest[Z] = -z;
        break;
    }
}
