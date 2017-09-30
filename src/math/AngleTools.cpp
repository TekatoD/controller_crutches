/**
 * Copyright 2016 Arseniy Ivin <arssivka@yandex.ru>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 *  @autor arssivka
 *  @date 8/14/17
 */

#include "math/AngleTools.h"

float degrees(float rad) {
    return rad / pi * 180.0f;
}

float radians(float deg) {
    return deg * pi / 180.0f;
}

float wsin(float time, float period, float period_shift, float mag, float mag_shift) {
    return mag * sinf(2.0f * pi / period * time - period_shift) + mag_shift;
}
