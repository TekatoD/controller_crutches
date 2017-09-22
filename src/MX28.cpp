/*
 *   MX28.cpp
 *
 *  
 *
 */
#include "MX28.h"

using namespace Robot;

const int MX28::MIN_VALUE = 0;

const int MX28::CENTER_VALUE = 2048;
const int MX28::MAX_VALUE = 4095;
const float MX28::MIN_DEGREES = -180.0; // degree
const float MX28::MAX_DEGREES = 180.0; // degree
const float MX28::RATIO_VALUE2DEGREES = 0.088; // 360 / 4096
const float MX28::RATIO_DEGREES2VALUE = 11.378; // 4096 / 360

const int MX28::PARAM_BYTES = 7;
