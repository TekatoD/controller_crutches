/*
 *   MX28.h
 *   MX28 is a motor. The DARwin robot is made up of 20 motors.
 *   This class only provide "static" features: enum, constants and conversion functions.
 *   Author: ROBOTIS
 *
 */

#ifndef _MX_28_H_
#define _MX_28_H_

#include <cmath>

namespace Robot {
    class MX28 {
    public:
        static constexpr int MIN_VALUE = 0;
        static constexpr int CENTER_VALUE = 2048;
        static constexpr int MAX_VALUE = 4095;
        static constexpr float MIN_DEGREES = -180.0f; // degree
        static constexpr float MAX_DEGREES = 180.0f; // degree
        static constexpr float MIN_RADIANS = -3.1415927f;
        static constexpr float MAX_RADIANS = 3.1415927f;
        static constexpr float RATIO_VALUE2DEGREES = 0.088f; // 360 / 4096
        static constexpr float RATIO_DEGREES2VALUE = 11.378f; // 4096 / 360
        static constexpr float RATIO_VALUE2RADIANS = 0.00076699042f; // pi / 4096
        static constexpr float RATIO_RADIANS2VALUE = 1303.7972f; // 4096 / pi
        static constexpr int PARAM_BYTES = 7;

        static constexpr float RATIO_VALUE2SPEED = 0.053;
        static constexpr float RATIO_SPEED2VALUE = 18.87;

        static constexpr float RATIO_VALUE2TORQUE = 0.01;
        static constexpr float RATIO_TORQUE2VALUE = 100;

        static int GetMirrorValue(int value) { return MAX_VALUE + 1 - value; }

        static float GetMirrorAngle(float angle) { return -angle; }

        static int Angle2Value(float angle) { return (int) (angle * RATIO_DEGREES2VALUE) + CENTER_VALUE; }

        static float Value2Angle(int value) { return (float) (value - CENTER_VALUE) * RATIO_VALUE2DEGREES; }

        static float Speed2Value(float speed) {
            int temp = ((int) (fabs(speed) * RATIO_SPEED2VALUE)) & 0x3FF;
            if (speed < 0) temp |= 0x400;
            return temp;
        }

        static float Value2Speed(int value) {
            float temp = (value & 0x3FF) * RATIO_VALUE2SPEED;
            if (value & 0x400) temp = -temp;
            return temp;
        }

        static float Torque2Value(float speed) {
            int temp = ((int) (fabs(speed) * RATIO_TORQUE2VALUE)) & 0x3FF;
            if (speed < 0) temp |= 0x400;
            return temp;
        }

        static float Value2Torque(int value) {
            float temp = (value & 0x3FF) * RATIO_VALUE2TORQUE;
            if (value & 0x400) temp = -temp;
            return temp;
        }

        /*EEPROM and RAM p. 4 in MX28 Technical Specifications.pdf
         This enum enumerates the adresses.
        */
        enum {
            P_MODEL_NUMBER_L = 0,
            P_MODEL_NUMBER_H = 1,
            P_VERSION = 2,
            P_ID = 3,
            P_BAUD_RATE = 4,
            P_RETURN_DELAY_TIME = 5,
            P_CW_ANGLE_LIMIT_L = 6,
            P_CW_ANGLE_LIMIT_H = 7,
            P_CCW_ANGLE_LIMIT_L = 8,
            P_CCW_ANGLE_LIMIT_H = 9,
            P_SYSTEM_DATA2 = 10,
            P_HIGH_LIMIT_TEMPERATURE = 11,
            P_LOW_LIMIT_VOLTAGE = 12,
            P_HIGH_LIMIT_VOLTAGE = 13,
            P_MAX_TORQUE_L = 14,
            P_MAX_TORQUE_H = 15,
            P_RETURN_LEVEL = 16,
            P_ALARM_LED = 17,
            P_ALARM_SHUTDOWN = 18,
            P_OPERATING_MODE = 19,
            P_MULTI_TURN_OFFSET_L = 20,
            P_MULTI_TURN_OFFSET_H = 21,
            P_RESOLUTION_DIVIDER = 22,
            P_TORQUE_ENABLE = 24,
            P_LED = 25,
            P_D_GAIN = 26,
            P_I_GAIN = 27,
            P_P_GAIN = 28,
            P_RESERVED = 29,
            P_GOAL_POSITION_L = 30,
            P_GOAL_POSITION_H = 31,
            P_MOVING_SPEED_L = 32,
            P_MOVING_SPEED_H = 33,
            P_TORQUE_LIMIT_L = 34,
            P_TORQUE_LIMIT_H = 35,
            P_PRESENT_POSITION_L = 36,
            P_PRESENT_POSITION_H = 37,
            P_PRESENT_SPEED_L = 38,
            P_PRESENT_SPEED_H = 39,
            P_PRESENT_LOAD_L = 40,
            P_PRESENT_LOAD_H = 41,
            P_PRESENT_VOLTAGE = 42,
            P_PRESENT_TEMPERATURE = 43,
            P_REGISTERED_INSTRUCTION = 44,
            P_PAUSE_TIME = 45,
            P_MOVING = 46,
            P_LOCK = 47,
            P_PUNCH_L = 48,
            P_PUNCH_H = 49,
            P_RESERVED4 = 50,
            P_RESERVED5 = 51,
            P_POT_L = 52,
            P_POT_H = 53,
            P_PWM_OUT_L = 54,
            P_PWM_OUT_H = 55,
            P_P_ERROR_L = 56,
            P_P_ERROR_H = 57,
            P_I_ERROR_L = 58,
            P_I_ERROR_H = 59,
            P_D_ERROR_L = 60,
            P_D_ERROR_H = 61,
            P_P_ERROR_OUT_L = 62,
            P_P_ERROR_OUT_H = 63,
            P_I_ERROR_OUT_L = 64,
            P_I_ERROR_OUT_H = 65,
            P_D_ERROR_OUT_L = 66,
            P_D_ERROR_OUT_H = 67,
            P_GOAL_ACCELERATION = 73,
            MAXNUM_ADDRESS
        };
    };
}

#endif
