/*
 *   Head.h
 *   represents the head of the robot
 *   Author: ROBOTIS
 *
 */

#ifndef _HEAD_H_
#define _HEAD_H_

#include <string.h>
#include <memory>

#include "minIni.h"
#include "motion/MotionModule.h"
#include "math/Point.h"

#define HEAD_SECTION    "Head Pan/Tilt"
#define INVALID_VALUE   -1024.0

namespace Robot {
    class Head
            : public MotionModule {
    private:
        float m_LeftLimit;
        float m_RightLimit;
        float m_TopLimit;
        float m_BottomLimit;
        float m_Pan_Home;
        float m_Tilt_Home;
        float m_Pan_err;
        float m_Pan_err_diff;
        float m_Pan_p_gain;
        float m_Pan_d_gain;
        float m_Tilt_err;
        float m_Tilt_err_diff;
        float m_Tilt_p_gain;
        float m_Tilt_d_gain;
        float m_PanAngle;
        float m_TiltAngle;

        Head();

        void CheckLimit();

    public:
        static Head* GetInstance() {
            static Head head;
            return &head;
        }

        ~Head();

        void Initialize();

        void Process();

        float GetTopLimitAngle() { return m_TopLimit; }

        float GetBottomLimitAngle() { return m_BottomLimit; }

        float GetRightLimitAngle() { return m_RightLimit; }

        float GetLeftLimitAngle() { return m_LeftLimit; }

        float GetPanAngle() { return m_PanAngle; }

        float GetTiltAngle() { return m_TiltAngle; }

        void MoveToHome();

        void MoveByAngle(float pan, float tilt);

        void MoveByAngleOffset(float pan, float tilt);

        void InitTracking();

        void MoveTracking(Point2D err); // For image processing
        void MoveTracking();

/*Read/write from a INI file*/

        void LoadINISettings(minIni* ini);

        void LoadINISettings(minIni* ini, const std::string& section);

        void SaveINISettings(minIni* ini);

        void SaveINISettings(minIni* ini, const std::string& section);
    };
}

#endif
