#pragma once

#include <cstring>
#include <memory>

#include "motion/MotionModule.h"
#include "math/Point.h"

namespace Robot {
    class Head
            : public MotionModule {
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

        float GetLeftLimit() const;

        void SetLeftLimit(float left_limit);

        float GetRightLimit() const;

        void SetRightLimit(float right_limit);

        float GetTopLimit() const;

        void SetTopLimit(float top_limit);

        float GetBottomLimit() const;

        void SetBottomLimit(float bottom_limit);

        float GetPanHome() const;

        void SetPanHome(float pan_home);

        float GetTiltHome() const;

        void SetTiltHome(float tilt_home);

        float GetPanPGain() const;

        void SetPanPGain(float pan_p_gain);

        float GetPanDGain() const;

        void SetPanDGain(float pan_d_gain);

        float GetTiltPGain() const;

        void SetTiltPGain(float tilt_p_gain);

        float GetTiltDGain() const;

        void SetTiltDGain(float tilt_d_gain);

        bool IsDebugEnabled() const;

        void EnableDebug(bool debug);

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
        bool m_debug{true};

        Head();

        void CheckLimit();

    };
}