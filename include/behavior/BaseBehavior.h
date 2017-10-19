#pragma once


#include <motion/MotionStatus.h>
#include "Behavior.h"

namespace Robot {
    class BaseBehavior : public Behavior {
    public:

    protected:
        static bool IsFallen() {
            return MotionStatus::FALLEN != STANDUP;
        }

        static void RaiseUp() {

        }

    private:
    };
}

