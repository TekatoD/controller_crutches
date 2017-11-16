/*
 *   MotionModule.h
 *
 *   Author: ROBOTIS
 *
 */

#pragma once
#include "motion/joint_data_t.h"

namespace drwn {

    /*
    Represents an abstract motion (maybe instanciated by a walking, etc.)
    */
    class motion_module_t {
    private:

    protected:

    public:
        /*state of all the articulations (the motors MX-28)*/
        joint_data_t joint;

        static const int TIME_UNIT = 8; //msec

        virtual void initialize() = 0;

        virtual void process() = 0;
    };
}

