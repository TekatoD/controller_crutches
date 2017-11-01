/*
 *   LinuxMotionTimer.h
 *
 *   Author: ROBOTIS
 *
 */

#pragma once

#include <pthread.h>
#include "motion/motion_manager_t.h"
#include <time.h>

namespace drwn {
    class linux_motion_timer_t {
    private:
        pthread_t m_thread;// thread structure
        unsigned long m_interval_ns;
        motion_manager_t* m_manager;// reference to the motion manager class.
        bool m_timer_running;
        bool m_finish_timer;

    protected:
        static void* timer_proc(void* param);// thread function

    public:
        explicit linux_motion_timer_t(motion_manager_t* manager);

        ~linux_motion_timer_t();

        void start();

        void stop();

        bool is_running();
    };
}

