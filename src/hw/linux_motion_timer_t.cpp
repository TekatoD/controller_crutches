/*
 *   LinuxMotionTimer.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include "motion/motion_module_t.h"
#include "hw/linux_motion_timer_t.h"

#include <stdlib.h>
#include <string.h>
#include <stdexcept>

using namespace drwn;


linux_motion_timer_t::linux_motion_timer_t(motion_manager_t* manager)
        : m_manager(manager) {
    m_finish_timer = false;
    m_timer_running = false;
}


void* linux_motion_timer_t::timer_proc(void* param) {
    auto* timer = (linux_motion_timer_t*) param;
    static struct timespec next_time;
    clock_gettime(CLOCK_MONOTONIC, &next_time);

    while (!timer->m_finish_timer) {
        next_time.tv_sec += (next_time.tv_nsec + motion_module_t::TIME_UNIT * 1000000) / 1000000000;
        next_time.tv_nsec = (next_time.tv_nsec + motion_module_t::TIME_UNIT * 1000000) % 1000000000;

        if (timer->m_manager != nullptr)
            timer->m_manager->process();

        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, nullptr);
    }

    pthread_exit(nullptr);
}


void linux_motion_timer_t::start() {
    int error;
    sched_param param;
    pthread_attr_t attr;

    pthread_attr_init(&attr);

    error = pthread_attr_setschedpolicy(&attr, SCHED_RR);
    if (error != 0)
        throw std::runtime_error("Linux motion timer can't call pthread_attr_setschedpolicy. Error = " +
                                         std::to_string(error));
    error = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    if (error != 0)
        throw std::runtime_error("Linux motion timer can't call pthread_attr_setinheritsched. Error = " +
                                         std::to_string(error));

    memset(&param, 0, sizeof(param));
    param.sched_priority = 31;// RT
    error = pthread_attr_setschedparam(&attr, &param);
    if (error != 0)
        throw std::runtime_error("Linux motion timer can't call pthread_attr_setschedparam. Error = " +
                                         std::to_string(error));

    // create and start the thread
    if ((error = pthread_create(&this->m_thread, &attr, this->timer_proc, this)) != 0)
        exit(-1);

    this->m_timer_running = true;

}


void linux_motion_timer_t::stop() {
    int error = 0;

    // seti the flag to end the thread
    if (this->m_timer_running) {
        this->m_finish_timer = true;
        // wait for the thread to end
        if ((error = pthread_join(this->m_thread, nullptr)) != 0)
            exit(-1);
        this->m_finish_timer = false;
        this->m_timer_running = false;
    }
}


bool linux_motion_timer_t::is_running() {
    return this->m_timer_running;
}


linux_motion_timer_t::~linux_motion_timer_t() {
    this->stop();
    this->m_manager = nullptr;
}
