/*
 *   LinuxMotionTimer.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include "motion/MotionModule.h"
#include "hw/LinuxMotionTimer.h"

#include <stdlib.h>
#include <string.h>
#include <stdexcept>

using namespace Robot;


LinuxMotionTimer::LinuxMotionTimer(MotionManager* manager)
        : m_Manager(manager) {
    m_FinishTimer = false;
    m_TimerRunning = false;
}


void* LinuxMotionTimer::TimerProc(void* param) {
    auto* timer = (LinuxMotionTimer*) param;
    static struct timespec next_time;
    clock_gettime(CLOCK_MONOTONIC, &next_time);

    while (!timer->m_FinishTimer) {
        next_time.tv_sec += (next_time.tv_nsec + MotionModule::TIME_UNIT * 1000000) / 1000000000;
        next_time.tv_nsec = (next_time.tv_nsec + MotionModule::TIME_UNIT * 1000000) % 1000000000;

        if (timer->m_Manager != nullptr)
            timer->m_Manager->Process();

        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, nullptr);
    }

    pthread_exit(nullptr);
}


void LinuxMotionTimer::Start() {
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
    if ((error = pthread_create(&this->m_Thread, &attr, this->TimerProc, this)) != 0)
        exit(-1);

    this->m_TimerRunning = true;

}


void LinuxMotionTimer::Stop() {
    int error = 0;

    // seti the flag to end the thread
    if (this->m_TimerRunning) {
        this->m_FinishTimer = true;
        // wait for the thread to end
        if ((error = pthread_join(this->m_Thread, nullptr)) != 0)
            exit(-1);
        this->m_FinishTimer = false;
        this->m_TimerRunning = false;
    }
}


bool LinuxMotionTimer::IsRunning() {
    return this->m_TimerRunning;
}


LinuxMotionTimer::~LinuxMotionTimer() {
    this->Stop();
    this->m_Manager = nullptr;
}
