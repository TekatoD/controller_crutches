/*
 *   MotionManager.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include <stdio.h>
#include <math.h>
#include <log/Logger.h>
#include "hw/FSR.h"
#include "hw/MX28.h"
#include "motion/MotionManager.h"

using namespace Robot;

MotionManager::MotionManager() {
    for (int i = 0; i < JointData::NUMBER_OF_JOINTS; i++)
        m_Offset[i] = 0;
}

bool MotionManager::Initialize(CM730* cm730) {
    int value, error;

    m_CM730 = cm730;
    m_Enabled = false;
    m_ProcessEnable = true;

    if (!m_CM730->Connect()) {
        if (m_debug) LOG_ERROR << "Fail to connect CM-730";
        return false;
    }

    for (int id = JointData::ID_R_SHOULDER_PITCH; id < JointData::NUMBER_OF_JOINTS; id++) {
        if (m_debug) LOG_DEBUG << "MOTION_MANAGER: ID: " << id << " initializing...";

        if (m_CM730->ReadWord(id, MX28::P_PRESENT_POSITION_L, &value, &error) == CM730::SUCCESS) {
            MotionStatus::m_CurrentJoints.SetValue(id, value);
            MotionStatus::m_CurrentJoints.SetEnable(id, true);

            if (m_debug) LOG_DEBUG << "MOTION MANAGER: [" << value << "] Success";
        } else {
            MotionStatus::m_CurrentJoints.SetEnable(id, false);

            if (m_debug) LOG_ERROR << "MOTION MANAGER: Fail";
        }
    }

    m_CalibrationStatus = 0;
    m_FBGyroCenter = 512;
    m_RLGyroCenter = 512;

    return true;
}


bool MotionManager::Reinitialize() {
    m_ProcessEnable = false;

    m_CM730->DXLPowerOn();

    int value, error;
    for (int id = JointData::ID_R_SHOULDER_PITCH; id < JointData::NUMBER_OF_JOINTS; id++) {
        if (m_debug) LOG_DEBUG << "MOTION MANAGER: ID: " << id << " initializing...";

        if (m_CM730->ReadWord(id, MX28::P_PRESENT_POSITION_L, &value, &error) == CM730::SUCCESS) {
            MotionStatus::m_CurrentJoints.SetValue(id, value);
            MotionStatus::m_CurrentJoints.SetEnable(id, true);

            if (m_debug) LOG_DEBUG << "MOTION MANAGERL: [" << value << "] Success";
        } else {
            MotionStatus::m_CurrentJoints.SetEnable(id, false);

            if (m_debug) LOG_ERROR << "MOTION MANAGER: Fail";
        }
    }

    m_ProcessEnable = true;
    return true;
}

void MotionManager::Process() {
    constexpr int GYRO_WINDOW_SIZE = 100;
    constexpr int ACCEL_WINDOW_SIZE = 30;
    constexpr float MARGIN_OF_SD = 2.0;

    if (!m_ProcessEnable || m_IsRunning)
        return;

    m_IsRunning = true;

    // calibrate gyro sensor
    if (m_CalibrationStatus == 0 || m_CalibrationStatus == -1) {
        static int fb_gyro_array[GYRO_WINDOW_SIZE] = {512,};
        static int rl_gyro_array[GYRO_WINDOW_SIZE] = {512,};
        static int buf_idx = 0;

        if (buf_idx < GYRO_WINDOW_SIZE) {
            if (m_CM730->m_BulkReadData[CM730::ID_CM].error == 0) {
                fb_gyro_array[buf_idx] = m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_GYRO_Y_L);
                rl_gyro_array[buf_idx] = m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_GYRO_X_L);
                buf_idx++;
            }
        } else {
            float fb_sum = 0.0, rl_sum = 0.0;
            float fb_sd = 0.0, rl_sd = 0.0;
            float fb_diff, rl_diff;
            float fb_mean = 0.0, rl_mean = 0.0;

            buf_idx = 0;

            for (int i = 0; i < GYRO_WINDOW_SIZE; i++) {
                fb_sum += fb_gyro_array[i];
                rl_sum += rl_gyro_array[i];
            }
            fb_mean = fb_sum / GYRO_WINDOW_SIZE;
            rl_mean = rl_sum / GYRO_WINDOW_SIZE;

            fb_sum = 0.0;
            rl_sum = 0.0;
            for (int i = 0; i < GYRO_WINDOW_SIZE; i++) {
                fb_diff = fb_gyro_array[i] - fb_mean;
                rl_diff = rl_gyro_array[i] - rl_mean;
                fb_sum += fb_diff * fb_diff;
                rl_sum += rl_diff * rl_diff;
            }
            fb_sd = sqrt(fb_sum / GYRO_WINDOW_SIZE);
            rl_sd = sqrt(rl_sum / GYRO_WINDOW_SIZE);

            if (fb_sd < MARGIN_OF_SD && rl_sd < MARGIN_OF_SD) {
                m_FBGyroCenter = (int) fb_mean;
                m_RLGyroCenter = (int) rl_mean;
                m_CalibrationStatus = 1;
                if (m_debug) {
                    LOG_DEBUG << "MOTION MANAGER: FBGyroCenter: " << m_FBGyroCenter
                              << ", RLGyroCenter: " << m_RLGyroCenter;
                }
            } else {
                m_FBGyroCenter = 512;
                m_RLGyroCenter = 512;
                m_CalibrationStatus = -1;
            }
        }
    }

    if (m_CalibrationStatus == 1 && m_Enabled) {
        static int fb_array[ACCEL_WINDOW_SIZE] = {512,};
        static int buf_idx = 0;
        if (m_CM730->m_BulkReadData[CM730::ID_CM].error == 0) {
            MotionStatus::FB_GYRO = m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_GYRO_Y_L) - m_FBGyroCenter;
            MotionStatus::RL_GYRO = m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_GYRO_X_L) - m_RLGyroCenter;
            MotionStatus::RL_ACCEL = m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_ACCEL_X_L);
            MotionStatus::FB_ACCEL = m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_ACCEL_Y_L);
            fb_array[buf_idx] = MotionStatus::FB_ACCEL;
            if (++buf_idx >= ACCEL_WINDOW_SIZE) buf_idx = 0;
        }

        int sum = 0, avr = 512;
        for (int idx = 0; idx < ACCEL_WINDOW_SIZE; idx++)
            sum += fb_array[idx];
        avr = sum / ACCEL_WINDOW_SIZE;

        if (avr < MotionStatus::FALLEN_F_LIMIT)
            MotionStatus::FALLEN = FORWARD;
        else if (avr > MotionStatus::FALLEN_B_LIMIT)
            MotionStatus::FALLEN = BACKWARD;
        else
            MotionStatus::FALLEN = STANDUP;

        if (!m_Modules.empty()) {
            for (auto& module : m_Modules) {
                module->Process();
                for (int id = JointData::ID_R_SHOULDER_PITCH; id < JointData::NUMBER_OF_JOINTS; id++) {
                    if (module->m_Joint.GetEnable(id)) {
                        MotionStatus::m_CurrentJoints.SetValue(id, module->m_Joint.GetValue(id));

                        MotionStatus::m_CurrentJoints.SetPGain(id, module->m_Joint.GetPGain(id));
                        MotionStatus::m_CurrentJoints.SetIGain(id, module->m_Joint.GetIGain(id));
                        MotionStatus::m_CurrentJoints.SetDGain(id, module->m_Joint.GetDGain(id));
                    }
                }
            }
        }

        int param[JointData::NUMBER_OF_JOINTS * MX28::PARAM_BYTES];
        int n = 0;
        int joint_num = 0;
        for (int id = JointData::ID_R_SHOULDER_PITCH; id < JointData::NUMBER_OF_JOINTS; id++) {
            if (MotionStatus::m_CurrentJoints.GetEnable(id)) {
                param[n++] = id;
                param[n++] = MotionStatus::m_CurrentJoints.GetDGain(id);
                param[n++] = MotionStatus::m_CurrentJoints.GetIGain(id);
                param[n++] = MotionStatus::m_CurrentJoints.GetPGain(id);
                param[n++] = 0;
                param[n++] = CM730::GetLowByte(MotionStatus::m_CurrentJoints.GetValue(id) + m_Offset[id]);
                param[n++] = CM730::GetHighByte(MotionStatus::m_CurrentJoints.GetValue(id) + m_Offset[id]);
                joint_num++;
            }

            if (m_debug)
                LOG_DEBUG << "MOTION MANAGER: ID[" << id << "] : " << MotionStatus::m_CurrentJoints.GetValue(id);
        }

        if (joint_num > 0)
            m_CM730->SyncWrite(MX28::P_D_GAIN, MX28::PARAM_BYTES, joint_num, param);
    }

    m_CM730->BulkRead();

    if (m_CM730->m_BulkReadData[CM730::ID_CM].error == 0)
        MotionStatus::BUTTON = m_CM730->m_BulkReadData[CM730::ID_CM].ReadByte(CM730::P_BUTTON);

    m_IsRunning = false;
}


void MotionManager::SetEnable(bool enable) {
    m_Enabled = enable;
    if (m_Enabled)
        m_CM730->WriteWord(CM730::ID_BROADCAST, MX28::P_MOVING_SPEED_L, 0, 0);
}


void MotionManager::AddModule(MotionModule* module) {
    module->Initialize();
    m_Modules.push_back(module);
}


void MotionManager::RemoveModule(MotionModule* module) {
    m_Modules.remove(module);
}


void MotionManager::SetJointDisable(int index) {
    if (!m_Modules.empty()) {
        for (auto& module : m_Modules)
            module->m_Joint.SetEnable(index, false);
    }
}

bool MotionManager::IsDebugEnabled() const {
    return m_debug;
}

void MotionManager::EnabledDebug(bool debug) {
    m_debug = debug;
}

void MotionManager::SetJointOffset(int id, int offset) {
    if(id >= 1 && id < JointData::NUMBER_OF_JOINTS) {
        m_Offset[id] = offset;
    } else {
        throw std::runtime_error("Can't set joint offset. Wrong joint ID: " + std::to_string(id));
    }
}

int MotionManager::GetJointOffset(int id) const {
    if(id >= 1 && id < JointData::NUMBER_OF_JOINTS) {
        return m_Offset[id];
    } else {
        throw std::runtime_error("Can't get joint offset. Wrong joint ID: " + std::to_string(id));
    }
}
