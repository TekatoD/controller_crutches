/*
 *   MotionManager.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include <stdio.h>
#include <math.h>
#include <log/trivial_logger_t.h>
#include "hw/FSR_t.h"
#include "hw/MX28_t.h"
#include "motion/motion_manager_t.h"

using namespace drwn;

motion_manager_t::motion_manager_t() {
    for (int i = 0; i < joint_data_t::NUMBER_OF_JOINTS; i++)
        Offset[i] = 0;
}

bool motion_manager_t::initialize(CM730_t* cm730) {
    int value, error;

    m_CM730 = cm730;
    m_enabled = false;
    m_process_enable = true;

    if (!m_CM730->connect()) {
        if (m_debug) LOG_ERROR << "Fail to connect CM-730";
        return false;
    }

    for (int id = joint_data_t::ID_R_SHOULDER_PITCH; id < joint_data_t::NUMBER_OF_JOINTS; id++) {
        if (m_debug) LOG_DEBUG << "MOTION_MANAGER: ID: " << id << " initializing...";

        if (m_CM730->read_word(id, MX28_t::P_PRESENT_POSITION_L, &value, &error) == CM730_t::SUCCESS) {
            motion_status_t::m_current_joints.set_value(id, value);
            motion_status_t::m_current_joints.set_enable(id, true);

            if (m_debug) LOG_DEBUG << "MOTION MANAGER: [" << value << "] Success";
        } else {
            motion_status_t::m_current_joints.set_enable(id, false);

            if (m_debug) LOG_ERROR << "MOTION MANAGER: Fail";
        }
    }

    m_calibration_status = 0;
    m_fb_gyro_center = 512;
    m_rl_gyro_center = 512;

    return true;
}


bool motion_manager_t::reinitialize() {
    m_process_enable = false;

    m_CM730->DXL_power_on();

    int value, error;
    for (int id = joint_data_t::ID_R_SHOULDER_PITCH; id < joint_data_t::NUMBER_OF_JOINTS; id++) {
        if (m_debug) LOG_DEBUG << "MOTION MANAGER: ID: " << id << " initializing...";

        if (m_CM730->read_word(id, MX28_t::P_PRESENT_POSITION_L, &value, &error) == CM730_t::SUCCESS) {
            motion_status_t::m_current_joints.set_value(id, value);
            motion_status_t::m_current_joints.set_enable(id, true);

            if (m_debug) LOG_DEBUG << "MOTION MANAGERL: [" << value << "] Success";
        } else {
            motion_status_t::m_current_joints.set_enable(id, false);

            if (m_debug) LOG_ERROR << "MOTION MANAGER: Fail";
        }
    }

    m_process_enable = true;
    return true;
}

void motion_manager_t::process() {
    constexpr int GYRO_WINDOW_SIZE = 100;
    constexpr int ACCEL_WINDOW_SIZE = 30;
    constexpr float MARGIN_OF_SD = 2.0;

    if (!m_process_enable || m_is_running)
        return;

    m_is_running = true;

    // calibrate gyro sensor
    if (m_calibration_status == 0 || m_calibration_status == -1) {
        static int fb_gyro_array[GYRO_WINDOW_SIZE] = {512,};
        static int rl_gyro_array[GYRO_WINDOW_SIZE] = {512,};
        static int buf_idx = 0;

        if (buf_idx < GYRO_WINDOW_SIZE) {
            if (m_CM730->m_bulk_read_data[CM730_t::ID_CM].error == 0) {
                fb_gyro_array[buf_idx] = m_CM730->m_bulk_read_data[CM730_t::ID_CM].read_word(CM730_t::P_GYRO_Y_L);
                rl_gyro_array[buf_idx] = m_CM730->m_bulk_read_data[CM730_t::ID_CM].read_word(CM730_t::P_GYRO_X_L);
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
                m_fb_gyro_center = (int) fb_mean;
                m_rl_gyro_center = (int) rl_mean;
                m_calibration_status = 1;
                if (m_debug) {
                    LOG_DEBUG << "MOTION MANAGER: FBGyroCenter: " << m_fb_gyro_center
                              << ", RLGyroCenter: " << m_rl_gyro_center;
                }
            } else {
                m_fb_gyro_center = 512;
                m_rl_gyro_center = 512;
                m_calibration_status = -1;
            }
        }
    }

    if (m_calibration_status == 1 && m_enabled) {
        static int fb_array[ACCEL_WINDOW_SIZE] = {512,};
        static int buf_idx = 0;
        if (m_CM730->m_bulk_read_data[CM730_t::ID_CM].error == 0) {
            motion_status_t::FB_GYRO = m_CM730->m_bulk_read_data[CM730_t::ID_CM].read_word(CM730_t::P_GYRO_Y_L) - m_fb_gyro_center;
            motion_status_t::RL_GYRO = m_CM730->m_bulk_read_data[CM730_t::ID_CM].read_word(CM730_t::P_GYRO_X_L) - m_rl_gyro_center;
            motion_status_t::RL_ACCEL = m_CM730->m_bulk_read_data[CM730_t::ID_CM].read_word(CM730_t::P_ACCEL_X_L);
            motion_status_t::FB_ACCEL = m_CM730->m_bulk_read_data[CM730_t::ID_CM].read_word(CM730_t::P_ACCEL_Y_L);
            fb_array[buf_idx] = motion_status_t::FB_ACCEL;
            if (++buf_idx >= ACCEL_WINDOW_SIZE) buf_idx = 0;
        }

        int sum = 0, avr = 512;
        for (int idx = 0; idx < ACCEL_WINDOW_SIZE; idx++)
            sum += fb_array[idx];
        avr = sum / ACCEL_WINDOW_SIZE;

        if (avr < motion_status_t::FALLEN_F_LIMIT)
            motion_status_t::FALLEN = FORWARD;
        else if (avr > motion_status_t::FALLEN_B_LIMIT)
            motion_status_t::FALLEN = BACKWARD;
        else
            motion_status_t::FALLEN = STANDUP;

        if (!m_modules.empty()) {
            for (auto& module : m_modules) {
                module->process();
                for (int id = joint_data_t::ID_R_SHOULDER_PITCH; id < joint_data_t::NUMBER_OF_JOINTS; id++) {
                    if (module->joint.get_enable(id)) {
                        motion_status_t::m_current_joints.set_value(id, module->joint.get_value(id));

                        motion_status_t::m_current_joints.set_p_gain(id, module->joint.get_p_gain(id));
                        motion_status_t::m_current_joints.set_i_gain(id, module->joint.get_i_gain(id));
                        motion_status_t::m_current_joints.set_d_gain(id, module->joint.get_d_gain(id));
                    }
                }
            }
        }

        int param[joint_data_t::NUMBER_OF_JOINTS * MX28_t::PARAM_BYTES];
        int n = 0;
        int joint_num = 0;
        for (int id = joint_data_t::ID_R_SHOULDER_PITCH; id < joint_data_t::NUMBER_OF_JOINTS; id++) {
            if (motion_status_t::m_current_joints.get_enable(id)) {
                param[n++] = id;
                param[n++] = motion_status_t::m_current_joints.get_d_gain(id);
                param[n++] = motion_status_t::m_current_joints.get_i_gain(id);
                param[n++] = motion_status_t::m_current_joints.get_p_gain(id);
                param[n++] = 0;
                param[n++] = CM730_t::get_low_byte(motion_status_t::m_current_joints.get_value(id) + Offset[id]);
                param[n++] = CM730_t::get_high_byte(motion_status_t::m_current_joints.get_value(id) + Offset[id]);
                joint_num++;
            }

            if (m_debug)
                LOG_DEBUG << "MOTION MANAGER: ID[" << id << "] : " << motion_status_t::m_current_joints.get_value(id);
        }

        if (joint_num > 0)
            m_CM730->sync_write(MX28_t::P_D_GAIN, MX28_t::PARAM_BYTES, joint_num, param);
    }

    m_CM730->bulk_read();

    if (m_CM730->m_bulk_read_data[CM730_t::ID_CM].error == 0)
        motion_status_t::BUTTON = m_CM730->m_bulk_read_data[CM730_t::ID_CM].read_byte(CM730_t::P_BUTTON);

    m_is_running = false;
}


void motion_manager_t::set_enable(bool enable) {
    m_enabled = enable;
    if (m_enabled)
        m_CM730->write_word(CM730_t::ID_BROADCAST, MX28_t::P_MOVING_SPEED_L, 0, 0);
}


void motion_manager_t::add_module(motion_module_t* module) {
    module->initialize();
    m_modules.push_back(module);
}


void motion_manager_t::remove_module(motion_module_t* module) {
    m_modules.remove(module);
}


void motion_manager_t::set_joint_disable(int index) {
    if (!m_modules.empty()) {
        for (auto& module : m_modules)
            module->joint.set_enable(index, false);
    }
}

bool motion_manager_t::is_debug_enabled() const {
    return m_debug;
}

void motion_manager_t::enable_debug(bool debug) {
    m_debug = debug;
}

void motion_manager_t::set_joint_offset(int id, int offset) {
    LOG_DEBUG << "MOTION MANAGER ID: " << id << " offset = " << offset;
    if(id >= 1 && id < joint_data_t::NUMBER_OF_JOINTS) {
        Offset[id] = offset;
    } else {
        throw std::runtime_error("Can't set joint offset. Wrong joint ID: " + std::to_string(id));
    }
}

int motion_manager_t::get_joint_offset(int id) const {
    if(id >= 1 && id < joint_data_t::NUMBER_OF_JOINTS) {
        return Offset[id];
    } else {
        throw std::runtime_error("Can't get joint offset. Wrong joint ID: " + std::to_string(id));
    }
}
