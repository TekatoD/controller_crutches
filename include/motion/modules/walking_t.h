/*
 *   Walking.h
 *
 *   Author: ROBOTIS
 *
 */

#pragma once

#include <string.h>
#include <memory>
#include <fstream>

#include "motion/motion_module_t.h"
#include "OdometryCollector.h"

namespace drwn {
    class walking_t
            : public motion_module_t {
    public:
        enum {
            PHASE0 = 0,
            PHASE1 = 1,
            PHASE2 = 2,
            PHASE3 = 3
        };

    private:
        bool m_debug{false};

        float m_cur_period_time;
        float m_cur_dsp_ratio;
        float m_cur_ssp_ratio;
        float m_cur_x_swap_period_time;
        float m_cur_x_move_period_time;
        float m_cur_y_swap_period_time;
        float m_cur_y_move_period_time;
        float m_cur_z_swap_period_time;
        float m_cur_z_move_period_time;
        float m_cur_a_move_period_time;
        float m_cur_ssp_time;
        float m_cur_ssp_time_start_l;
        float m_cur_ssp_time_end_l;
        float m_cur_ssp_time_start_r;
        float m_cur_ssp_time_end_r;
        float m_cur_phase_time1;
        float m_cur_phase_time2;
        float m_cur_phase_time3;

        float m_cur_x_offset;
        float m_cur_y_offset;
        float m_cur_z_offset;
        float m_cur_r_offset;
        float m_cur_p_offset;
        float m_cur_a_offset;

        float m_cur_x_swap_phase_shift;
        float m_cur_x_swap_amplitude;
        float m_cur_x_swap_amplitude_shift;
        float m_cur_x_move_phase_shift;
        float m_cur_x_move_amplitude;
        float m_cur_x_move_amplitude_shift;
        float m_cur_y_swap_phase_shift;
        float m_cur_y_swap_amplitude;
        float m_cur_y_swap_amplitude_shift;
        float m_cur_y_move_phase_shift;
        float m_cur_y_move_amplitude;
        float m_cur_y_move_amplitude_shift;
        float m_cur_z_swap_phase_shift;
        float m_cur_z_swap_amplitude;
        float m_cur_z_swap_amplitude_shift;
        float m_cur_z_move_phase_shift;
        float m_cur_z_move_amplitude;
        float m_cur_z_move_amplitude_shift;
        float m_cur_a_move_phase_shift;
        float m_cur_a_move_amplitude;
        float m_cur_a_move_amplitude_shift;

        float m_cur_pelvis_offset;
        float m_cur_pelvis_swing;
        float m_cur_hip_pitch_offset;
        float m_cur_arm_swing_gain;

        bool m_ctrl_running;
        bool m_real_running;
        float m_time;

        int m_phase;
        float m_cur_body_swing_y;
        float m_cur_body_swing_z;

        float m_odo_x;
        float m_odo_y;
        float m_odo_theta;

        float m_left_odo_x;
        float m_left_odo_y;
        float m_left_odo_theta;


        float m_right_odo_x;
        float m_right_odo_y;
        float m_right_odo_theta;

        bool m_left_end;
        bool m_right_end;
        bool m_left_start;
        bool m_right_start;

        float m_odo_x_factor;
        float m_odo_y_factor;
        float m_odo_a_factor;

        OdometryCollector m_odometry_collector;

        walking_t();

        void update_param_time();

        void update_param_move();

        void update_param_balance();

        Pose2D get_odo_offset();

        // Walking initial pose
        float m_x_offset;
        float m_y_offset;
        float m_z_offset;
        float m_a_offset;
        float m_p_offset;
        float m_r_offset;

        // Walking control
        float m_period_time;
        float m_dsp_ratio;
        float m_step_fb_ratio;
        float m_x_move_amplitude;
        float m_y_move_amplitude;
        float m_z_move_amplitude;
        float m_a_move_amplitude;
        bool m_move_aim_on;

        // Balance control
        bool m_balance_enable;
        float m_balance_knee_gain;
        float m_balance_ankle_pitch_gain;
        float m_balance_hip_roll_gain;
        float m_balance_ankle_roll_gain;
        float m_y_swap_amplitude;
        float m_z_swap_amplitude;
        float m_arm_swing_gain;
        float m_pelvis_offset;
        float m_hip_pitch_offset;

        int m_p_gain;
        int m_i_gain;
        int m_d_gain;

    public:
        static walking_t* GetInstance();

        void initialize() override;

        void start();

        void stop();

        void process() override;

        bool is_running();

        Pose2D get_odo();

        void reset_odo(const Pose2D& pose);

        void set_odo(const Pose2D& pose);

        float get_x_offset() const;

        void set_x_offset(float x_offset);

        float get_y_offset() const;

        void set_y_offset(float y_offset);

        float get_z_offset() const;

        void set_z_offset(float z_offset);

        float get_yaw_offset() const;

        void set_yaw_offset(float a_offset);

        float get_pitch_offset() const;

        void set_pitch_offset(float p_offset);

        float get_roll_offset() const;

        void set_roll_offset(float r_offset);

        float get_period_time() const;

        void set_period_time(float period_time);

        float get_DSP_ratio() const;

        void set_DSP_ratio(float dsp_ratio);

        float get_step_FB_ratio() const;

        void set_step_FB_ratio(float step_fb_ratio);

        float get_x_move_amplitude() const;

        void set_x_move_amplitude(float x_move_amplitude);

        float get_y_move_amplitude() const;

        void set_y_move_amplitude(float y_move_amplitude);

        float get_z_move_amplitude() const;

        void set_z_move_amplitude(float z_move_amplitude);

        float get_a_move_amplitude() const;

        void set_a_move_amplitude(float a_move_amplitude);

        bool get_a_move_aim_on() const;

        void set_move_aim_on(bool move_aim_on);

        bool get_balance_enable() const;

        void set_balance_enable(bool balance_enable);

        float get_balance_knee_gain() const;

        void set_balance_knee_gain(float balance_knee_gain);

        float get_balance_ankle_pitch_gain() const;

        void set_balance_ankle_pitch_gain(float balance_ankle_pitch_gain);

        float get_balance_hip_roll_gain() const;

        void set_balance_hip_roll_gain(float balance_hip_roll_gain);

        float get_balance_ankle_roll_gain() const;

        void set_balance_ankle_roll_gain(float balance_ankle_roll_gain);

        float get_y_swap_amplitude() const;

        void set_y_swap_amplitude(float y_swap_amplitude);

        float get_z_swap_amplitude() const;

        void set_z_swap_amplitude(float z_swap_amplitude);

        float get_arm_swing_gain() const;

        void set_arm_swing_gain(float arm_swing_gain);

        float get_pelvis_offset() const;

        void set_pelvis_offset(float pelvis_offset);

        float get_hip_pitch_offset() const;

        void set_hip_pitch_offset(float hip_pitch_offset);

        int get_p_gain() const;

        void set_p_gain(int p_gain);

        int get_i_gain() const;

        void set_i_gain(int i_gain);

        int get_d_gain() const;

        void set_d_gain(int d_gain);

        float get_odo_x_factor() const;

        void set_odo_x_factor(float odo_x_factor);

        float get_odo_y_factor() const;

        void set_odo_y_factor(float odo_y_factor);

        float get_odo_a_factor() const;

        void set_odo_a_factor(float odo_a_factor);

        bool is_debug_enabled() const;

        void enable_debug(bool debug);
    };
}

