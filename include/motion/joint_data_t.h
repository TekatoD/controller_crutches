/*
 *   JointData.h
 *   This class represents the state of all articulations (all the motors MX28)
 *   Author: ROBOTIS
 *
 */

#pragma once


namespace drwn {
    class joint_data_t {
    public:
        enum {
            ID_R_SHOULDER_PITCH = 1,
            ID_L_SHOULDER_PITCH = 2,
            ID_R_SHOULDER_ROLL = 3,
            ID_L_SHOULDER_ROLL = 4,
            ID_R_ELBOW = 5,
            ID_L_ELBOW = 6,
            ID_R_HIP_YAW = 7,
            ID_L_HIP_YAW = 8,
            ID_R_HIP_ROLL = 9,
            ID_L_HIP_ROLL = 10,
            ID_R_HIP_PITCH = 11,
            ID_L_HIP_PITCH = 12,
            ID_R_KNEE = 13,
            ID_L_KNEE = 14,
            ID_R_ANKLE_PITCH = 15,
            ID_L_ANKLE_PITCH = 16,
            ID_R_ANKLE_ROLL = 17,
            ID_L_ANKLE_ROLL = 18,
            ID_HEAD_PAN = 19,
            ID_HEAD_TILT = 20,
            NUMBER_OF_JOINTS
        };

        enum {
            P_GAIN_DEFAULT = 32,
            I_GAIN_DEFAULT = 0,
            D_GAIN_DEFAULT = 0
        };

    private:

    protected:
        /*the values*/
        bool m_enable[NUMBER_OF_JOINTS];
        int m_value[NUMBER_OF_JOINTS];
        float m_angle[NUMBER_OF_JOINTS];
        int m_p_gain[NUMBER_OF_JOINTS];
        int m_i_gain[NUMBER_OF_JOINTS];
        int m_d_gain[NUMBER_OF_JOINTS];

    public:
        joint_data_t();

        ~joint_data_t();

        /*accessors*/
        void set_enable(int id, bool enable);

        void set_enable(int id, bool enable, bool exclusive);

        void set_enable_head_only(bool enable);

        void set_enable_head_only(bool enable, bool exclusive);

        void set_enable_right_arm_only(bool enable);

        void set_enable_right_arm_only(bool enable, bool exclusive);

        void set_enable_left_arm_only(bool enable);

        void set_enable_left_arm_only(bool enable, bool exclusive);

        void set_enable_right_leg_only(bool enable);

        void set_enable_right_leg_only(bool enable, bool exclusive);

        void set_enable_left_leg_only(bool enable);

        void set_enable_left_leg_only(bool enable, bool exclusive);

        void set_enable_upper_body_without_head(bool enable);

        void set_enable_upper_body_without_head(bool enable, bool exclusive);

        void set_enable_lower_body(bool enable);

        void set_enable_lower_body(bool enable, bool exclusive);

        void set_enable_body_without_head(bool enable);

        void set_enable_body_without_head(bool enable, bool exclusive);

        void set_enable_body(bool enable);

        void set_enable_body(bool enable, bool exclusive);

        bool get_enable(int id);

        void set_value(int id, int value);

        int get_value(int id);

        /*setter/getter angle of articulation id (in degrees)*/
        void set_angle(int id, float angle);

        float get_angle(int id);

        void set_radian(int id, float radian);

        float get_radian(int id);

        void set_p_gain(int id, int pgain) { m_p_gain[id] = pgain; }

        int get_p_gain(int id) { return m_p_gain[id]; }

        void set_i_gain(int id, int igain) { m_i_gain[id] = igain; }

        int get_i_gain(int id) { return m_i_gain[id]; }

        void set_d_gain(int id, int dgain) { m_d_gain[id] = dgain; }

        int get_d_gain(int id) { return m_d_gain[id]; }
    };
}

