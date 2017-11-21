#pragma once

#include <cstring>
#include <memory>

#include "motion/motion_module_t.h"
#include "math/point_t.h"

namespace drwn {
    class head_t
            : public motion_module_t {
    public:
        static head_t* get_instance() {
            static head_t head;
            return &head;
        }

        ~head_t();

        void initialize();

        void process();

        float get_top_limit_angle() { return m_top_limit; }

        float get_bottom_limit_angle() { return m_bottom_limit; }

        float get_right_limit_angle() { return m_right_limit; }

        float get_left_limit_angle() { return m_left_limit; }

        float get_pan_angle() { return m_pan_angle; }

        float get_tilt_angle() { return m_tilt_angle; }

        void move_to_home();

        void move_by_angle(float pan, float tilt);

        void move_by_angle_offset(float pan, float tilt);

        void init_tracking();

        void move_tracking(point2d_t err); // For image processing
        void move_tracking();

        float get_left_limit() const;

        void set_left_limit(float left_limit);

        float get_right_limit() const;

        void set_right_limit(float right_limit);

        float get_top_limit() const;

        void set_top_limit(float top_limit);

        float get_bottom_limit() const;

        void set_bottom_limit(float bottom_limit);

        float get_pan_home() const;

        void set_pan_home(float pan_home);

        float get_tilt_home() const;

        void set_tilt_home(float tilt_home);

        float get_pan_p_gain() const;

        void set_pan_p_gain(float pan_p_gain);

        float get_pan_d_gain() const;

        void set_pan_d_gain(float pan_d_gain);

        float get_tilt_p_gain() const;

        void set_tilt_p_gain(float tilt_p_gain);

        float get_tilt_d_gain() const;

        void set_tilt_d_gain(float tilt_d_gain);

        bool is_debug_enabled() const;

        void enable_debug(bool debug);

    private:
        float m_left_limit;
        float m_right_limit;
        float m_top_limit;
        float m_bottom_limit;
        float m_pan_home;
        float m_tilt_home;
        float m_pan_err;
        float m_pan_err_diff;
        float m_pan_p_gain;
        float m_pan_d_gain;
        float m_tilt_err;
        float m_tilt_err_diff;
        float m_tilt_p_gain;
        float m_tilt_d_gain;
        float m_pan_angle;
        float m_tilt_angle;
        bool m_debug{false};

        head_t();

        void check_limit();

    };
}