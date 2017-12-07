/**
 *  @autor arssivka
 *  @date 5/11/17
 */

#pragma once


#include <vector>
#include <math/point_t.h>
#include <hw/camera_t.h>

namespace drwn {
    class ball_searcher_t {
    public:
        static ball_searcher_t* get_instance();

        void set_last_position(const point2d_t& pos);

        void process();

        void enable_walking(bool enabled);

        bool is_walking_enabled() const;

        const point2d_t& get_last_position() const;

        float get_tilt_phase_step() const;

        void set_tilt_phase_step(float tilt_phase_step);

        float get_pan_phase_step() const;

        void set_pan_phase_step(float pan_phase_step);

        float get_phase_size() const;

        void set_phase_size(float phase_size);

        float get_turn_step() const;

        void set_turn_step(float turn_step);

        float get_max_turn() const;

        void set_max_turn(float max_turn);

        void enable_debug(bool debug);

        bool is_debug_enabled() const;

        bool is_pan_enabled() const;

        void enable_pan(bool pan_active);

    private:
        ball_searcher_t() = default;

    private:
        bool m_debug{false};
        bool m_active{false};
        point2d_t m_last_position{camera_t::WIDTH / 2, camera_t::HEIGHT / 2};

        float m_tilt_phase{0.0f};
        float m_pan_phase{0.0f};
        float m_tilt_phase_step{1.0f};
        float m_pan_phase_step{2.0f};
        float m_phase_size{500.0f};

        float m_turn_speed{0.0f};
        float m_turn_step{1.0f};
        float m_max_turn{20.0f};

        int m_turn_direction{0};
        int m_pan_direction{0};
        bool m_walking_enabled{true};
        bool m_pan_enabled{true};

    };
}

