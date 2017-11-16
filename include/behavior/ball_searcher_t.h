/**
 *  @autor arssivka
 *  @date 5/11/17
 */

#pragma once


#include <vector>
#include <math/point_t.h>

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

    private:
        ball_searcher_t();

    private:
        bool m_debug{false};
        bool m_active;
        point2d_t m_last_position;

        float m_tilt_phase;
        float m_pan_phase;
        float m_tilt_phase_step;
        float m_pan_phase_step;
        float m_phase_size;

        float m_turn_speed;
        float m_turn_step;
        float m_max_turn;

        int m_turn_direction;
        int m_pan_direction;
        bool m_walking_enabled;
    };
}

