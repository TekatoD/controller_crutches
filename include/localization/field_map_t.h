/// \autor arssivka
/// \date 11/17/17

#pragma once


#include <math/point_t.h>
#include <localization/line_t.h>
#include <motion/pose2d_t.h>

namespace drwn {
    class field_map_t {
    public:
        enum class line_type_t {
            CENTRAL_LINE = 0,
            FIELD_LEFT = 1,
            FIELD_RIGHT = 2,
            FIELD_TOP = 3,
            FIELD_BOTTOM = 4,
            PENALTY_LEFT_TOP = 5,
            PENALTY_LEFT_BOTTOM = 6,
            PENALTY_LEFT_HEIGHT = 7,
            PENALTY_RIGHT_TOP = 8,
            PENALTY_RIGHT_BOTTOM = 9,
            PENALTY_RIGHT_HEIGHT = 10,
            NONE = 11
        };

        static constexpr float MAX_DIST = 12000.0f;
        static constexpr float DEFAULT_FIELD_WIDTH = 6000.0f;
        static constexpr float DEFAULT_FIELD_HEIGHT = 4000.0f;
        static constexpr float DEFAULT_GATE_HEIGHT = 1400.0f;
        static constexpr float DEFAULT_PENALTY_WIDTH = 600.0f;
        static constexpr float DEFAULT_PENALTY_HEIGHT = 2200.0f;

        static field_map_t* get_instance();

        ~field_map_t() = default;

        void initialize_field();

        void log_field_lines() const;

        std::tuple<line_type_t, point2d_t> intersect_with_field(const line_t& l, float min_dist = 0.0f);

        void set_field_width(float field_width);

        float get_field_width() const;

        void set_field_height(float field_height);

        float get_field_height() const;

        void set_gate_height(float gate_height);

        float get_gate_height() const;

        void set_penalty_width(float penalty_width);

        float get_penalty_width() const;

        void set_penalty_height(float penalty_height);

        float get_penalty_height() const;

        bool is_debug_enabled() const;

        void enable_debug(bool debug);

        const pose2d_t& get_spawn_pose() const;

        void set_spawn_pose(const pose2d_t& spawn_pose);

        const pose2d_t& get_start_pose() const;

        void set_start_pose(const pose2d_t& staring_pose);

    private:
        field_map_t();

    private:
        struct config_t {
            float field_width{DEFAULT_FIELD_WIDTH};
            float field_height{DEFAULT_FIELD_HEIGHT};
            float gate_height{DEFAULT_GATE_HEIGHT};
            float penalty_width{DEFAULT_PENALTY_WIDTH};
            float penalty_height{DEFAULT_PENALTY_HEIGHT};
        } m_config;

        pose2d_t m_spawn_pose;
        pose2d_t m_start_pose;

        bool m_debug {false};

        std::map<line_type_t, line_t> m_field_lines;

        void make_lines(float fw, float fh, float pw, float ph, float gh);
    };
}



