/// \autor arssivka
/// \date 11/17/17

#pragma once


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

    private:
        field_map_t();

    private:
        struct config_t {
            float field_width, field_height;
            float gate_height;
            float penalty_width, penalty_height;

            config_t()
                    : field_width(DEFAULT_FIELD_WIDTH),
                      field_height(DEFAULT_FIELD_HEIGHT),
                      gate_height(DEFAULT_GATE_HEIGHT),
                      penalty_width(DEFAULT_PENALTY_WIDTH),
                      penalty_height(DEFAULT_PENALTY_HEIGHT) {

            }
        } m_config;

        bool m_debug;

        std::map<line_type_t, line_t> m_field_lines;

        void make_lines(float fw, float fh, float pw, float ph, float gh);
    };
}



