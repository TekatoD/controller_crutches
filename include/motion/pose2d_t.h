/**
 *  @autor tekatod
 *  @date 4/28/17
 */
#pragma once


#include <ostream>

namespace drwn {
    class pose2d_t {
    public:

        pose2d_t();

        pose2d_t(float x, float y, float theta = 0);

        float get_x() const;

        float get_y() const;

        float get_theta() const;

        void set_x(float x);

        void set_y(float y);

        void set_theta(float theta);

        bool is_nan() const;

        void normalize_theta();

        void rotate_around(const pose2d_t& pose);

        pose2d_t operator+(const pose2d_t& rhs) const;

        pose2d_t& operator+=(const pose2d_t& rhs);

        pose2d_t operator-(const pose2d_t& rhs) const;

        pose2d_t& operator-=(const pose2d_t& rhs);

        friend std::ostream& operator << (std::ostream& os, const pose2d_t& data);

    protected:
        float m_x;
        float m_y;
        float m_theta;
    };
}

