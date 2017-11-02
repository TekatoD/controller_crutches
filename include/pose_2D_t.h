/**
 *  @autor tekatod
 *  @date 4/28/17
 */
#pragma once


#include <ostream>

namespace drwn {
    class pose_2D_t {
    public:

        pose_2D_t();

        pose_2D_t(float x, float y, float theta = 0);

        float x() const;

        float y() const;

        float theta() const;

        void set_x(float x);

        void set_y(float y);

        void set_theta(float theta);

        void normalize_theta();

        void rotate_around(const pose_2D_t& pose);

        pose_2D_t operator+(const pose_2D_t& rhs) const;

        pose_2D_t& operator+=(const pose_2D_t& rhs);

        pose_2D_t operator-(const pose_2D_t& rhs) const;

        pose_2D_t& operator-=(const pose_2D_t& rhs);

        friend std::ostream& operator << (std::ostream& os, const pose_2D_t& data);

    private:
        float m_x;
        float m_y;
        float m_theta;
    };
}

