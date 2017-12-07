/**
 *  @autor arssivka
 *  @date 12/6/17
 */

#pragma once


#include <cv.hpp>

namespace drwn {
    class ball_filter {
    public:
        void set_ball(const cv::Rect& ball);

        cv::Rect get_ball() const noexcept;

        void reset();

    private:
        bool m_found{false};
        int m_counter{0};
        int m_counter_limit{2};
        cv::Rect m_curr_ball_position{};
    };
}


