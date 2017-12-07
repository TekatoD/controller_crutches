/**
 *  @autor arssivka
 *  @date 12/6/17
 */

#include "behavior/ball_filter.h"

using namespace drwn;

void ball_filter::set_ball(const cv::Rect& ball) {
    if (ball == cv::Rect()) {
        m_counter = std::max(m_counter - 1, 0);
        if (m_counter == 0) m_found = false;
    } else {
        m_curr_ball_position = ball;
        m_counter = std::min(m_counter + 1, m_counter_limit);
        m_found = true;
    }
}

cv::Rect ball_filter::get_ball() const noexcept {
    if (!m_found) {
        return cv::Rect();
    } else {
        return m_curr_ball_position;
    }
}

void ball_filter::reset() {
    m_found = false;
    m_counter = 0;
}
