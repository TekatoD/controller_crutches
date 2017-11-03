/// \autor arssivka
/// \date 11/3/17

#include <cv.hpp>
#include <log/trivial_logger_t.h>
#include "vision/white_ball_vision_processor_t.h"

const cv::Rect& drwn::white_ball_vision_processor_t::detect_ball() {
    this->process();
    return m_ball;
}

const std::vector<cv::Vec4i>& drwn::white_ball_vision_processor_t::detect_lines() {
    this->process();
    return m_lines;
}

const std::vector<cv::Vec3d>& drwn::white_ball_vision_processor_t::detect_angles() {
    throw std::runtime_error("detect angles isn't implemented");
}

const cv::Mat& drwn::white_ball_vision_processor_t::detect_field() {
    this->process();
    return m_field_mask;
}

void drwn::white_ball_vision_processor_t::set_frame(cv::Mat frame) {
    this->reset();
    m_img = frame;
    if (m_debug) LOG_DEBUG << "WHITE BALL VISION PROCESSOR: New frame has been set";
}

void drwn::white_ball_vision_processor_t::reset() {
    m_img_processed = false;
    m_lines.clear();
    m_img = cv::Mat();
    m_field_mask = cv::Mat();
}

bool drwn::white_ball_vision_processor_t::is_debug_enabled() const noexcept {
    return m_debug;
}

void drwn::white_ball_vision_processor_t::enable_debug(bool debug) noexcept {
    m_debug = debug;
}

void drwn::white_ball_vision_processor_t::process() {
    if (!m_img_processed) {
        if (m_debug) LOG_DEBUG << "WHITE BALL VISION PROCESSOR: Processing field";
        cv::Mat field_prep = m_field_preproc.preprocess(m_img);
        m_field_mask = m_field_detector.detect(field_prep);

        // we can use prep as mask
        cv::Mat tmp;
        m_img.copyTo(tmp, m_field_mask);
        m_img = tmp;

        if (m_debug) LOG_DEBUG << "WHITE BALL VISION PROCESSOR: Detecting lines";
        m_ball_detector.detect(m_img, m_lines);
        cv::Mat line_preproc = m_line_preproc.preprocess(m_img);
        if (m_debug) LOG_DEBUG << "WHITE BALL VISION PROCESSOR: Detecting white ball";
        m_lines = m_line_detector.detect(line_preproc);
        m_img_processed = true;
        if (m_debug) LOG_DEBUG << "WHITE BALL VISION PROCESSOR: Processing has been finished";
    }
}
