//
// Created by nikitas on 26.03.16.
//

#include "vision/vision_t.h"
#include <boost/exception/all.hpp>

using namespace drwn;


vision_t* vision_t::get_instance() {
    static vision_t instance;
    return &instance;
}

cv::Rect vision_t::detect_ball() {
    return m_processor->detect_ball();
}

std::vector<cv::Vec3d> vision_t::detect_angles() {
    return m_processor->detect_angles();
}

cv::Mat vision_t::detect_field() {
    return m_processor->detect_field();
}

void vision_t::set_frame(cv::Mat frame) {
    m_processor->set_frame(frame);
}

vision_processor_t* vision_t::get_processor() const noexcept {
    return m_processor;
}

void vision_t::set_processor(vision_processor_t* processor) noexcept {
    m_processor = processor;
}
