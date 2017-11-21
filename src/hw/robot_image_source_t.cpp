/// \autor arssivka
/// \date 11/7/17

#include <log/trivial_logger_t.h>
#include "hw/robot_image_source_t.h"

using namespace drwn;

robot_image_source_t::robot_image_source_t(double width, double height, int camera_id)
        : m_capture(camera_id) {
    m_capture.set(cv::CAP_PROP_FRAME_WIDTH, width);
    m_capture.set(cv::CAP_PROP_FRAME_HEIGHT, height);
}

void robot_image_source_t::set_gain(float gain) {
    if(m_debug) {
        LOG_DEBUG << "ROBOT IMAGE SOURCE: gain = " << gain;
    }
    m_capture.set(CV_CAP_PROP_GAIN, gain);
}

float robot_image_source_t::get_gain() const {
    return (float) m_capture.get(CV_CAP_PROP_GAIN);
}

void robot_image_source_t::set_brightness(float brightness) {
    if(m_debug) {
        LOG_DEBUG << "ROBOT IMAGE SOURCE: brightness = " << brightness;
    }
    m_capture.set(CV_CAP_PROP_BRIGHTNESS, brightness);
}

float robot_image_source_t::get_brightness() const {
    return (float) m_capture.get(CV_CAP_PROP_BRIGHTNESS);
}

void robot_image_source_t::set_contrast(float contrast) {
    if(m_debug) {
        LOG_DEBUG << "ROBOT IMAGE SOURCE: contrast = " << contrast;
    }
    m_capture.set(CV_CAP_PROP_CONTRAST, contrast);
}

float robot_image_source_t::get_contrast() const {
    return (float) m_capture.get(CV_CAP_PROP_CONTRAST);
}

void robot_image_source_t::set_hue(float hue) {
    if(m_debug) {
        LOG_DEBUG << "ROBOT IMAGE SOURCE: hue = " << hue;
    }
    m_capture.set(CV_CAP_PROP_HUE, hue);
}

float robot_image_source_t::get_hue() const {
    return (float) m_capture.get(CV_CAP_PROP_HUE);
}

void robot_image_source_t::set_width(float width) {
    if(m_debug) {
        LOG_DEBUG << "ROBOT IMAGE SOURCE: width = " << width;
    }
    m_capture.set(CV_CAP_PROP_FRAME_WIDTH, width);
}

float robot_image_source_t::get_width() const {
    return (float) m_capture.get(CV_CAP_PROP_FRAME_WIDTH);
}

void robot_image_source_t::set_height(float height) {
    if(m_debug) {
        LOG_DEBUG << "ROBOT IMAGE SOURCE: height = " << height;
    }
    m_capture.set(CV_CAP_PROP_FRAME_HEIGHT, height);
}

float robot_image_source_t::get_height() const {
    return (float) m_capture.get(CV_CAP_PROP_FRAME_HEIGHT);
}

cv::Mat robot_image_source_t::capture_frame() {
    cv::Mat frame;
    m_capture >> frame;
    if (m_rotate_frame) {
        if(m_debug) {
            LOG_TRACE << "ROBOT IMAGE SOURCE: Rotating frame";
        }
        cv::rotate(frame, frame, cv::ROTATE_180);
    }

    if(m_debug) {
        LOG_DEBUG << "ROBOT IMAGE SOURCE: Received image";
    }
    return frame;
}

bool robot_image_source_t::is_debug_enabled() const noexcept {
    return m_debug;
}

void robot_image_source_t::enable_debug(bool debug) noexcept {
    m_debug = debug;
}

bool robot_image_source_t::is_rotate_frame_enabled() const {
    return m_rotate_frame;
}

void robot_image_source_t::enable_rotate_frame(bool rotate_frame) {
    if(m_debug) {
        LOG_DEBUG << "ROBOT IMAGE SOURCE: rotate_frame = " << std::boolalpha << rotate_frame;
    }
    m_rotate_frame = rotate_frame;
}
