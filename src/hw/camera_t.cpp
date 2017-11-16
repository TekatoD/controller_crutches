/// \autor arssivka
/// \date 11/3/17

#include <log/trivial_logger_t.h>
#include "hw/camera_t.h"

using namespace drwn;

camera_t* camera_t::get_instance() {
    static camera_t instance;
    return &instance;
}

void camera_t::update_image() {
    if (m_debug) LOG_TRACE << "CAMERA: Updating image...";
    m_img = m_img_source->capture_frame();
    if (m_debug) LOG_DEBUG << "CAMERA: Image has been updated";
}

const cv::Mat& camera_t::get_image() const noexcept {
    return m_img;
}

image_source_t* camera_t::get_image_source() const noexcept {
    return m_img_source;
}

void camera_t::set_image_source(image_source_t* img_source) noexcept {
    m_img_source = img_source;
}

bool camera_t::is_debug_enabled() const noexcept {
    return m_debug;
}

void camera_t::enable_debug(bool debug) noexcept {
    m_debug = debug;
}
