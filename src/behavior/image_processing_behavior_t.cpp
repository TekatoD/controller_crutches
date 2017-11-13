/// \autor arssivka
/// \date 11/13/17

#include "behavior/image_processing_behavior_t.h"
#include <hw/camera_t.h>
#include <vision/vision_t.h>
#include <log/trivial_logger_t.h>
#include <hw/image_source_failure.h>

using namespace drwn;

image_processing_behavior_t::image_processing_behavior_t()
        : m_camera(camera_t::get_instance()),
          m_vision(vision_t::get_instance()) {
    // Do nothing
}

void image_processing_behavior_t::process() {
    LOG_DEBUG << "IMAGE PROCESSING BEHAVIOR: Processing...";
    try {
        m_camera->update_image();
        cv::Mat frame = m_camera->get_image();
        m_vision->set_frame(frame);
        m_vision->process();
    } catch(const image_source_failure& e) {
        LOG_WARNING << "IMAGE PROCESSING BEHAVIOR: CV loop iteration failed: " << e.what();
    }
}
