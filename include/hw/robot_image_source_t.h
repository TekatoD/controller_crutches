/// \autor arssivka
/// \date 11/7/17

#pragma once


#include <cv.hpp>
#include "image_source_t.h"
#include "camera_t.h"

namespace drwn {
    class robot_image_source_t
            : public image_source_t {
    public:
        robot_image_source_t(double width, double height, int camera_id = 0);

        void set_gain(float gain);

        float get_gain() const;

        void set_brightness(float brightness);

        float get_brightness() const;

        void set_contrast(float contrast);

        void set_hue(float hue);

        float get_hue() const;

        void set_width(float width);

        float get_width() const;

        void set_height(float height);

        float get_height() const;

        cv::Mat capture_frame() const override;

        bool is_debug_enabled() const noexcept;

        void enable_debug(bool debug) noexcept;

    private:
        bool m_debug{false};
        cv::VideoCapture m_capture{0};
    };
}



