/// \autor arssivka
/// \date 11/3/17

#pragma once


#include "image_source_t.h"

namespace drwn {
    class camera_t {
    public:
        static constexpr float VIEW_V_ANGLE = 46.0; //degree
        static constexpr float VIEW_H_ANGLE = 58.0; //degree

        static constexpr int WIDTH = 320;
        static constexpr int HEIGHT = 240;

        camera_t(camera_t&&) = delete;

        static camera_t* get_instance();

        void update_image();

        image_source_t* get_image_source() const noexcept;

        void set_image_source(image_source_t* img_source) noexcept;

        const cv::Mat& get_image() const noexcept;

    private:
        camera_t() = default;

    private:
        image_source_t* m_img_source{nullptr};
        cv::Mat m_img;
    };
}



