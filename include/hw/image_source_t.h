/// \autor arssivka
/// \date 11/3/17

#pragma once


#include <opencv2/core/mat.hpp>

namespace drwn {
    class image_source_t {
    public:
        image_source_t() = default;
        image_source_t(const image_source_t&) = delete;
        image_source_t& operator=(const image_source_t&) = delete;

        virtual cv::Mat capture_frame() = 0;

        virtual ~image_source_t() = default;

    };
}



