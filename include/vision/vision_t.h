//
// Created by nikitas on 26.03.16.
//

#pragma once

#include "vision_processor_t.h"

namespace drwn {
    class vision_t {
    public:
        static vision_t* get_instance();

        cv::Rect detect_ball();

        std::vector<cv::Vec4i> detect_lines() {
            return m_processor->detect_lines();
        }

        std::vector<cv::Vec3d> detect_angles();

        cv::Mat detect_field();

        void set_frame(cv::Mat frame);

        vision_processor_t* get_processor() const noexcept;

        void set_processor(vision_processor_t* processor) noexcept;

        void process() {
             m_processor->process();
        }

    private:
        vision_t() = default;

    private:
        vision_processor_t* m_processor{nullptr};
    };
}
