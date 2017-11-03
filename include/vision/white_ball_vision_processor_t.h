/// \autor arssivka
/// \date 11/3/17

#pragma once


#include <vision/detectors/ball_preprocessor_t.h>
#include <vision/detectors/field_preprocessor_t.h>
#include <vision/detectors/new_line_preprocessor_t.h>
#include <vision/detectors/white_ball_detector_t.h>
#include <vision/detectors/line_detector_t.h>
#include <vision/detectors/field_detector_t.h>
#include "vision_processor_t.h"

namespace drwn {
    class white_ball_vision_processor_t : public vision_processor_t {
    public:
        const cv::Rect& detect_ball() override;

        const std::vector<cv::Vec4i>& detect_lines() override;

        const std::vector<cv::Vec3d>& detect_angles() override;

        const cv::Mat& detect_field() override;

        void set_frame(cv::Mat frame) override;

        void reset() override;

        bool is_debug_enabled() const noexcept;

        void enable_debug(bool debug) noexcept;

    private:
        void process();

    private:
        bool m_debug{false};

        cv::Mat m_img;

        cv::Rect m_ball;
        cv::Mat m_field_mask;
        std::vector<cv::Vec4i> m_lines{};
        std::vector<cv::Vec3d> m_angles{};

        bool m_img_processed{false};

        ball_preprocessor_t m_ball_preproc;
        new_line_preprocessor_t m_line_preproc;
        field_preprocessor_t m_field_preproc;

        white_ball_detector_t m_ball_detector;
        field_detector_t m_field_detector;
        line_detector_t m_line_detector;
    };
}



