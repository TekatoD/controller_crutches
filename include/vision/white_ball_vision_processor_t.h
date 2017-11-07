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

        const cv::Scalar& get_ball_preprocessor_threshold_gabor_bgr_min() const;

        void set_ball_preprocessor_threshold_gabor_bgr_min(const cv::Scalar& threshold_gabor_bgr_min);

        const cv::Scalar& get_ball_preprocessor_threshold_gabor_bgr_max() const;

        void set_ball_preprocessor_threshold_gabor_bgr_max(const cv::Scalar& threshold_gabor_bgr_max);

        const cv::Scalar& get_ball_preprocessor_threshold_color_bgr_min() const;

        void set_ball_preprocessor_threshold_color_bgr_min(const cv::Scalar& threshold_color_bgr_min);

        const cv::Scalar& get_ball_preprocessor_threshold_color_bgr_max() const;

        void set_ball_preprocessor_threshold_color_bgr_max(const cv::Scalar& threshold_color_bgr_max);

        int get_ball_preprocessor_median_blur_size() const;

        void set_ball_preprocessor_median_blur_size(int median_blur_size);
        

        const cv::Scalar& get_field_preprocessor_threshold_gabor_bgr_min() const;

        void set_field_preprocessor_threshold_gabor_bgr_min(const cv::Scalar& threshold_gabor_bgr_min);

        const cv::Scalar& get_field_preprocessor_threshold_gabor_bgr_max() const;

        void set_field_preprocessor_threshold_gabor_bgr_max(const cv::Scalar& threshold_gabor_bgr_max);

        const cv::Scalar& get_field_preprocessor_threshold_color_hsv_min() const;

        void set_field_preprocessor_threshold_color_hsv_min(const cv::Scalar& threshold_color_hsv_min);

        const cv::Scalar& get_field_preprocessor_threshold_color_hsv_max() const;

        void set_field_preprocessor_threshold_color_hsv_max(const cv::Scalar& threshold_color_hsv_max);


        const cv::Scalar& get_line_preprocessor_threshold_gabor_bgr_min() const;

        void set_line_preprocessor_threshold_gabor_bgr_min(const cv::Scalar& threshold_bgr_min);

        const cv::Scalar& get_line_preprocessor_threshold_gabor_bgr_max() const;

        void set_line_preprocessor_threshold_gabor_bgr_max(const cv::Scalar& threshold_bgr_max);

        const cv::Scalar& get_line_preprocessor_threshold_color_hsv_min() const;

        void set_line_preprocessor_threshold_color_hsv_min(const cv::Scalar& threshold_hsv_min);

        const cv::Scalar& get_line_preprocessor_threshold_color_hsv_max() const;

        void set_line_preprocessor_threshold_color_hsv_max(const cv::Scalar& threshold_hsv_max);


        float get_line_detector_hough_lines_rho() const;

        void set_line_detector_hough_lines_rho(float hough_lines_rho);

        float get_line_detector_hough_lines_theta() const;

        void set_line_detector_hough_lines_theta(float hough_lines_theta);

        float get_line_detector_hough_lines_min_line_length() const;

        void set_line_detector_hough_lines_min_line_length(float hough_lines_min_line_length);

        float get_line_detector_hough_lines_max_line_gap() const;

        void set_line_detector_hough_lines_max_line_gap(float hough_lines_max_line_gap);

        int get_line_detector_hough_lines_threshold() const;

        void set_line_detector_hough_lines_threshold(int hough_lines_threshold);

        float get_line_detector_line_equality_angle_eps() const;

        void set_line_detector_line_equality_angle_eps(float line_equality_pred_angle_eps);

        int get_line_detector_line_equality_error_px() const;

        void set_line_detector_line_equality_error_px(int line_equality_pred_error_px);

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



