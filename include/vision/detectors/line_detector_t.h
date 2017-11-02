//
// Created by nikitas on 26.03.16.
//

#pragma once

#include "preprocessor_t.h"

namespace drwn {
    class line_detector_t {
    public:
        std::vector<cv::Vec4i> detect(const cv::Mat& preproc_image) const;

        float get_hough_lines_rho() const noexcept;

        void set_hough_lines_rho(float hough_lines_rho) noexcept;

        float get_hough_lines_theta() const noexcept;

        void set_hough_lines_theta(float hough_lines_theta) noexcept;

        float get_hough_lines_min_line_length() const noexcept;

        void set_hough_lines_min_line_length(float hough_lines_min_line_length) noexcept;

        float get_hough_lines_max_line_gap() const noexcept;

        void set_hough_lines_max_line_gap(float hough_lines_max_line_gap) noexcept;

        int get_hough_lines_threshold() const noexcept;

        void set_hough_lines_threshold(int hough_lines_threshold) noexcept;

        float get_line_equality_angle_eps() const noexcept;

        void set_line_equality_angle_eps(float line_equality_pred_angle_eps) noexcept;

        int get_line_equality_error_px() const noexcept;

        void set_line_equality_error_px(int line_equality_pred_error_px) noexcept;

    private:
        void get_skeleton(const cv::Mat& img, cv::Mat& result) const;

        void get_simple_skeleton(const cv::Mat& img, cv::Mat& result) const;

        void zhang_suen(const cv::Mat& img, cv::Mat& result) const;

        std::vector<cv::Vec4i> join_lines(std::vector<cv::Vec4i>& lines) const;

        bool is_lines_equal(const cv::Vec4i& line1, const cv::Vec4i& line2) const;

    private:
        float m_hough_lines_rho{0.5};
        float m_hough_lines_theta{0.0027};
        float m_hough_lines_min_line_length{0};
        float m_hough_lines_max_line_gap{10};
        int m_hough_lines_threshold{28};

        float m_line_equality_angle_eps{0.048};
        int m_line_equality_error_px{4};
    };


}
