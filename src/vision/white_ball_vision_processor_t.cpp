/// \autor arssivka
/// \date 11/3/17

#include <cv.hpp>
#include <log/trivial_logger_t.h>
#include "vision/white_ball_vision_processor_t.h"

using namespace drwn;

const cv::Rect& white_ball_vision_processor_t::detect_ball() {
    this->process();
    return m_ball;
}

const std::vector<cv::Vec4i>& white_ball_vision_processor_t::detect_lines() {
    this->process();
    return m_lines;
}

const std::vector<cv::Vec3d>& white_ball_vision_processor_t::detect_angles() {
    throw std::runtime_error("detect angles isn't implemented");
}

const cv::Mat& white_ball_vision_processor_t::detect_field() {
    this->process();
    return m_field_mask;
}

void white_ball_vision_processor_t::set_frame(cv::Mat frame) {
    this->reset();
    m_src_img = frame;
    if (m_debug) LOG_DEBUG << "WHITE BALL VISION PROCESSOR: New frame has been set";
}

void white_ball_vision_processor_t::reset() {
    m_img_processed = false;
    m_lines.clear();
    m_src_img = cv::Mat();
    m_field_img = cv::Mat();
    m_field_mask = cv::Mat();
}

bool white_ball_vision_processor_t::is_debug_enabled() const noexcept {
    return m_debug;
}

void white_ball_vision_processor_t::enable_debug(bool debug) noexcept {
    m_debug = debug;
}

void white_ball_vision_processor_t::process() {
    if (!m_img_processed) {
        this->update_members();
        this->show_windows();
        this->dump_images();
    }
}

void white_ball_vision_processor_t::update_members() {

    if (m_field_processing_enabled) {
        if (m_debug) LOG_DEBUG << "WHITE BALL VISION PROCESSOR: Processing field";
        cv::Mat field_prep = m_field_preproc.preprocess(m_src_img);
        m_field_mask = m_field_detector.detect(field_prep);
        cv::Mat tmp;
        m_src_img.copyTo(tmp, m_field_mask);
        m_field_img = tmp;
    }

    auto& img = (m_field_processing_enabled) ? m_field_img : m_src_img;

    if (m_debug) LOG_DEBUG << "WHITE BALL VISION PROCESSOR: Detecting lines";
    m_line_preproc_img = m_line_preproc.preprocess(img);
    m_lines = m_line_detector.detect(m_line_preproc_img);
    if (m_debug) LOG_DEBUG << "WHITE BALL VISION PROCESSOR: Detecting white ball";
    m_ball_detector.detect(img, m_lines);
    m_img_processed = true;
    if (m_debug) LOG_DEBUG << "WHITE BALL VISION PROCESSOR: Processing has been finished";
}

const cv::Scalar& white_ball_vision_processor_t::get_ball_preprocessor_threshold_gabor_bgr_min() const {
    return m_ball_preproc.get_threshold_gabor_bgr_min();
}

void white_ball_vision_processor_t::set_ball_preprocessor_threshold_gabor_bgr_min(
        const cv::Scalar& threshold_gabor_bgr_min) {
    if (m_debug) LOG_DEBUG << "WHITE BALL VISION PROCESSOR: threshold_gabor_bgr_min: " << threshold_gabor_bgr_min;
    m_ball_preproc.set_threshold_gabor_bgr_min(threshold_gabor_bgr_min);
}

const cv::Scalar& white_ball_vision_processor_t::get_ball_preprocessor_threshold_gabor_bgr_max() const {
    return m_ball_preproc.get_threshold_gabor_bgr_max();
}

void white_ball_vision_processor_t::set_ball_preprocessor_threshold_gabor_bgr_max(
        const cv::Scalar& threshold_gabor_bgr_max) {
    if (m_debug) LOG_DEBUG << "WHITE BALL VISION PROCESSOR: threshold_gabor_bgr_max: " << threshold_gabor_bgr_max;
    m_ball_preproc.set_threshold_gabor_bgr_max(threshold_gabor_bgr_max);
}

const cv::Scalar& white_ball_vision_processor_t::get_ball_preprocessor_threshold_color_bgr_min() const {
    return m_ball_preproc.get_threshold_color_bgr_min();
}

void white_ball_vision_processor_t::set_ball_preprocessor_threshold_color_bgr_min(
        const cv::Scalar& threshold_color_bgr_min) {
    if (m_debug) LOG_DEBUG << "WHITE BALL VISION PROCESSOR: threshold_color_bgr_min: " << threshold_color_bgr_min;
    m_ball_preproc.set_threshold_color_bgr_min(threshold_color_bgr_min);
}

const cv::Scalar& white_ball_vision_processor_t::get_ball_preprocessor_threshold_color_bgr_max() const {
    return m_ball_preproc.get_threshold_color_bgr_max();
}

void white_ball_vision_processor_t::set_ball_preprocessor_threshold_color_bgr_max(
        const cv::Scalar& threshold_color_bgr_max) {
    if (m_debug) LOG_DEBUG << "WHITE BALL VISION PROCESSOR: threshold_color_bgr_max: " << threshold_color_bgr_max;
    m_ball_preproc.set_threshold_color_bgr_max(threshold_color_bgr_max);
}

int white_ball_vision_processor_t::get_ball_preprocessor_median_blur_size() const {
    return m_ball_preproc.get_median_blur_size();
}

void white_ball_vision_processor_t::set_ball_preprocessor_median_blur_size(int median_blur_size) {
    if (m_debug) LOG_DEBUG << "WHITE BALL VISION PROCESSOR: median_blur_size: " << median_blur_size;
    m_ball_preproc.set_median_blur_size(median_blur_size);
}

const cv::Scalar& white_ball_vision_processor_t::get_field_preprocessor_threshold_gabor_bgr_min() const {
    return m_field_preproc.get_threshold_gabor_bgr_min();
}

void white_ball_vision_processor_t::set_field_preprocessor_threshold_gabor_bgr_min(
        const cv::Scalar& threshold_gabor_bgr_min) {
    if (m_debug) LOG_DEBUG << "WHITE BALL VISION PROCESSOR: threshold_gabor_bgr_min: " << threshold_gabor_bgr_min;
    m_field_preproc.set_threshold_gabor_bgr_min(threshold_gabor_bgr_min);
}

const cv::Scalar& white_ball_vision_processor_t::get_field_preprocessor_threshold_gabor_bgr_max() const {
    return m_field_preproc.get_threshold_gabor_bgr_max();
}

void white_ball_vision_processor_t::set_field_preprocessor_threshold_gabor_bgr_max(
        const cv::Scalar& threshold_gabor_bgr_max) {
    if (m_debug) LOG_DEBUG << "WHITE BALL VISION PROCESSOR: threshold_gabor_bgr_max: " << threshold_gabor_bgr_max;
    m_field_preproc.set_threshold_gabor_bgr_max(threshold_gabor_bgr_max);
}

const cv::Scalar& white_ball_vision_processor_t::get_field_preprocessor_threshold_color_hsv_min() const {
    return m_field_preproc.get_threshold_color_hsv_min();
}

void white_ball_vision_processor_t::set_field_preprocessor_threshold_color_hsv_min(
        const cv::Scalar& threshold_color_hsv_min) {
    if (m_debug) LOG_DEBUG << "WHITE BALL VISION PROCESSOR: threshold_color_hsv_min: " << threshold_color_hsv_min;
    m_field_preproc.set_threshold_color_hsv_min(threshold_color_hsv_min);
}

const cv::Scalar& white_ball_vision_processor_t::get_field_preprocessor_threshold_color_hsv_max() const {
    return m_field_preproc.get_threshold_color_hsv_max();
}

void white_ball_vision_processor_t::set_field_preprocessor_threshold_color_hsv_max(
        const cv::Scalar& threshold_color_hsv_max) {
    if (m_debug) LOG_DEBUG << "WHITE BALL VISION PROCESSOR: threshold_color_hsv_max: " << threshold_color_hsv_max;
    m_field_preproc.set_threshold_color_hsv_max(threshold_color_hsv_max);
}

const cv::Scalar& white_ball_vision_processor_t::get_line_preprocessor_threshold_gabor_bgr_min() const {
    return m_line_preproc.get_threshold_gabor_bgr_min();
}

void
white_ball_vision_processor_t::set_line_preprocessor_threshold_gabor_bgr_min(const cv::Scalar& threshold_bgr_min) {
    if (m_debug) LOG_DEBUG << "WHITE BALL VISION PROCESSOR: threshold_bgr_min: " << threshold_bgr_min;
    m_line_preproc.set_threshold_gabor_bgr_min(threshold_bgr_min);
}

const cv::Scalar& white_ball_vision_processor_t::get_line_preprocessor_threshold_gabor_bgr_max() const {
    return m_line_preproc.get_threshold_gabor_bgr_max();
}

void
white_ball_vision_processor_t::set_line_preprocessor_threshold_gabor_bgr_max(const cv::Scalar& threshold_bgr_max) {
    if (m_debug) LOG_DEBUG << "WHITE BALL VISION PROCESSOR: threshold_bgr_max: " << threshold_bgr_max;
    m_line_preproc.set_threshold_gabor_bgr_max(threshold_bgr_max);
}

const cv::Scalar& white_ball_vision_processor_t::get_line_preprocessor_threshold_color_hsv_min() const {
    return m_line_preproc.get_threshold_gabor_hsv_min();
}

void
white_ball_vision_processor_t::set_line_preprocessor_threshold_color_hsv_min(const cv::Scalar& threshold_hsv_min) {
    if (m_debug) LOG_DEBUG << "WHITE BALL VISION PROCESSOR: threshold_hsv_min: " << threshold_hsv_min;
    m_line_preproc.set_threshold_color_hsv_min(threshold_hsv_min);
}

const cv::Scalar& white_ball_vision_processor_t::get_line_preprocessor_threshold_color_hsv_max() const {
    return m_line_preproc.get_threshold_color_hsv_max();
}

void white_ball_vision_processor_t::set_line_preprocessor_threshold_color_hsv_max(
        const cv::Scalar& threshold_hsv_max) {
    if (m_debug) LOG_DEBUG << "WHITE BALL VISION PROCESSOR: threshold_hsv_max: " << threshold_hsv_max;
    m_line_preproc.set_threshold_hsv_max(threshold_hsv_max);
}

float white_ball_vision_processor_t::get_line_detector_hough_lines_rho() const {
    return m_line_detector.get_hough_lines_rho();
}

void drwn::white_ball_vision_processor_t::set_line_detector_hough_lines_rho(float hough_lines_rho) {
    m_line_detector.set_hough_lines_rho(hough_lines_rho);
}

float white_ball_vision_processor_t::get_line_detector_hough_lines_theta() const {
    return m_line_detector.get_hough_lines_theta();
}

void drwn::white_ball_vision_processor_t::set_line_detector_hough_lines_theta(float hough_lines_theta) {
    m_line_detector.set_hough_lines_theta(hough_lines_theta);
}

float white_ball_vision_processor_t::get_line_detector_hough_lines_min_line_length() const {
    return m_line_detector.get_hough_lines_min_line_length();
}

void
drwn::white_ball_vision_processor_t::set_line_detector_hough_lines_min_line_length(float hough_lines_min_line_length) {
    m_line_detector.set_hough_lines_min_line_length(hough_lines_min_line_length);
}

float white_ball_vision_processor_t::get_line_detector_hough_lines_max_line_gap() const {
    return m_line_detector.get_hough_lines_max_line_gap();
}

void drwn::white_ball_vision_processor_t::set_line_detector_hough_lines_max_line_gap(float hough_lines_max_line_gap) {
    m_line_detector.set_hough_lines_max_line_gap(hough_lines_max_line_gap);
}

int white_ball_vision_processor_t::get_line_detector_hough_lines_threshold() const {
    return m_line_detector.get_hough_lines_threshold();
}

void drwn::white_ball_vision_processor_t::set_line_detector_hough_lines_threshold(int hough_lines_threshold) {
    m_line_detector.set_hough_lines_threshold(hough_lines_threshold);
}

float white_ball_vision_processor_t::get_line_detector_line_equality_angle_eps() const {
    return m_line_detector.get_line_equality_angle_eps();
}

void drwn::white_ball_vision_processor_t::set_line_detector_line_equality_angle_eps(float line_equality_pred_angle_eps) {
    m_line_detector.set_line_equality_angle_eps(line_equality_pred_angle_eps);
}

int white_ball_vision_processor_t::get_line_detector_line_equality_error_px() const {
    return m_line_detector.get_line_equality_error_px();
}

void drwn::white_ball_vision_processor_t::set_line_detector_line_equality_error_px(int line_equality_pred_error_px) {
    m_line_detector.set_line_equality_error_px(line_equality_pred_error_px);
}

void white_ball_vision_processor_t::show_windows() {
    if (m_show_images_enabled) {
        cv::imshow("source", m_src_img);
        if (m_field_processing_enabled) {
            cv::imshow("field", m_field_img);
        }

        cv::Mat lines_img = (m_field_processing_enabled)
                           ? m_field_img.clone()
                           : m_src_img.clone();

        for (auto& line : m_lines) {
            cv::line(lines_img, {line[0], line[1]}, {line[2], line[3]}, {255, 0, 0});
        }
        cv::imshow("lines", lines_img);

        cv::Mat ball_img = (m_field_processing_enabled)
                           ? m_field_img.clone()
                           : m_src_img.clone();

        cv::rectangle(ball_img, m_ball, {255, 0, 0});
        cv::imshow("ball", ball_img);

        cv::waitKey(1);
    }
}

void white_ball_vision_processor_t::dump_images() {
    if (m_dump_images_enabled) {

    }
}

bool white_ball_vision_processor_t::is_show_images_enabled() const {
    return m_show_images_enabled;
}

void white_ball_vision_processor_t::enable_show_images(bool show_images_enabled) {
    if (show_images_enabled != m_show_images_enabled) {
        if (show_images_enabled) {
            cv::namedWindow("source");
            if (m_field_processing_enabled) {
                cv::namedWindow("field");
            }
            cv::namedWindow("ball");
            cv::namedWindow("lines");
        } else {
            cv::destroyWindow("source");
            if (m_field_processing_enabled) {
                cv::destroyWindow("field");
            }
            cv::destroyWindow("ball");
            cv::destroyWindow("lines");
        }
        m_show_images_enabled = show_images_enabled;
    }
}

bool white_ball_vision_processor_t::is_dump_images_enabled() const {
    return m_dump_images_enabled;
}

void white_ball_vision_processor_t::enable_dump_images(bool dump_images_enabled) {
    m_dump_images_enabled = dump_images_enabled;
}

bool white_ball_vision_processor_t::is_field_processing_enabled() const {
    return m_field_processing_enabled;
}

void white_ball_vision_processor_t::enable_field_processing(bool field_processing_enabled) {
    if (m_field_processing_enabled != field_processing_enabled) {
        if (field_processing_enabled) {
            cv::namedWindow("field");
        } else {
            cv::destroyWindow("field");
        }
        m_field_processing_enabled = field_processing_enabled;
    }
}
