//
// Created by nikitas on 26.03.16.
//

#include <cv.hpp>
#include <functional>
#include "vision/detectors/line_detector_t.h"
#include "vision/vision_utils.h"

using namespace drwn;

int fill_neighbours(const cv::Mat& window, bool* neighbours) {
    constexpr int indexes[][2]{
            {0, 1},
            {0, 2},
            {1, 2},
            {2, 2},
            {2, 1},
            {2, 0},
            {1, 0},
            {0, 0}
    };

    int table_idx = 0;

    for (int i = 0; i < sizeof(indexes) / sizeof(*indexes); ++i) {
        const int* idx = indexes[i];
        neighbours[i] = window.at<uchar>(idx[0], idx[1]);
        table_idx = table_idx << 1 | neighbours[i];
    }
    return table_idx;
}

std::vector<cv::Vec4i> line_detector_t::detect(const cv::Mat& preproc_image) const {
    cv::Mat skeleton;

    this->get_skeleton(preproc_image, skeleton); // O(n) n = h*w of img

    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(skeleton, lines,
                    m_hough_lines_rho, m_hough_lines_theta, m_hough_lines_threshold,
                    m_hough_lines_min_line_length, m_hough_lines_max_line_gap);

    this->join_lines(lines); // O(m^2) - m = countLines
    return lines;
}

std::vector<cv::Vec4i> line_detector_t::join_lines(std::vector<cv::Vec4i>& lines) const {
    using namespace std::placeholders;

    std::vector<int> clusters;
    auto predicate = std::bind(&line_detector_t::is_lines_equal, this, _1, _2);
    const size_t num_clusters = (size_t) cv::partition(lines, clusters, predicate);
    std::vector<cv::Vec4i> joined_lines(num_clusters);

    for (std::size_t i = 0; i < lines.size(); ++i)
        joined_lines[clusters[i]] += lines[i];

    return joined_lines;
}

void line_detector_t::get_skeleton(const cv::Mat& img, cv::Mat& result) const {
    zhang_suen(img, result);
}

void line_detector_t::get_simple_skeleton(const cv::Mat& img, cv::Mat& result) const {
    result = cv::Mat::zeros(img.rows, img.cols, CV_8UC1);

    //scan x
    const uchar white = 255;
    for (int r = 0; r < img.rows; r++) {
        for (int c = 0, e; c < img.cols; c++) {
            uchar px = img.at<uchar>(r, c);
            if (px != white) continue;
            e = c;
            while (px == white && e < img.cols)
                px = img.at<uchar>(r, e++);
            int dx = e - c;
            if (dx > 1 && dx < 30)
                result.at<uchar>(r, c + dx / 2) = 255;
            c = e;
        }
    }

    //scan y
    for (int c = 0; c < img.cols; c++) {
        for (int r = 0, e; r < img.rows; r++) {
            uchar px = img.at<uchar>(r, c);
            if (px != white) continue;
            e = r;
            while (px == white && e < img.rows)
                px = img.at<uchar>(e++, c);
            int dx = e - r;
            if (dx > 1 && dx < 30)
                result.at<uchar>(r + dx / 2, c) = 255;
            r = e;
        }
    }
}

void line_detector_t::zhang_suen(const cv::Mat& img, cv::Mat& result) const {
    static constexpr char table[]{
            0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 0, 0, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0,
            0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            1, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            1, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0
    };

    result = img.clone();

    assert(result.depth() == CV_8U);
    assert(result.channels() == 1);

    const int rows = result.rows;
    const int cols = result.cols;

    bool neighbours[8];

    std::vector<cv::Point> points(rows * cols);

    int inter = 0, count;
    do {
        count = 0;

        for (int r = 1; r < rows - 1; ++r) {
            for (int c = 1; c < cols - 1; ++c) {
                if (!result.at<uchar>(r, c))
                    continue;

                const cv::Rect window(c - 1, r - 1, 3, 3);
                int idx = fill_neighbours(result(window), neighbours);
                if (table[idx]) points[count++] = cv::Point(c, r);
            }
        }

        for (int i = 0; i < count; ++i)
            result.at<uchar>(points[i]) = 0;
        inter++;
    } while (count != 0);
}

bool line_detector_t::is_lines_equal(const cv::Vec4i& line1, const cv::Vec4i& line2) const {
    using namespace vision_utils;

    const cv::Point a(line1(0), line1(1));
    const cv::Point b(line1(2), line1(3));
    const cv::Point c(line2(2), line2(3));

    const float angle = get_angle(get_vector<float>(line1), get_vector<float>(line2));
    const bool is_parallel = angle < m_line_equality_angle_eps;
    const bool is_point_in_line = std::abs(get_altitude(c, a, b)) < m_line_equality_error_px;

    return is_parallel && is_point_in_line;
}

float line_detector_t::get_hough_lines_rho() const noexcept {
    return m_hough_lines_rho;
}

void line_detector_t::set_hough_lines_rho(float hough_lines_rho) noexcept {
    m_hough_lines_rho = hough_lines_rho;
}

float line_detector_t::get_hough_lines_theta() const noexcept {
    return m_hough_lines_theta;
}

void line_detector_t::set_hough_lines_theta(float hough_lines_theta) noexcept {
    m_hough_lines_theta = hough_lines_theta;
}

float line_detector_t::get_hough_lines_min_line_length() const noexcept {
    return m_hough_lines_min_line_length;
}

void line_detector_t::set_hough_lines_min_line_length(float hough_lines_min_line_length) noexcept {
    m_hough_lines_min_line_length = hough_lines_min_line_length;
}

float line_detector_t::get_hough_lines_max_line_gap() const noexcept {
    return m_hough_lines_max_line_gap;
}

void line_detector_t::set_hough_lines_max_line_gap(float hough_lines_max_line_gap) noexcept {
    m_hough_lines_max_line_gap = hough_lines_max_line_gap;
}

int line_detector_t::get_hough_lines_threshold() const noexcept {
    return m_hough_lines_threshold;
}

void line_detector_t::set_hough_lines_threshold(int hough_lines_threshold) noexcept {
    m_hough_lines_threshold = hough_lines_threshold;
}

float line_detector_t::get_line_equality_angle_eps() const noexcept {
    return m_line_equality_angle_eps;
}

void line_detector_t::set_line_equality_angle_eps(float line_equality_pred_angle_eps) noexcept {
    m_line_equality_angle_eps = line_equality_pred_angle_eps;
}

int line_detector_t::get_line_equality_error_px() const noexcept {
    return m_line_equality_error_px;
}

void line_detector_t::set_line_equality_error_px(int line_equality_pred_error_px) noexcept {
    m_line_equality_error_px = line_equality_pred_error_px;
}

//void line_detector_t::load(const boost::property_tree::ptree& config) {
//    const boost::property_tree::ptree line_config = config.get_child(DetectorName());
//
//    m_conf.HoughLines.max_line_gap = line_config.get<double>("HoughLines.max_line_gap");
//    m_conf.HoughLines.min_line_length = line_config.get<double>("HoughLines.min_line_length");
//    m_conf.HoughLines.rho = line_config.get<double>("HoughLines.rho");
//    m_conf.HoughLines.theta = line_config.get<double>("HoughLines.theta");
//    m_conf.HoughLines.threshold = line_config.get<int>("HoughLines.threshold");
//    m_conf.LineEqualPredicate.angle_eps = line_config.get<float>("LineEqualPredicate.angle_eps");
//    m_conf.LineEqualPredicate.error_px = line_config.get<int>("LineEqualPredicate.error_px");
//    m_conf.Preproc.kernel_size = line_config.get<int>("Preproc.kernel_size");
//    m_conf.Preproc.min_thresh = line_config.get<int>("Preproc.min_thresh");
//    m_conf.Preproc.ColorThresh.min_1 = line_config.get<uchar>("Preproc.ColorThresh.min_1");
//    m_conf.Preproc.ColorThresh.min_2 = line_config.get<uchar>("Preproc.ColorThresh.min_2");
//    m_conf.Preproc.ColorThresh.min_3 = line_config.get<uchar>("Preproc.ColorThresh.min_3");
//    m_conf.Preproc.ColorThresh.max_1 = line_config.get<uchar>("Preproc.ColorThresh.max_1");
//    m_conf.Preproc.ColorThresh.max_2 = line_config.get<uchar>("Preproc.ColorThresh.max_2");
//    m_conf.Preproc.ColorThresh.max_3 = line_config.get<uchar>("Preproc.ColorThresh.max_3");
//
//    m_conf.Preproc_new.kernel_size = line_config.get<int>("Preproc_new.kernel_size");
//    m_conf.Preproc_new.min_thresh = line_config.get<int>("Preproc_new.min_thresh");
//    m_conf.Preproc_new.ColorThresh.min_1 = line_config.get<uchar>("Preproc_new.ColorThresh1.min_1");
//    m_conf.Preproc_new.ColorThresh.min_2 = line_config.get<uchar>("Preproc_new.ColorThresh1.min_2");
//    m_conf.Preproc_new.ColorThresh.min_3 = line_config.get<uchar>("Preproc_new.ColorThresh1.min_3");
//    m_conf.Preproc_new.ColorThresh.max_1 = line_config.get<uchar>("Preproc_new.ColorThresh1.max_1");
//    m_conf.Preproc_new.ColorThresh.max_2 = line_config.get<uchar>("Preproc_new.ColorThresh1.max_2");
//    m_conf.Preproc_new.ColorThresh.max_3 = line_config.get<uchar>("Preproc_new.ColorThresh1.max_3");
//    m_conf.Preproc_new.ColorThresh2.min_1 = line_config.get<uchar>("Preproc_new.ColorThresh2.min_1");
//    m_conf.Preproc_new.ColorThresh2.min_2 = line_config.get<uchar>("Preproc_new.ColorThresh2.min_2");
//    m_conf.Preproc_new.ColorThresh2.min_3 = line_config.get<uchar>("Preproc_new.ColorThresh2.min_3");
//    m_conf.Preproc_new.ColorThresh2.max_1 = line_config.get<uchar>("Preproc_new.ColorThresh2.max_1");
//    m_conf.Preproc_new.ColorThresh2.max_2 = line_config.get<uchar>("Preproc_new.ColorThresh2.max_2");
//    m_conf.Preproc_new.ColorThresh2.max_3 = line_config.get<uchar>("Preproc_new.ColorThresh2.max_3");
//}


