/// \autor arssivka
/// \date 11/2/17

#include <log/trivial_logger_t.h>
#include "vision/detectors/white_ball_detector_t.h"

using namespace drwn;

cv::Rect white_ball_detector_t::detect(const cv::Mat& prep_img, const cv::Mat& src_img, const std::vector<cv::Vec4i>& lines) const {
    if(m_detector_type == 1) {
        std::vector<cv::Rect> balls;
        m_ball_cascade.detectMultiScale(src_img, balls, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, cv::Size(30, 30));
        if (!balls.empty()) {
            if(m_network_enabled) {
                size_t ind = 0;
                float wieght = 0;
                bool found = false;
                for(size_t i = 0; i < balls.size(); ++i) {
                    cv::Mat rec = src_img(balls[i]);
                    cv::cvtColor(rec, rec, cv::COLOR_BGR2GRAY);
                    cv::resize(rec, rec, m_network_window);
                    rec.convertTo(rec, CV_32F);
                    m_network->predict(rec.reshape(1, 1), m_classes);
                    if (m_classes.at<float>(0, 0) > m_classes.at<float>(0, 1)) {
                        found = true;
                        if (m_classes.at<float>(0, 0) > wieght) {
                            wieght = m_classes.at<float>(0, 0);
                            ind = i;
                        }
                    }
                }
                if(found) {
                    return balls[ind];
                } else {
                    return cv::Rect{};
                }

            } else {
                return balls[0];
            }
        }
        return cv::Rect{};
    } else if(m_detector_type == 0) {
        cv::Mat preproc = prep_img;
        for (auto&& line : lines) {
            double x0 = line(0);
            double x1 = line(2);
            double y0 = line(1);
            double y1 = line(3);
            auto k = (y1 - y0) / (x1 - x0);
            auto m = y1 - k * x1;
            if (x0 > x1) {
                std::swap(x0, x1);
                std::swap(y0, y1);
                k = -k;
            }
            auto R = 10;
            for (int x = x0; x <= x1; x++) {
                auto y = k * (double) x + m;
                auto i_limit = (x + R < preproc.cols) ? x + R : preproc.cols;
                for (int i = (((x - R) >= 0) ? x - R : 0); i < i_limit; i++) {
                    if (y != y) {
                        auto j_limit = y0 > y1 ? y0 : y1;
                        for (int j = (y0 < y1 ? y0 : y1); j < j_limit; j++) {
                            preproc.at<uchar>(j, i) = 0;
                        }
                    } else {
                        auto j_limit = (y + R < preproc.rows) ? y + R : preproc.rows;
                        for (int j = (((y - R) >= 0) ? y - R : 0);
                             j < j_limit; j++) {
                            preproc.at<uchar>(j, i) = 0;
                        }
                    }
                }
            }
        }
        cv::Rect result;
        std::vector<std::vector<cv::Point> > contours;
        cv::findContours(preproc, contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
        double max_c = 2.0;
        double min_c = 0.5;
        double cur_c = 0.0;

        if (!contours.empty()) {
            double max_area = 0;
            int max_area_idx = -1;

            for (size_t i = 0; i < contours.size(); i++) {
                const double area = cv::contourArea(contours[i]);
                if (area > m_area_low && area < m_area_top && area > max_area) {
                    cv::Mat a_matrix = cv::Mat::zeros(5, 5, CV_64F);
                    cv::Mat b_vector = cv::Mat::zeros(5, 1, CV_64F);
                    for (auto&& point : contours[i]) {
                        a_matrix.at<double>(0, 0) += std::pow((double) point.x, 4.0);
                        auto x_2 = std::pow((double) point.x, 2.0);
                        auto x_3 = std::pow((double) point.x, 3.0);
                        auto y_2 = std::pow((double) point.y, 2.0);
                        auto y_4 = std::pow((double) point.y, 4.0);
                        auto y_3 = std::pow((double) point.y, 3.0);

                        a_matrix.at<double>(0, 1) += x_2 * y_2;
                        a_matrix.at<double>(0, 2) += 2.0 * x_3 * point.y;
                        a_matrix.at<double>(0, 3) += 2.0 * x_3;
                        a_matrix.at<double>(0, 4) += 2.0 * x_2 * point.y;
                        //
                        a_matrix.at<double>(1, 0) += x_2 * y_2;
                        a_matrix.at<double>(1, 1) += y_4;
                        a_matrix.at<double>(1, 2) += 2.0 * point.x * y_3;
                        a_matrix.at<double>(1, 3) += 2.0 * point.x * y_2;
                        a_matrix.at<double>(1, 4) += 2.0 * y_3;
                        //
                        a_matrix.at<double>(2, 0) += std::pow((long double) point.x, 3.0) * (long double) point.y;
                        a_matrix.at<double>(2, 1) += point.x * y_3;
                        a_matrix.at<double>(2, 2) +=
                                2.0 * x_2 * y_2;
                        a_matrix.at<double>(2, 3) += 2.0 * x_2 * point.y;
                        a_matrix.at<double>(2, 4) += 2.0 * point.x * y_2;
                        //
                        a_matrix.at<double>(3, 0) += x_3;
                        a_matrix.at<double>(3, 1) += point.x * y_2;
                        a_matrix.at<double>(3, 2) += 2.0 * point.x * y_2;
                        a_matrix.at<double>(3, 3) += 2.0 * x_2;
                        a_matrix.at<double>(3, 4) += 2.0 * point.x * point.y;
                        //
                        a_matrix.at<double>(4, 0) += x_2 * point.y;
                        a_matrix.at<double>(4, 1) += y_3;
                        a_matrix.at<double>(4, 2) += 2.0 * point.x * y_2;
                        a_matrix.at<double>(4, 3) += 2.0 * point.x * point.y;
                        a_matrix.at<double>(4, 4) += 2.0 * y_2;
                        //
                        b_vector.at<double>(0) += x_2;
                        b_vector.at<double>(1) += y_2;
                        b_vector.at<double>(2) += (double) point.x * (double) point.y;
                        b_vector.at<double>(3) += point.x;
                        b_vector.at<double>(4) += point.y;
                    }
                    cv::Mat inverse_a_matrix = a_matrix.inv();
                    cv::Mat params = inverse_a_matrix * b_vector;
                    double a11 = params.at<double>(0);
                    double a22 = params.at<double>(1);
                    double a12 = params.at<double>(2);
                    double a13 = params.at<double>(3);
                    double a23 = params.at<double>(4);
                    double a33 = -1.0;
                    cv::Mat delta = cv::Mat::zeros(3, 3, CV_64F);
                    delta.at<double>(0, 0) = a11;
                    delta.at<double>(0, 1) = a12;
                    delta.at<double>(0, 2) = a13;
                    //
                    delta.at<double>(1, 0) = a12;
                    delta.at<double>(1, 1) = a22;
                    delta.at<double>(1, 2) = a23;
                    //
                    delta.at<double>(2, 0) = a13;
                    delta.at<double>(2, 1) = a23;
                    delta.at<double>(2, 2) = a33;
                    cv::Mat d_matrix = cv::Mat::zeros(2, 2, CV_64F);
                    d_matrix.at<double>(0, 0) = a11;
                    d_matrix.at<double>(0, 1) = a12;
                    d_matrix.at<double>(1, 0) = a12;
                    d_matrix.at<double>(1, 1) = a22;
                    cv::Mat lam;
                    cv::eigen(d_matrix, lam);
                    if (!lam.empty()) {
                        double delta_det = cv::determinant(delta);
                        double d_det = cv::determinant(d_matrix);
                        if (std::abs(lam.at<double>(0)) < 0.0001) {
                            lam.at<double>(0) = -std::abs(lam.at<double>(0));
                        }
                        if(std::abs(lam.at<double>(1)) < 0.0001){
                            lam.at<double>(1) = -std::abs(lam.at<double>(1));
                        }
                        double a_2 = -1.0 / lam.at<double>(0) * delta_det / d_det;
                        double b_2 = -1.0 / lam.at<double>(1) * delta_det / d_det;
                        double a = std::sqrt(a_2);
                        double b = std::sqrt(b_2);
                        double c = 0.0;
                        if (a != a || b != b) {
                            c = 0.0;
                        } else {
                            if(a > 50 || b > 50){
                                c = 0.0;
                            } else {
                                if (a < b) {
                                    c = a / b;
                                } else {
                                    c = b / a;
                                }
                                if(c >= min_c && c <= max_c) {
                                    if ((std::abs(1 - c) < cur_c || cur_c == 0.0) && area >= max_area) {
                                        max_area_idx = i;
                                        max_area = area;
                                        cur_c = std::abs(1 - c);
                                    }
                                } else {
                                    c = 0.0;
                                }
                            }
                        }
                    }
                }
            }
            if (max_area_idx >= 0) {
                result = cv::boundingRect(contours[max_area_idx]);
            }
        }
        return result;
    } else if(m_detector_type == 2) {
//        cv::Mat src = src_img.clone(); //TODO: Is it needed?
//        cv::cvtColor(src_img, src, CV_YUV2BGR);
        const cv::Mat preproc = m_ball_preprocessor.preprocess(src_img);
        return m_coloured_ball_detector.detect(preproc);
    }
    return cv::Rect();
}

int white_ball_detector_t::get_area_top() const {
    return m_area_top;
}

void white_ball_detector_t::set_area_top(int area_top) {
    m_area_top = area_top;
}

double white_ball_detector_t::get_area_low() const {
    return m_area_low;
}

void white_ball_detector_t::set_area_low(double area_low) {
    m_area_low = area_low;
}

int white_ball_detector_t::get_detector_type() const {
    return m_detector_type;
}

void white_ball_detector_t::set_detector_type(int detector_type) {
    m_detector_type = detector_type;
}

void white_ball_detector_t::set_cascade_config(std::string path) {
    m_path_to_cascade_config = std::move(path);
    m_ball_cascade.load(m_path_to_cascade_config);
}

const std::string& white_ball_detector_t::get_cascade_config() const {
    return m_path_to_cascade_config;
}

const cv::Scalar& white_ball_detector_t::get_threshold_gabor_bgr_min() const {
    return m_ball_preprocessor.get_threshold_color_bgr_min();
}

void white_ball_detector_t::set_threshold_gabor_bgr_min(const cv::Scalar& threshold_gabor_bgr_min) {
    m_ball_preprocessor.set_threshold_color_bgr_min(threshold_gabor_bgr_min);
}

const cv::Scalar& white_ball_detector_t::get_threshold_gabor_bgr_max() const {
    return m_ball_preprocessor.get_threshold_color_bgr_max();
}

void white_ball_detector_t::set_threshold_gabor_bgr_max(const cv::Scalar& threshold_gabor_bgr_max) {
    m_ball_preprocessor.set_threshold_gabor_bgr_max(threshold_gabor_bgr_max);
}

const cv::Scalar& white_ball_detector_t::get_threshold_color_bgr_min() const {
    return m_ball_preprocessor.get_threshold_color_bgr_min();
}

void white_ball_detector_t::set_threshold_color_bgr_min(const cv::Scalar& threshold_color_bgr_min) {
    m_ball_preprocessor.set_threshold_color_bgr_min(threshold_color_bgr_min);
}

const cv::Scalar& white_ball_detector_t::get_threshold_color_bgr_max() const {
    return m_ball_preprocessor.get_threshold_color_bgr_max();
}

void white_ball_detector_t::set_threshold_color_bgr_max(const cv::Scalar& threshold_color_bgr_max) {
    m_ball_preprocessor.set_threshold_gabor_bgr_max(threshold_color_bgr_max);
}

int white_ball_detector_t::get_median_blur_size() const {
    return m_ball_preprocessor.get_median_blur_size();
}

void white_ball_detector_t::set_median_blur_size(int median_blur_size) {
    m_ball_preprocessor.set_median_blur_size(median_blur_size);
}

const std::string& white_ball_detector_t::get_path_to_ann_config() const {
    return m_path_to_ann_config;
}

void white_ball_detector_t::set_path_to_ann_config(const std::string& path_to_ann_config) {
    m_path_to_ann_config = path_to_ann_config;
    m_network = cv::ml::ANN_MLP::load(path_to_ann_config);
    LOG_INFO << "Artificial neural network loaded";
}

bool white_ball_detector_t::is_network_enabled() const {
    return m_network_enabled;
}

void white_ball_detector_t::enable_network(bool enable) {
    m_network_enabled = enable;
}

const cv::Size& white_ball_detector_t::get_network_window() const {
    return m_network_window;
}

void white_ball_detector_t::set_network_window(const cv::Size& network_window) {
    m_network_window = network_window;
}
