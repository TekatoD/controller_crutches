/// \autor arssivka
/// \date 11/2/17

#include <cv.hpp>
#include <algorithm>
#include "vision/detectors/white_ball_detector_t.h"

using namespace drwn;

cv::Rect white_ball_detector_t::detect(const cv::Mat& prep_img, const std::vector<cv::Vec4i>& lines) const {
    cv::Mat preproc = prep_img.clone();//TODO MAYBE CLONE
    for (auto &&line : lines) {
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
            auto y = k * (double)x + m;
            for (int i = (((x - R) >= 0) ? x - R : 0); i < ((x + R < preproc.cols) ? x + R : preproc.cols); i++) {
                if (y != y) {
                    for (int j = (y0 < y1 ? y0 : y1); j < (y0 > y1 ? y0 : y1); j++) {
                        preproc.at<uchar>(j, i) = 0;
                    }
                } else {
                    for (int j = (((y - R) >= 0) ? y - R : 0); j < ((y + R < preproc.rows) ? y + R : preproc.rows); j++) {
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

        for (int i = 0; i < contours.size(); i++) {
            const double area = cv::contourArea(contours[i]);
            if (area > m_area_low && area < m_area_top && area > max_area) {
                cv::Mat a_matrix = cv::Mat::zeros(5, 5, CV_64F);
                cv::Mat b_vector = cv::Mat::zeros(5, 1, CV_64F);
                for (auto &&point : contours[i]) {
                    a_matrix.at<double>(0, 0) += std::pow((double) point.x, 4.0);
                    a_matrix.at<double>(0, 1) += std::pow((double) point.x, 2.0) * std::pow((double) point.y, 2.0);
                    a_matrix.at<double>(0, 2) += 2.0 * std::pow((double) point.x, 3.0) * point.y;
                    a_matrix.at<double>(0, 3) += 2.0 * std::pow((double) point.x, 3.0);
                    a_matrix.at<double>(0, 4) += 2.0 * std::pow((double) point.x, 2.0) * point.y;
                    //
                    a_matrix.at<double>(1, 0) += std::pow((double) point.x, 2.0) * std::pow((double) point.y, 2.0);
                    a_matrix.at<double>(1, 1) += std::pow((double) point.y, 4.0);
                    a_matrix.at<double>(1, 2) += 2.0 * point.x * std::pow((double) point.y, 3.0);
                    a_matrix.at<double>(1, 3) += 2.0 * point.x * std::pow((double) point.y, 2.0);
                    a_matrix.at<double>(1, 4) += 2.0 * std::pow((double) point.y, 3.0);
                    //
                    a_matrix.at<double>(2, 0) += std::pow((long double) point.x, 3.0) * (long double) point.y;
                    a_matrix.at<double>(2, 1) += point.x * std::pow((double) point.y, 3.0);
                    a_matrix.at<double>(2, 2) += 2.0 * std::pow((double) point.x, 2.0) * std::pow((double) point.y, 2.0);
                    a_matrix.at<double>(2, 3) += 2.0 * std::pow((double) point.x, 2.0) * point.y;
                    a_matrix.at<double>(2, 4) += 2.0 * point.x * std::pow((double) point.y, 2.0);
                    //
                    a_matrix.at<double>(3, 0) += std::pow((double) point.x, 3.0);
                    a_matrix.at<double>(3, 1) += point.x * std::pow((double) point.y, 2.0);
                    a_matrix.at<double>(3, 2) += 2.0 * point.x * std::pow((double) point.y, 2.0);
                    a_matrix.at<double>(3, 3) += 2.0 * std::pow((double) point.x, 2.0);
                    a_matrix.at<double>(3, 4) += 2.0 * point.x * point.y;
                    //
                    a_matrix.at<double>(4, 0) += std::pow((double) point.x, 2.0) * point.y;
                    a_matrix.at<double>(4, 1) += std::pow((double) point.y, 3.0);
                    a_matrix.at<double>(4, 2) += 2.0 * point.x * std::pow((double) point.y, 2.0);
                    a_matrix.at<double>(4, 3) += 2.0 * point.x * point.y;
                    a_matrix.at<double>(4, 4) += 2.0 * std::pow((double) point.y, 2.0);
                    //
                    b_vector.at<double>(0) += std::pow((double) point.x, 2.0);
                    b_vector.at<double>(1) += std::pow((double) point.y, 2.0);
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
                    if(std::abs(lam.at<double>(0)) < 0.0001){
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
