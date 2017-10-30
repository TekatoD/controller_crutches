//
// Created by nikitas on 4/1/16.
//


#include <vision/detectors/BallDetector.h>
#include <array>


namespace Robot {

  BallDetector::BallDetector() : BaseDetector("BallDetector") {}

  cv::Rect BallDetector::DetectOld(const cv::Mat& preprocImage) {
    cv::Rect ans;
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(preprocImage, contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    if (!contours.empty()) {
      double maxArea = 0;
      int maxAreaIdx = 0;

      for (int i = 0; i < contours.size(); i++) {
        const double area = cv::contourArea(contours[i]);
        if (area > maxArea) maxArea = area, maxAreaIdx = i;
      }

      ans = cv::boundingRect(contours[maxAreaIdx]);
    }
    return ans;
  }

  cv::Rect BallDetector::Detect(const cv::Mat& preprocImage, const std::vector<cv::Vec4i>& lines) {
    if(!m_conf.white_ball){
      return DetectOld(preprocImage);
    }
    cv::Mat preproc = preprocImage;
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
      auto R = 4;
      for (auto x = x0; x <= x1; x++) {
        auto y = k * x + m;
        for (auto i = (((x - R) >= 0) ? x - R : 0); i < ((x + R < preproc.cols) ? x + R : preproc.cols); i++) {
          for (auto j = (((y - R) >= 0) ? y - R : 0); j < ((y + R < preproc.rows) ? y + R : preproc.rows); j++) {
            preproc.at<uchar>(j, i) = 0;
          }
        }
      }
    }
    cv::Rect ans;
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(preproc, contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    cv::Mat cont_mat = cv::Mat::zeros(preproc.size(), preproc.type());
    double max_c = 2.0;
    double min_c = 0.5;
    double cur_c = 0.0;

    if (!contours.empty()) {
      double maxArea = 0;
      int maxAreaIdx = -1;

      for (int i = 0; i < contours.size(); i++) {
        cv::drawContours(cont_mat, contours, i, cv::Scalar(255));
        const double area = cv::contourArea(contours[i]);
        if (area > 18.0 && area < 1000 && area > maxArea) {
          cv::Mat A_matrix = cv::Mat::zeros(5, 5, CV_64F);
          cv::Mat B_vector = cv::Mat::zeros(5, 1, CV_64F);
          for (auto &&point : contours[i]) {
            A_matrix.at<double>(0, 0) += std::pow((double) point.x, 4.0);
            A_matrix.at<double>(0, 1) += std::pow((double) point.x, 2.0) * std::pow((double) point.y, 2.0);
            A_matrix.at<double>(0, 2) += 2.0 * std::pow((double) point.x, 3.0) * point.y;
            A_matrix.at<double>(0, 3) += 2.0 * std::pow((double) point.x, 3.0);
            A_matrix.at<double>(0, 4) += 2.0 * std::pow((double) point.x, 2.0) * point.y;
            //
            A_matrix.at<double>(1, 0) += std::pow((double) point.x, 2.0) * std::pow((double) point.y, 2.0);
            A_matrix.at<double>(1, 1) += std::pow((double) point.y, 4.0);
            A_matrix.at<double>(1, 2) += 2.0 * point.x * std::pow((double) point.y, 3.0);
            A_matrix.at<double>(1, 3) += 2.0 * point.x * std::pow((double) point.y, 2.0);
            A_matrix.at<double>(1, 4) += 2.0 * std::pow((double) point.y, 3.0);
            //
            A_matrix.at<double>(2, 0) += std::pow((long double) point.x, 3.0) * (long double) point.y;
            A_matrix.at<double>(2, 1) += point.x * std::pow((double) point.y, 3.0);
            A_matrix.at<double>(2, 2) += 2.0 * std::pow((double) point.x, 2.0) * std::pow((double) point.y, 2.0);
            A_matrix.at<double>(2, 3) += 2.0 * std::pow((double) point.x, 2.0) * point.y;
            A_matrix.at<double>(2, 4) += 2.0 * point.x * std::pow((double) point.y, 2.0);
            //
            A_matrix.at<double>(3, 0) += std::pow((double) point.x, 3.0);
            A_matrix.at<double>(3, 1) += point.x * std::pow((double) point.y, 2.0);
            A_matrix.at<double>(3, 2) += 2.0 * point.x * std::pow((double) point.y, 2.0);
            A_matrix.at<double>(3, 3) += 2.0 * std::pow((double) point.x, 2.0);
            A_matrix.at<double>(3, 4) += 2.0 * point.x * point.y;
            //
            A_matrix.at<double>(4, 0) += std::pow((double) point.x, 2.0) * point.y;
            A_matrix.at<double>(4, 1) += std::pow((double) point.y, 3.0);
            A_matrix.at<double>(4, 2) += 2.0 * point.x * std::pow((double) point.y, 2.0);
            A_matrix.at<double>(4, 3) += 2.0 * point.x * point.y;
            A_matrix.at<double>(4, 4) += 2.0 * std::pow((double) point.y, 2.0);
            //
            B_vector.at<double>(0) += std::pow((double) point.x, 2.0);
            B_vector.at<double>(1) += std::pow((double) point.y, 2.0);
            B_vector.at<double>(2) += (double) point.x * (double) point.y;
            B_vector.at<double>(3) += point.x;
            B_vector.at<double>(4) += point.y;
          }
          cv::Mat inverse_A_matrix = A_matrix.inv();
          cv::Mat params = inverse_A_matrix * B_vector;
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
          cv::Mat D_matrix = cv::Mat::zeros(2, 2, CV_64F);
          D_matrix.at<double>(0, 0) = a11;
          D_matrix.at<double>(0, 1) = a12;
          D_matrix.at<double>(1, 0) = a12;
          D_matrix.at<double>(1, 1) = a22;
          cv::Mat lam;
          cv::eigen(D_matrix, lam);
          if (!lam.empty()) {
            double delta_det = cv::determinant(delta);
            double d_det = cv::determinant(D_matrix);
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
                  if ((std::abs(1 - c) < cur_c || cur_c == 0.0) && area >= maxArea) {
                    maxAreaIdx = i;
                    maxArea = area;
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
      if (maxAreaIdx >= 0) {
        ans = cv::boundingRect(contours[maxAreaIdx]);
      }
    }
    return ans;
  }

  cv::Mat BallDetector::Preproccess(const cv::Mat& image) {
    cv::Mat preprocImage, prepImage;
    cv::cvtColor(image, preprocImage, CV_YUV2BGR);
    cv::Mat medianBlurFrame;
    cv::medianBlur(preprocImage, medianBlurFrame, m_conf.median_blur_size);
    cv::Mat afterGaborRange, gaborImage;
    cv::Mat kernel = cv::getGaborKernel(cv::Size(3, 3), 10, 7.8, 9.4, 0);
    cv::filter2D(medianBlurFrame, gaborImage, -1, kernel);


    const cv::Scalar minGaborColor(m_conf.GaborThresh.min_1,
                                   m_conf.GaborThresh.min_2,
                                   m_conf.GaborThresh.min_3);
    const cv::Scalar maxGaborColor(m_conf.GaborThresh.max_1,
                                   m_conf.GaborThresh.max_2,
                                   m_conf.GaborThresh.max_3);
    cv::Mat gaborInRange;
    cv::inRange(gaborImage, minGaborColor, maxGaborColor, gaborInRange);

//        afterGaborRange = gaborInRange;
    cv::morphologyEx(gaborInRange, afterGaborRange, CV_MOP_DILATE, cv::Mat::ones(3, 3, CV_8UC1));


    const cv::Scalar minColor(m_conf.ColorThresh.min_1,
                              m_conf.ColorThresh.min_2,
                              m_conf.ColorThresh.min_3);
    const cv::Scalar maxColor(m_conf.ColorThresh.max_1,
                              m_conf.ColorThresh.max_2,
                              m_conf.ColorThresh.max_3);

    cv::inRange(medianBlurFrame, minColor, maxColor, preprocImage);
    cv::morphologyEx(preprocImage, preprocImage, CV_MOP_DILATE, cv::Mat::ones(3, 3, CV_8UC1));

    cv::Mat ball = preprocImage.clone();
    for (int r = 0; r < ball.rows; r++) {
      for (int c = 0; c < ball.cols; c++) {
        if (afterGaborRange.at<uchar>(r, c) != 0) {
          cv::floodFill(ball, cv::Point(c, r), 0);
        }
      }
    }
    return preprocImage - ball;
  }

  void BallDetector::load(const boost::property_tree::ptree &config) {
    const boost::property_tree::ptree ball_config = config.get_child(DetectorName());
    m_conf.median_blur_size = ball_config.get<int>("median_blur_size");
    m_conf.white_ball = ball_config.get<bool>("white_ball");
    m_conf.ColorThresh.min_1 = ball_config.get<uchar>("ColorThresh.min_1");
    m_conf.ColorThresh.min_2 = ball_config.get<uchar>("ColorThresh.min_2");
    m_conf.ColorThresh.min_3 = ball_config.get<uchar>("ColorThresh.min_3");
    m_conf.ColorThresh.max_1 = ball_config.get<uchar>("ColorThresh.max_1");
    m_conf.ColorThresh.max_2 = ball_config.get<uchar>("ColorThresh.max_2");
    m_conf.ColorThresh.max_3 = ball_config.get<uchar>("ColorThresh.max_3");
    m_conf.GaborThresh.min_1 = ball_config.get<uchar>("GaborThresh.min_1");
    m_conf.GaborThresh.min_2 = ball_config.get<uchar>("GaborThresh.min_2");
    m_conf.GaborThresh.min_3 = ball_config.get<uchar>("GaborThresh.min_3");
    m_conf.GaborThresh.max_1 = ball_config.get<uchar>("GaborThresh.max_1");
    m_conf.GaborThresh.max_2 = ball_config.get<uchar>("GaborThresh.max_2");
    m_conf.GaborThresh.max_3 = ball_config.get<uchar>("GaborThresh.max_3");
  }


  boost::property_tree::ptree BallDetector::get_params() {
    boost::property_tree::ptree ball_config, ptree;

    ball_config.put("median_blur_size", m_conf.median_blur_size);
    ball_config.put("ColorThresh.min_1", m_conf.ColorThresh.min_1);
    ball_config.put("ColorThresh.min_2", m_conf.ColorThresh.min_2);
    ball_config.put("ColorThresh.min_3", m_conf.ColorThresh.min_3);
    ball_config.put("ColorThresh.max_1", m_conf.ColorThresh.max_1);
    ball_config.put("ColorThresh.max_2", m_conf.ColorThresh.max_2);
    ball_config.put("ColorThresh.max_3", m_conf.ColorThresh.max_3);
    ball_config.put("GaborThresh.min_1", m_conf.GaborThresh.min_1);
    ball_config.put("GaborThresh.min_2", m_conf.GaborThresh.min_2);
    ball_config.put("GaborThresh.min_3", m_conf.GaborThresh.min_3);
    ball_config.put("GaborThresh.max_1", m_conf.GaborThresh.max_1);
    ball_config.put("GaborThresh.max_2", m_conf.GaborThresh.max_2);
    ball_config.put("GaborThresh.max_3", m_conf.GaborThresh.max_3);

    ptree.put_child(DetectorName(), ball_config);
    return ptree;
  }

  bool BallDetector::IsWhite() const noexcept {
    return m_conf.white_ball;
  }


}