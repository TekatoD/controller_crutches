// Created by Andrey Potemkin

#include <vision/detectors/AngleDetector.h>
//#include <rrc/core.h>


namespace drwn {
  AngleDetector::AngleDetector() : BaseDetector("AngleDetector") {};

  std::vector<cv::Vec3d> AngleDetector::Detect(cv::Mat& img, const std::vector<cv::Vec4i>& lines) {
//    std::vector<std::vector<cv::Vec3d>> angles(lines.size(), std::vector<cv::Vec3d>(lines.size(), cv::Vec3d(0, 0, 0)));
    std::vector<cv::Vec3d> angles(0);
    for (size_t i = 0; i < lines.size(); i++) {
      for (size_t j = i + 1; j < lines.size(); j++) {
        double ki, mi, kj, mj;
        std::tie(ki, mi) = GetLineParams(lines[i]);
        std::tie(kj, mj) = GetLineParams(lines[j]);
        double x = (mj - mi) / (ki - kj);
        double y = ki * x + mi;
        //
        if (IsOnLine(lines[i], x, y) && ( sqrt(pow(x-lines[j](0),2.)+pow(y-lines[j](1),2.))< 40  ||
                                          sqrt(pow(x-lines[j](2),2.)+pow(y-lines[j](3),2.)) < 40 ||
                IsOnLine(lines[j], x, y))
            || IsOnLine(lines[j], x, y) && ( sqrt(pow(x-lines[i](0),2.)+pow(y-lines[i](1),2.))< 40  ||
                                             sqrt(pow(x-lines[i](2),2.)+pow(y-lines[i](3),2.)) < 40 ||
                IsOnLine(lines[i], x, y))) {
          double xi = (lines[i](2) - lines[i](0));
          double xj = (lines[j](2) - lines[j](0));
          double yi = (lines[i](3) - lines[i](1));
          double yj = (lines[j](3) - lines[j](1));
          double angle = acos((xi * xj + yi * yj) / (sqrt(xi * xi + yi * yi) * sqrt(xj * xj + yj * yj)));

          if (angle > 0.15 && angle < 2. * boost::math::constants::pi<float>() - 0.15) {
              angle = ((angle > 2. * boost::math::constants::pi<float>() - angle)
                       ? 2. * boost::math::constants::pi<float>() - angle
                       : angle);
              angles.push_back(cv::Vec3d(x, y, angle));
          }
        }
      }
    }

    return angles;
  }

  void AngleDetector::load(const boost::property_tree::ptree &config) {

  }


  boost::property_tree::ptree AngleDetector::get_params() {
    return boost::property_tree::ptree();
  }


  std::pair<double, double> AngleDetector::GetLineParams(const cv::Vec4i& line) {
    double x0 = line(0);
    double y0 = line(1);
    double x1 = line(2);
    double y1 = line(3);
    double k = (y1 - y0) / (x1 - x0);
    double m = y0 - k * x0;
    return std::make_pair(k, m);
  }

  bool AngleDetector::IsOnLine(const cv::Vec4i& line, double x, double y) {
    double x0 = line(0);
    double y0 = line(1);
    double x1 = line(2);
    double y1 = line(3);
    if (x0 > x1) {
      std::swap(x0, x1);
    }
    if (y0 > y1) {
      std::swap(y0, y1);
    }
    if (sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0)) < 15. ||
        sqrt((x - x1) * (x - x1) + (y - y1) * (y - y1)) < 15.) {
      return true;
    }
    return x0 <= x && x <= x1 && y0 <= y && y <= y1;
  }

}

