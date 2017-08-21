//
// Created by nikitas on 26.03.16.
//

#include "detectors/LineDetector.h"
#include "VisionUtils.h"

const char table[]{
  0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 0, 0, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0,
  0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  1, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  1, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0
};

int fillNeighbours(const cv::Mat &window, bool *neighbours) {
  const int indexes[][2]{
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

//  const uchar *data = window.ptr(0);
  for (int i = 0; i < sizeof(indexes) / sizeof(*indexes); ++i) {
    const int *idx = indexes[i];
    neighbours[i] = window.at<uchar>(idx[0], idx[1]);
    table_idx = table_idx << 1 | neighbours[i];
  }
  return table_idx;
}


namespace ant {

  LineDetector::LineDetector() : BaseDetector("LineDetector") {}

  std::vector<cv::Vec4i> LineDetector::detect(const cv::Mat &preprocImage) {
    cv::Mat skeleton;
//
    __get_skeleton(preprocImage, skeleton); // O(n) n = h*w of img
//
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(skeleton, lines,
                    m_conf.HoughLines.rho, m_conf.HoughLines.theta, m_conf.HoughLines.threshold,
                    m_conf.HoughLines.min_line_length, m_conf.HoughLines.max_line_gap);

    __join_lines(lines); // O(m^2) - m = countLines

    return lines;
  }

  std::vector<cv::Vec4i> LineDetector::detect_old(const cv::Mat &preprocImage) {
    cv::Mat skeleton;

    __get_skeleton(preprocImage, skeleton); // O(n) n = h*w of img
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(skeleton, lines,
                    m_conf.HoughLines.rho, m_conf.HoughLines.theta, m_conf.HoughLines.threshold,
                    m_conf.HoughLines.min_line_length, m_conf.HoughLines.max_line_gap);

    __join_lines(lines); // O(m^2) - m = countLines

    return lines;
  }


  cv::Mat LineDetector::preproccess(const cv::Mat &image) {
    cv::Mat hsv_img, gray_img, bgr_img, buffer;

    cv::cvtColor(image, bgr_img, CV_YUV2BGR);
    cv::GaussianBlur(bgr_img,bgr_img,cv::Size(5,5),1.5,1.5);
    cv::cvtColor(bgr_img, gray_img, CV_BGR2GRAY);
    cv::cvtColor(bgr_img, hsv_img, CV_BGR2HSV);

    const cv::Mat kernel = cv::getGaborKernel(cv::Size(3, 3), 10.0, 4.1, M_PI, 0.0);
    cv::Mat filtred_image, thresh_filt_image;
    cv::filter2D(bgr_img, filtred_image, -1, kernel);
    cv::cvtColor(filtred_image, buffer, CV_BGR2Lab);
    const cv::Scalar min_color(m_conf.Preproc_new.ColorThresh.min_1,
                               m_conf.Preproc_new.ColorThresh.min_2,
                               m_conf.Preproc_new.ColorThresh.min_3);
    const cv::Scalar max_color(m_conf.Preproc_new.ColorThresh.max_1,
                               m_conf.Preproc_new.ColorThresh.max_2,
                               m_conf.Preproc_new.ColorThresh.max_3);
    cv::inRange(buffer, min_color, max_color, thresh_filt_image);
    const cv::Scalar min_color2(m_conf.Preproc_new.ColorThresh2.min_1,
                                m_conf.Preproc_new.ColorThresh2.min_2,
                                m_conf.Preproc_new.ColorThresh2.min_3);
    const cv::Scalar max_color2(m_conf.Preproc_new.ColorThresh2.max_1,
                                m_conf.Preproc_new.ColorThresh2.max_2,
                                m_conf.Preproc_new.ColorThresh2.max_3);
    cv::Mat thresh_filt_image2;
    cv::inRange(hsv_img, min_color2, max_color2, thresh_filt_image2);
    thresh_filt_image2.copyTo(thresh_filt_image, thresh_filt_image2);
//        cv::imshow("inrange_hsv",thresh_filt_image);
    return thresh_filt_image;
    cv::adaptiveThreshold(gray_img, gray_img, 255,
                          CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY,
                          m_conf.Preproc.kernel_size, 0.0);

    cv::Mat colorImg = gray_img.clone();

    for (int r = 0; r < colorImg.rows; r++) {
      for (int c = 0; c < colorImg.cols; c++) {
        if (thresh_filt_image.at<uchar>(r, c) != 0) {
          cv::floodFill(colorImg, cv::Point(c, r), 0);
          cv::floodFill(thresh_filt_image, cv::Point(c, r), 0);
        }
      }
    }

    const cv::Mat result_matrix = gray_img - colorImg;

    return result_matrix;
  }

  cv::Mat LineDetector::preproccess_old(const cv::Mat &image) {
    cv::Mat hsv_img, gray_img, bgr_img, buffer;

    cv::cvtColor(image, bgr_img, CV_YUV2BGR);
    cv::cvtColor(bgr_img, gray_img, CV_BGR2GRAY);
    cv::cvtColor(bgr_img, hsv_img, CV_BGR2HSV);

    const cv::Mat kernel = cv::getGaborKernel(cv::Size(3, 3), 10.0, 4.1, M_PI, 0.0);
    cv::Mat filtred_image, thresh_filt_image;
    cv::filter2D(bgr_img, filtred_image, -1, kernel);
    cv::cvtColor(filtred_image, buffer, CV_BGR2Lab);
    const cv::Scalar min_color(m_conf.Preproc.ColorThresh.min_1,
                               m_conf.Preproc.ColorThresh.min_2,
                               m_conf.Preproc.ColorThresh.min_3);
    const cv::Scalar max_color(m_conf.Preproc.ColorThresh.max_1,
                               m_conf.Preproc.ColorThresh.max_2,
                               m_conf.Preproc.ColorThresh.max_3);
    cv::inRange(buffer, min_color, max_color, thresh_filt_image);
    cv::adaptiveThreshold(gray_img, gray_img, 255,
                          CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY,
                          m_conf.Preproc.kernel_size, 0.0);

    cv::Mat colorImg = gray_img.clone();

    for (int r = 0; r < colorImg.rows; r++) {
      for (int c = 0; c < colorImg.cols; c++) {
        if (thresh_filt_image.at<uchar>(r, c) != 0) {
          cv::floodFill(colorImg, cv::Point(c, r), 0);
          cv::floodFill(thresh_filt_image, cv::Point(c, r), 0);
        }
      }
    }

    const cv::Mat result_matrix = gray_img - colorImg;

    return result_matrix;
  }


  void LineDetector::__join_lines(std::vector<cv::Vec4i> &lines) {
    std::vector<int> clusters;

    const int num_clusters = cv::partition(lines, clusters, m_conf.LineEqualPredicate);

    std::vector<cv::Vec4i> joinLines((std::size_t) num_clusters);

    for (std::size_t i = 0; i < lines.size(); ++i)
      joinLines[clusters[i]] += lines[i];

    lines = joinLines;
  }

  void LineDetector::__get_skeleton(const cv::Mat &img, cv::Mat &result) {
    __zhang_suen(img, result);


  }

  void LineDetector::__simple_skeleton(const cv::Mat &img, cv::Mat &result) {
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

  void LineDetector::__zhang_suen(const cv::Mat &img, cv::Mat &result) {
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

          int idx = fillNeighbours(result(window), neighbours);

          if (table[idx]) points[count++] = cv::Point(c, r);
        }
      }

      for (int i = 0; i < count; ++i)
        result.at<uchar>(points[i]) = 0;
      inter++;
    } while (count != 0);

//    std::cout << inter << std::endl;
  }

  bool LineDetector::configuration::LineEqualPredicate::operator()
    (const cv::Vec4i &line1, const cv::Vec4i &line2) {
    using namespace vision_utils;

    const cv::Point a(line1(0), line1(1));
    const cv::Point b(line1(2), line1(3));
    const cv::Point c(line2(2), line2(3));

    const bool parallel = getAngle(getVector<float>(line1),
                                   getVector<float>(line2)) < angle_eps;
    const bool pointInLineC = std::abs(getAltitude(c, a, b)) < error_px;

    return parallel && pointInLineC;
  }

  void LineDetector::load(minIni* ini)
  {
      std::cout << "Loading LineDetector ini" << std::endl;
      m_conf.HoughLines.max_line_gap = ini->getf("HoughLines", "max_line_gap");
      m_conf.HoughLines.min_line_length = ini->getf("HoughLines", "min_line_length");
      m_conf.HoughLines.rho = ini->getf("HoughLines", "rho");
      m_conf.HoughLines.theta = ini->getf("HoughLines", "theta");
      m_conf.HoughLines.threshold = ini->geti("HoughLines", "threshold");
      
      m_conf.LineEqualPredicate.angle_eps = ini->getf("LineEqualpredicate", "angle_eps");
      m_conf.LineEqualPredicate.error_px = ini->geti("LineEqualPredicate", "error_px");
      
      m_conf.Preproc.kernel_size = ini->geti("Preproc", "kernel_size");
      m_conf.Preproc.min_thresh = ini->geti("Preproc", "min_thresh");
      m_conf.Preproc.ColorThresh.min_1 = ini->geti("ColorThresh", "min_1");
      m_conf.Preproc.ColorThresh.min_2 = ini->geti("ColorThresh", "min_2");
      m_conf.Preproc.ColorThresh.min_3 = ini->geti("ColorThresh", "min_3");
      m_conf.Preproc.ColorThresh.max_1 = ini->geti("ColorThresh", "max_1");
      m_conf.Preproc.ColorThresh.max_2 = ini->geti("ColorThresh", "max_2");
      m_conf.Preproc.ColorThresh.max_3 = ini->geti("ColorThresh", "max_3");
      
      m_conf.Preproc_new.kernel_size = ini->geti("Preproc_new", "kernel_size");
      m_conf.Preproc_new.min_thresh = ini->geti("Preproc_new", "min_thresh");
      m_conf.Preproc_new.ColorThresh.min_1 = ini->geti("ColorThresh1", "min_1");
      m_conf.Preproc_new.ColorThresh.min_2 = ini->geti("ColorThresh1", "min_2");
      m_conf.Preproc_new.ColorThresh.min_3 = ini->geti("ColorThresh1", "min_3");
      m_conf.Preproc_new.ColorThresh.max_1 = ini->geti("ColorThresh1", "max_1");
      m_conf.Preproc_new.ColorThresh.max_2 = ini->geti("ColorThresh1", "max_2");
      m_conf.Preproc_new.ColorThresh.max_3 = ini->geti("ColorThresh1", "max_3");
      m_conf.Preproc_new.ColorThresh2.min_1 = ini->geti("ColorThresh2", "min_1");
      m_conf.Preproc_new.ColorThresh2.min_2 = ini->geti("ColorThresh2", "min_2");
      m_conf.Preproc_new.ColorThresh2.min_3 = ini->geti("ColorThresh2", "min_3");
      m_conf.Preproc_new.ColorThresh2.max_1 = ini->geti("ColorThresh2", "max_1");
      m_conf.Preproc_new.ColorThresh2.max_2 = ini->geti("ColorThresh2", "max_2");
      m_conf.Preproc_new.ColorThresh2.max_3 = ini->geti("ColorThresh2", "max_3");
  }
}
