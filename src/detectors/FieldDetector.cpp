//
// Created by pav on 25/01/2017.
//

#include "detectors/FieldDetector.h"

ant::FieldDetector::FieldDetector() : BaseDetector("FieldDetector"){

}

cv::Mat ant::FieldDetector::preproccess(const cv::Mat &image) {
  cv::Mat preprocImage, prepImage, hsvImg;
  image.copyTo(preprocImage);
  cv::cvtColor(preprocImage,hsvImg,cv::COLOR_BGR2HSV);
  cv::Mat medianBlurFrame;
  cv::medianBlur(preprocImage, medianBlurFrame, 3);
  cv::Mat afterGaborRange, gaborImage;
  cv::Mat kernel = cv::getGaborKernel(cv::Size(3, 3), 10, 7.8, 9.4, 0);
  cv::filter2D(medianBlurFrame, gaborImage, -1, kernel);
  const cv::Scalar minGaborColor(ColorThresh.min_1,
                                 ColorThresh.min_2,
                                 ColorThresh.min_3);
  const cv::Scalar maxGaborColor(ColorThresh.max_1,
                                 ColorThresh.max_2,
                                 ColorThresh.max_3);
  cv::Mat gaborInRange;
  cv::inRange(gaborImage, minGaborColor, maxGaborColor, gaborInRange);
  cv::morphologyEx(gaborInRange, afterGaborRange, CV_MOP_DILATE, cv::Mat::ones(3, 3, CV_8UC1));
  const cv::Scalar minColor(ColorThresh2.min_1,
                            ColorThresh2.min_2,
                            ColorThresh2.min_3);
  const cv::Scalar maxColor(ColorThresh2.max_1,
                            ColorThresh2.max_2,
                            ColorThresh2.max_3);
//
  cv::inRange(hsvImg, minColor, maxColor, preprocImage);
  cv::morphologyEx(preprocImage, preprocImage, CV_MOP_DILATE, cv::Mat::ones(7, 7, CV_8UC1));

  cv::Mat field = preprocImage.clone();
  for (int r = 0; r < field.rows; r++) {
    for (int c = 0; c < field.cols; c++) {
      if (afterGaborRange.at<uchar>(r, c) != 0) {
        cv::floodFill(field, cv::Point(c, r), 0);
      }
    }
  }
  
  return preprocImage + field;
}

cv::Mat ant::FieldDetector::detect(const cv::Mat &preprocImage) {
  cv::Rect ans;
  cv::Mat temp_image=cv::Mat::zeros(preprocImage.rows,preprocImage.cols,CV_8UC3);
  std::vector<std::vector<cv::Point> > contours;
  cv::findContours(preprocImage, contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
  if (!contours.empty()) {
    double maxArea = 0;
    int maxAreaIdx = 0;

    for (int i = 0; i < contours.size(); i++) {
      const double area = cv::contourArea(contours[i]);
      if (area > maxArea) maxArea = area, maxAreaIdx = i;
    }
    for(size_t i = 1; i < contours[maxAreaIdx].size(); ++i){
      cv::line(temp_image, contours[maxAreaIdx][i-1], contours[maxAreaIdx][i], 255);
    }
    cv::line(temp_image, contours[maxAreaIdx][0], contours[maxAreaIdx][contours[maxAreaIdx].size()-1], 255);
    cv::Scalar color(1,1,1);
    cv::drawContours( temp_image, contours, maxAreaIdx, color, CV_FILLED, 8 );

    return temp_image;
  }

  return cv::Mat::zeros(preprocImage.rows,preprocImage.cols,CV_8U);
}

void ant::FieldDetector::load(minIni* ini)
{
    int defKernelSize = 32;
    int defMinThresh = 0, defMaxThresh = 255;
    int val = -1;
    
    val = ini->geti("FieldDetector", "kernel_size", defKernelSize);
    //set kernel size
    
    val = ini->geti("FieldDetector", "min_threshold", defMinThresh);
    // set min thresh
    
    ColorThresh.min_1 = ini->geti("ColorThresh1", "min_1", defMinThresh);
    ColorThresh.min_2 = ini->geti("ColorThresh1", "min_2", defMinThresh);
    ColorThresh.min_3 = ini->geti("ColorThresh1", "min_3", defMinThresh);
    ColorThresh.max_1 = ini->geti("ColorThresh1", "max_1", defMinThresh);
    ColorThresh.max_2 = ini->geti("ColorThresh1", "max_2", defMinThresh);
    ColorThresh.max_3 = ini->geti("ColorThresh1", "max_3", defMinThresh);
    ColorThresh2.min_1 = ini->geti("ColorThresh2", "min_1", defMinThresh);
    ColorThresh2.min_2 = ini->geti("ColorThresh2", "min_2", defMinThresh);
    ColorThresh2.min_3 = ini->geti("ColorThresh2", "min_3", defMinThresh);
    ColorThresh2.max_1 = ini->geti("ColorThresh2", "max_1", defMinThresh);
    ColorThresh2.max_2 = ini->geti("ColorThresh2", "max_2", defMinThresh);
    ColorThresh2.max_3 = ini->geti("ColorThresh2", "max_3", defMinThresh);
}

void ant::FieldDetector::load(const boost::property_tree::ptree &config) {
  const boost::property_tree::ptree line_config = config.get_child(detectorName());
  ColorThresh.min_1 = line_config.get<uchar>("ColorThresh1.min_1");
  ColorThresh.min_2 = line_config.get<uchar>("ColorThresh1.min_2");
  ColorThresh.min_3 = line_config.get<uchar>("ColorThresh1.min_3");
  ColorThresh.max_1 = line_config.get<uchar>("ColorThresh1.max_1");
  ColorThresh.max_2 = line_config.get<uchar>("ColorThresh1.max_2");
  ColorThresh.max_3 = line_config.get<uchar>("ColorThresh1.max_3");
  ColorThresh2.min_1 = line_config.get<uchar>("ColorThresh2.min_1");
  ColorThresh2.min_2 = line_config.get<uchar>("ColorThresh2.min_2");
  ColorThresh2.min_3 = line_config.get<uchar>("ColorThresh2.min_3");
  ColorThresh2.max_1 = line_config.get<uchar>("ColorThresh2.max_1");
  ColorThresh2.max_2 = line_config.get<uchar>("ColorThresh2.max_2");
  ColorThresh2.max_3 = line_config.get<uchar>("ColorThresh2.max_3");
}

boost::property_tree::ptree ant::FieldDetector::get_params() {
  boost::property_tree::ptree line_config, ptree;
  line_config.put("ColorThresh1.min_2", ColorThresh.min_2);
  line_config.put("ColorThresh1.min_1", ColorThresh.min_1);
  line_config.put("ColorThresh1.min_3", ColorThresh.min_3);
  line_config.put("ColorThresh1.max_1", ColorThresh.max_1);
  line_config.put("ColorThresh1.max_2", ColorThresh.max_2);
  line_config.put("ColorThresh1.max_3", ColorThresh.max_3);

  line_config.put("ColorThresh2.min_1", ColorThresh2.min_1);
  line_config.put("ColorThresh2.min_2", ColorThresh2.min_2);
  line_config.put("ColorThresh2.min_3", ColorThresh2.min_3);
  line_config.put("ColorThresh2.max_1", ColorThresh2.max_1);
  line_config.put("ColorThresh2.max_2", ColorThresh2.max_2);
  line_config.put("ColorThresh2.max_3", ColorThresh2.max_3);
  ptree.put_child(detectorName(), line_config);
  return ptree;
}
