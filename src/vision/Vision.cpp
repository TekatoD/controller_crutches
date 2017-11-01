//
// Created by nikitas on 26.03.16.
//

#include "vision/Vision.h"
#include <boost/exception/all.hpp>
//#include <lines_generated.h>
//#include <vec3_generated.h>
//#include <rect_generated.h>

using namespace drwn;


Vision::Vision(std::string prop_file) {
    UpdateProperties();
}

cv::Rect Vision::BallDetect() {
    FieldDetect();
    LineDetect();
    if (m_ballDetector.IsWhite()) {
        return m_ballDetector.Detect(m_preprocImage, m_lines);
    } else {
        return BallDetectOld();
    }
}

cv::Rect Vision::BallDetectOld() {
    FieldDetect();
    const cv::Mat preprocImage = m_ballDetector.Preproccess(m_image);
    //        cv::imshow("preproc_ball",preprocImage);
    return m_ballDetector.DetectOld(preprocImage);
}

std::vector<cv::Vec4i> Vision::LineDetect() {
    FieldDetect();
    if (m_lines.empty()) {
        m_preprocImage = m_lineDetector.Preproccess(m_image);
        m_lines = m_lineDetector.Detect(m_preprocImage);
    }
    return m_lines;
}

std::vector<cv::Vec4i> Vision::LineDetectOld() {
    const cv::Mat preprocImage = m_lineDetector.PreproccessOld(m_src_image);


    return m_lineDetector.DetectOld(preprocImage);
}

std::vector<cv::Vec3d> Vision::AngleDetect() {
//        if(m_lines.empty()){
//          lineDetect();
//        }
//        return m_angleDetector.detect(m_image,m_lines);
    if (m_lines.empty()) {
        LineDetect();
    }
    return m_angleDetector.Detect(m_image, m_lines);
}

cv::Mat Vision::FieldDetect() {
    if (m_mask.empty()) {
        cv::Mat prep = m_fieldDetector.Preproccess(m_image);
        m_mask = m_fieldDetector.Detect(prep);
        cv::Mat tmp;
        cvtColor(m_image, m_src_image, CV_BGR2YUV);
        m_image.copyTo(tmp, m_mask);
        m_image = tmp;
        cv::cvtColor(m_image, m_image, CV_BGR2YUV);
    }
    return m_mask;
}

void Vision::SetFrame(const cv::Mat& frame) {
    m_image = frame.clone();
    m_lines.clear();
    m_mask.release();
    m_mask = cv::Mat();
}

void Vision::SetFrame(cv::Mat&& frame) {
    m_image = frame;
    frame.release();
    m_lines.clear();
    m_mask = cv::Mat();
}

void Vision::UpdateProperties() {
    // Members from old module class
    std::string m_prop_file;
    bool m_enabled;

    boost::property_tree::ptree properties;
    try {
        boost::property_tree::json_parser::read_json(m_prop_file, properties);
        const boost::property_tree::ptree vision_config = properties.get_child("Vision");
        m_enabled = properties.get<bool>("enabled", true);
        m_lineDetector.load(vision_config);
        m_ballDetector.load(vision_config);
        m_angleDetector.load(vision_config);
        m_fieldDetector.load(vision_config);
    } catch (boost::exception& exception) {
        std::cerr << boost::diagnostic_information(exception) << std::endl;
        std::cerr << "Seemed to be no config for vision or it is incorrect. "
                  << "It is required to correct that. So, we disabled vision module :("
                  << std::endl;
        m_enabled = false;
    }
}
