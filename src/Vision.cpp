//
// Created by nikitas on 26.03.16.
//
#include<chrono>

#include "Vision.h"
#include <VisionUtils.h>
#include <boost/exception/all.hpp>
#include <VisionUtils.h>
//#include <lines_generated.h>
//#include <vec3_generated.h>
//#include <rect_generated.h>

namespace ant {

    Vision::Vision(std::string prop_file) :
        BaseModule(prop_file) {
        update_properties();
    }

    cv::Rect Vision::ballDetect() {
        fieldDetect();
        lineDetect();
        if (m_ballDetector.is_white()) {
            return m_ballDetector.detect(m_preprocImage, m_lines);
        } else {
            return ballDetect_old();
        }
    }

    cv::Rect Vision::ballDetect_old() {
        fieldDetect();
        const cv::Mat preprocImage = m_ballDetector.preproccess(m_image);
        //        cv::imshow("preproc_ball",preprocImage);
        return m_ballDetector.detect_old(preprocImage);
    }

    std::vector<cv::Vec4i> Vision::lineDetect() {
        std::chrono::time_point<std::chrono::system_clock> start, end1, end2;
        
        start = std::chrono::system_clock::now();
        fieldDetect();
        end1 = std::chrono::system_clock::now();
        std::chrono::duration<double> fieldDur = end1 - start;
        
        if (m_lines.empty()) {
            cv::namedWindow("preproc", cv::WINDOW_AUTOSIZE);
            
            start = std::chrono::system_clock::now();
            m_preprocImage = m_lineDetector.preproccess(m_image);
            end1 = std::chrono::system_clock::now();
            
            std::chrono::duration<double> procDur = end1 - start;
            
            m_lines = m_lineDetector.detect(m_preprocImage);
            end2 = std::chrono::system_clock::now();
            std::chrono::duration<double> lineDur = end2 - start;
            
            std::cout << "lineDetector.fieldDetect duration: " << fieldDur.count() << "s" << std::endl;
            std::cout << "lineDetector.preprocess duration: " << procDur.count() << "s" << std::endl;
            std::cout << "lineDetector.detect duration: " << lineDur.count() << "s" << std::endl;
            
            cv::imshow("preproc", m_preprocImage);
        }
        return m_lines;
    }

    std::vector<cv::Vec4i> Vision::lineDetect_old() {
        const cv::Mat preprocImage = m_lineDetector.preproccess_old(m_src_image);


        return m_lineDetector.detect_old(preprocImage);
    }

    std::vector<cv::Vec3d> Vision::angleDetect() {
//        if(m_lines.empty()){
//          lineDetect();
//        }
//        return m_angleDetector.detect(m_image,m_lines);
        if (m_lines.empty()) {
            lineDetect();
        }
        return m_angleDetector.detect(m_image, m_lines);
    }

    cv::Mat Vision::fieldDetect() {
        if (m_mask.empty()) {
            cv::Mat prep = m_fieldDetector.preproccess(m_image);
            m_mask = m_fieldDetector.detect(prep);
            cv::Mat tmp;
            cvtColor(m_image, m_src_image, CV_BGR2YUV);
            m_image.copyTo(tmp, m_mask);
            m_image = tmp;
            cv::cvtColor(m_image, m_image, CV_BGR2YUV);
        }
        return m_mask;
    }

    void Vision::setFrame(const cv::Mat &frame) {
        m_image = frame.clone();
        m_lines.clear();
        m_mask.release();
        m_mask = cv::Mat();
    }

    void Vision::setFrame(cv::Mat &&frame) {
        m_image = frame;
        frame.release();
        m_lines.clear();
        m_mask = cv::Mat();
    }

    void Vision::update_properties() {
        boost::property_tree::ptree properties;
        try {
            boost::property_tree::json_parser::read_json(m_prop_file, properties);
            const boost::property_tree::ptree vision_config = properties.get_child("Vision");
            m_enabled = properties.get<bool>("enabled", true);
            m_lineDetector.load(vision_config);
            m_ballDetector.load(vision_config);
            m_angleDetector.load(vision_config);
            m_fieldDetector.load(vision_config);
        } catch (boost::exception &exception){
            std::cerr << boost::diagnostic_information(exception) << std::endl;
            std::cerr << "Seemed to be no config for vision or it is incorrect. "
                      << "It is required to correct that. So, we disabled vision module :("
                      << std::endl;
            m_enabled = false;
        }
    }
}
