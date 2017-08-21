//
// Created by nikitas on 26.03.16.
//
#include<chrono>

#include "minIni.h"
#include "Vision.h"
#include <VisionUtils.h>
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
            start = std::chrono::system_clock::now();
            m_preprocImage = m_lineDetector.preproccess(m_image);
            end1 = std::chrono::system_clock::now();
            //cv::imshow("preprocess", m_preprocImage);
            
            std::chrono::duration<double> procDur = end1 - start;
            
            m_lines = m_lineDetector.detect(m_preprocImage);
            end2 = std::chrono::system_clock::now();
            std::chrono::duration<double> lineDur = end2 - start;
            
            std::cout << "lineDetector.fieldDetect duration: " << fieldDur.count() << "s" << std::endl;
            std::cout << "lineDetector.preprocess duration: " << procDur.count() << "s" << std::endl;
            std::cout << "lineDetector.detect duration: " << lineDur.count() << "s" << std::endl;
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
        auto visionConfig = m_prop_file + "Vision.ini";
        minIni visionIni(visionConfig);
        m_enabled = visionIni.geti("Vision", "enabled");
        
        if (m_enabled) {
            std::cout << "Vision module is enabled" << std::endl;
        }
        
        // TODO: Fix this mess
        auto fieldDetectorConfig = m_prop_file + "FieldDetector.ini";
        auto lineDetectorConfig = m_prop_file + "LineDetector.ini";
        auto ballDetectorConfig = m_prop_file + "BallDetector.ini";
        auto angleDetectorConfig = m_prop_file + "AngleDetector.ini";
        minIni fieldDetectorIni(fieldDetectorConfig);
        minIni lineDetectorIni(lineDetectorConfig);
        minIni ballDetectorIni(ballDetectorConfig);
        minIni angleDetectorIni(angleDetectorConfig);
        
        m_fieldDetector.load(&fieldDetectorIni);
        m_lineDetector.load(&lineDetectorIni);
        m_ballDetector.load(&ballDetectorIni);
        m_angleDetector.load(&angleDetectorIni);
    }
}

