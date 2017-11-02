//
// Created by nikitas on 26.03.16.
//

#pragma once

#include <vision/detectors/ball_detector_t.h>
#include <vision/detectors/line_detector_t.h>
#include <vision/detectors/AngleDetector.h>
#include <vision/detectors/field_detector_t.h>

namespace drwn {
    class Vision {
    public:
        explicit Vision(std::string prop_file = std::string());

        cv::Rect BallDetect();

        std::vector<cv::Vec4i> LineDetect();

        std::vector<cv::Vec3d> AngleDetect();

        cv::Mat FieldDetect();

        void SetFrame(const cv::Mat& frame);

        void SetFrame(cv::Mat&& frame);

        virtual void UpdateProperties();

    private:
        cv::Rect BallDetectOld();

        std::vector<cv::Vec4i> LineDetectOld();

    private:
        ball_detector_t m_ballDetector;
        line_detector_t m_lineDetector;
        AngleDetector m_angleDetector;
        field_detector_t m_fieldDetector;
        cv::Mat m_image;
        cv::Mat m_src_image;
        cv::Mat m_mask;
        cv::Mat m_preprocImage;
        std::vector<cv::Vec4i> m_lines;
        int m_rotate;
        bool m_send_field;
        bool m_send_ball;
        bool m_send_lines;
        bool m_send_angles;
        std::string m_field_topic;
        std::string m_ball_topic;
        std::string m_lines_topic;
        std::string m_angles_topic;
    };
}
