//
// Created by nikitas on 26.03.16.
//

#ifndef NAOMECH_VISION_H
#define NAOMECH_VISION_H

#include <boost/shared_ptr.hpp>
#include <boost/property_tree/ptree.hpp>
#include <BaseModule.h>
#include <detectors/BallDetector.h>
#include <detectors/LineDetector.h>
#include <detectors/AngleDetector.h>
#include <detectors/FieldDetector.h>
//#include <rrc/core.h>
//#include <rrc/async_worker.h>
//#include <include/BaseModule.h>
//#include <frame_generated.h>

namespace ant {

    class Vision : BaseModule {
        BallDetector m_ballDetector;
        LineDetector m_lineDetector;
        AngleDetector m_angleDetector;
        FieldDetector m_fieldDetector;
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
    public:

        Vision(std::string prop_file = std::string());

        cv::Rect ballDetect();

        std::vector<cv::Vec4i> lineDetect();

        std::vector<cv::Vec3d> angleDetect();

        cv::Mat fieldDetect();

        void setFrame(const cv::Mat &frame);

        void setFrame(cv::Mat &&frame);

        virtual void update_properties() override;

//        virtual void disable() override;

//        virtual void enable() override;
    private:

        cv::Rect ballDetect_old();

        std::vector<cv::Vec4i> lineDetect_old();

    };


}


#endif //NAOMECH_VISION_H

