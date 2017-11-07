/**
 *  @autor tekatod
 *  @date 11/7/17
 */
#include "config/strategies/white_ball_vision_processor_configuration_strategy_t.h"

using namespace drwn;

white_ball_vision_processor_configuration_strategy_t::white_ball_vision_processor_configuration_strategy_t(
        white_ball_vision_processor_t* white_ball_vision_processor, std::string section)
        : configuration_strategy_t(std::move(section)), m_white_ball_vision_processor(white_ball_vision_processor) { }

void white_ball_vision_processor_configuration_strategy_t::read_config(const boost::property_tree::ptree& prop) {
    if (m_white_ball_vision_processor != nullptr) {
        if (prop.count(get_section()) == 0) return; // Section doesn't exist

        auto& vision_section = prop.get_child(this->get_section());
        //LINES CONFIGS
        auto& lines_section = vision_section.get_child("LineDetector");
        //Hough
        auto& hough_lines = lines_section.get_child("HoughLines");
        auto max_line_gap = hough_lines.get_optional<float>("max_line_gap");
        auto min_line_length = hough_lines.get_optional<float>("min_line_length");
        auto rho = hough_lines.get_optional<float>("rho");
        auto theta = hough_lines.get_optional<float>("theta");
        auto threshold = hough_lines.get_optional<int>("threshold");
        //Line equal predicate
        auto& line_equal_predicate = lines_section.get_child("LineEqualPredicate");
        auto angle_eps = line_equal_predicate.get_optional<float>("angle_eps");
        auto error_px = line_equal_predicate.get_optional<int>("error_px");
        //Preprocessor
        auto& line_preprocessor = lines_section.get_child("Preproc_new");
//        auto kernel_size = line_preprocessor.get_optional<float>("kernel_size");
//        auto min_thresh = line_preprocessor.get_optional<float>("min_thresh");
        auto& line_preproc_color_thresh1 = line_preprocessor.get_child("ColorThresh1");
        auto line_preproc_color_thresh1_min_1 = line_preproc_color_thresh1.get_optional<float>("min_1");
        auto line_preproc_color_thresh1_min_2 = line_preproc_color_thresh1.get_optional<float>("min_2");
        auto line_preproc_color_thresh1_min_3 = line_preproc_color_thresh1.get_optional<float>("min_3");
        auto line_preproc_color_thresh1_max_1 = line_preproc_color_thresh1.get_optional<float>("max_1");
        auto line_preproc_color_thresh1_max_2 = line_preproc_color_thresh1.get_optional<float>("max_2");
        auto line_preproc_color_thresh1_max_3 = line_preproc_color_thresh1.get_optional<float>("max_3");

        auto& line_preproc_color_thresh2 = line_preprocessor.get_child("ColorThresh2");
        auto line_preproc_color_thresh2_min_1 = line_preproc_color_thresh2.get_optional<float>("min_1");
        auto line_preproc_color_thresh2_min_2 = line_preproc_color_thresh2.get_optional<float>("min_2");
        auto line_preproc_color_thresh2_min_3 = line_preproc_color_thresh2.get_optional<float>("min_3");
        auto line_preproc_color_thresh2_max_1 = line_preproc_color_thresh2.get_optional<float>("max_1");
        auto line_preproc_color_thresh2_max_2 = line_preproc_color_thresh2.get_optional<float>("max_2");
        auto line_preproc_color_thresh2_max_3 = line_preproc_color_thresh2.get_optional<float>("max_3");


        if(max_line_gap) m_white_ball_vision_processor->set_line_detector_hough_lines_max_line_gap(max_line_gap.get());
        if(min_line_length) m_white_ball_vision_processor->set_line_detector_hough_lines_min_line_length(min_line_length.get());
        if(rho) m_white_ball_vision_processor->set_line_detector_hough_lines_rho(rho.get());
        if(theta) m_white_ball_vision_processor->set_line_detector_hough_lines_theta(theta.get());
        if(threshold) m_white_ball_vision_processor->set_line_detector_hough_lines_threshold(threshold.get());
        if(angle_eps) m_white_ball_vision_processor->set_line_detector_line_equality_angle_eps(angle_eps.get());
        if(error_px) m_white_ball_vision_processor->set_line_detector_line_equality_error_px(error_px.get());
        if(line_preproc_color_thresh1_max_1 && line_preproc_color_thresh1_max_2 && line_preproc_color_thresh1_max_3) {
            m_white_ball_vision_processor->set_line_preprocessor_threshold_gabor_bgr_max(cv::Scalar(
                    line_preproc_color_thresh1_max_1.get(),
                    line_preproc_color_thresh1_max_2.get(),
                    line_preproc_color_thresh1_max_3.get()
            ));
        }
        if(line_preproc_color_thresh1_min_1 && line_preproc_color_thresh1_min_2 && line_preproc_color_thresh1_min_3) {
            m_white_ball_vision_processor->set_line_preprocessor_threshold_gabor_bgr_min(cv::Scalar(
                    line_preproc_color_thresh1_min_1.get(),
                    line_preproc_color_thresh1_min_2.get(),
                    line_preproc_color_thresh1_min_3.get()
            ));
        }
        if(line_preproc_color_thresh2_max_1 && line_preproc_color_thresh2_max_2 && line_preproc_color_thresh2_max_3) {
            m_white_ball_vision_processor->set_line_preprocessor_threshold_color_hsv_max(cv::Scalar(
                    line_preproc_color_thresh2_max_1.get(),
                    line_preproc_color_thresh2_max_2.get(),
                    line_preproc_color_thresh2_max_3.get()
            ));
        }
        if(line_preproc_color_thresh2_min_1 && line_preproc_color_thresh2_min_2 && line_preproc_color_thresh2_min_3) {
            m_white_ball_vision_processor->set_line_preprocessor_threshold_color_hsv_min(cv::Scalar(
                    line_preproc_color_thresh2_min_1.get(),
                    line_preproc_color_thresh2_min_2.get(),
                    line_preproc_color_thresh2_min_3.get()
            ));
        }
        //FIELD CONFIGS
        auto& field_section = vision_section.get_child("FieldDetector");
//        auto field_kernel_size = field_section.get_optional<float>("kernel_size");
//        auto field_min_thresh = field_section.get_optional<float>("min_thresh");

        auto& field_color_thresh1 = field_section.get_child("ColorThresh1");
        auto field_color_thresh1_min_1 = field_color_thresh1.get_optional<float>("min_1");
        auto field_color_thresh1_min_2 = field_color_thresh1.get_optional<float>("min_2");
        auto field_color_thresh1_min_3 = field_color_thresh1.get_optional<float>("min_3");
        auto field_color_thresh1_max_1 = field_color_thresh1.get_optional<float>("max_1");
        auto field_color_thresh1_max_2 = field_color_thresh1.get_optional<float>("max_2");
        auto field_color_thresh1_max_3 = field_color_thresh1.get_optional<float>("max_3");

        auto& field_color_thresh2 = field_section.get_child("ColorThresh2");
        auto field_color_thresh2_min_1 = field_color_thresh2.get_optional<float>("min_1");
        auto field_color_thresh2_min_2 = field_color_thresh2.get_optional<float>("min_2");
        auto field_color_thresh2_min_3 = field_color_thresh2.get_optional<float>("min_3");
        auto field_color_thresh2_max_1 = field_color_thresh2.get_optional<float>("max_1");
        auto field_color_thresh2_max_2 = field_color_thresh2.get_optional<float>("max_2");
        auto field_color_thresh2_max_3 = field_color_thresh2.get_optional<float>("max_3");

        if(field_color_thresh1_max_1 && field_color_thresh1_max_2 && field_color_thresh1_max_3) {
            m_white_ball_vision_processor->set_field_preprocessor_threshold_gabor_bgr_max(cv::Scalar(
                    field_color_thresh1_max_1.get(),
                    field_color_thresh1_max_2.get(),
                    field_color_thresh1_max_3.get()
            ));
        }
        if(field_color_thresh1_min_1 && field_color_thresh1_min_2 && field_color_thresh1_min_3) {
            m_white_ball_vision_processor->set_field_preprocessor_threshold_gabor_bgr_min(cv::Scalar(
                    field_color_thresh1_min_1.get(),
                    field_color_thresh1_min_2.get(),
                    field_color_thresh1_min_3.get()
            ));
        }
        if(field_color_thresh2_max_1 && field_color_thresh2_max_2 && field_color_thresh2_max_3) {
            m_white_ball_vision_processor->set_field_preprocessor_threshold_color_hsv_max(cv::Scalar(
                    field_color_thresh2_max_1.get(),
                    field_color_thresh2_max_2.get(),
                    field_color_thresh2_max_3.get()
            ));
        }
        if(field_color_thresh2_min_1 && field_color_thresh2_min_2 && field_color_thresh2_min_3) {
            m_white_ball_vision_processor->set_field_preprocessor_threshold_color_hsv_min(cv::Scalar(
                    field_color_thresh2_min_1.get(),
                    field_color_thresh2_min_2.get(),
                    field_color_thresh2_min_3.get()
            ));
        }
        //BALL CONFIGS
        auto& ball_section = vision_section.get_child("BallDetector");

//        auto ball_white_ball = ball_section.get_optional<int>("white_ball");
        auto ball_median_blur_size = ball_section.get_optional<int>("median_blur_size");

        auto& ball_color_thresh = ball_section.get_child("ColorThresh");
        auto ball_color_thresh_min_1 = ball_color_thresh.get_optional<float>("min_1");
        auto ball_color_thresh_min_2 = ball_color_thresh.get_optional<float>("min_2");
        auto ball_color_thresh_min_3 = ball_color_thresh.get_optional<float>("min_3");
        auto ball_color_thresh_max_1 = ball_color_thresh.get_optional<float>("max_1");
        auto ball_color_thresh_max_2 = ball_color_thresh.get_optional<float>("max_2");
        auto ball_color_thresh_max_3 = ball_color_thresh.get_optional<float>("max_3");

        auto& ball_color_gabor_thresh = ball_section.get_child("GaborThresh");
        auto ball_color_gabor_thresh_min_1 = ball_color_gabor_thresh.get_optional<float>("min_1");
        auto ball_color_gabor_thresh_min_2 = ball_color_gabor_thresh.get_optional<float>("min_2");
        auto ball_color_gabor_thresh_min_3 = ball_color_gabor_thresh.get_optional<float>("min_3");
        auto ball_color_gabor_thresh_max_1 = ball_color_gabor_thresh.get_optional<float>("max_1");
        auto ball_color_gabor_thresh_max_2 = ball_color_gabor_thresh.get_optional<float>("max_2");
        auto ball_color_gabor_thresh_max_3 = ball_color_gabor_thresh.get_optional<float>("max_3");

        if(ball_median_blur_size) m_white_ball_vision_processor->set_ball_preprocessor_median_blur_size(ball_median_blur_size.get());

        if(ball_color_thresh_max_1 && ball_color_thresh_max_2 && ball_color_thresh_max_3) {
            m_white_ball_vision_processor->set_ball_preprocessor_threshold_color_bgr_max(cv::Scalar(
                    ball_color_thresh_max_1.get(),
                    ball_color_thresh_max_2.get(),
                    ball_color_thresh_max_3.get()
            ));
        }
        if(ball_color_thresh_min_1 && ball_color_thresh_min_2 && ball_color_thresh_min_3) {
            m_white_ball_vision_processor->set_ball_preprocessor_threshold_color_bgr_min(cv::Scalar(
                    ball_color_thresh_min_1.get(),
                    ball_color_thresh_min_2.get(),
                    ball_color_thresh_min_3.get()
            ));
        }
        if(ball_color_gabor_thresh_max_1 && ball_color_gabor_thresh_max_2 && ball_color_gabor_thresh_max_3) {
            m_white_ball_vision_processor->set_ball_preprocessor_threshold_gabor_bgr_max(cv::Scalar(
                    ball_color_gabor_thresh_max_1.get(),
                    ball_color_gabor_thresh_max_2.get(),
                    ball_color_gabor_thresh_max_3.get()
            ));
        }
        if(ball_color_gabor_thresh_min_1 && ball_color_gabor_thresh_min_2 && ball_color_gabor_thresh_min_3) {
            m_white_ball_vision_processor->set_ball_preprocessor_threshold_gabor_bgr_min(cv::Scalar(
                    ball_color_gabor_thresh_min_1.get(),
                    ball_color_gabor_thresh_min_2.get(),
                    ball_color_gabor_thresh_min_3.get()
            ));
        }
    }
    else {
        throw std::runtime_error("Vision configuration load fail: m_white_ball_vision_processor nullptr");
    }
}

void white_ball_vision_processor_configuration_strategy_t::write_config(boost::property_tree::ptree& prop) const {
    if (m_white_ball_vision_processor != nullptr) {
        if (prop.count(get_section()) == 0) prop.add_child(get_section(), {});

        auto& vision_section = prop.get_child(this->get_section());

        //LINES
        if (vision_section.count("LineDetector") == 0) vision_section.add_child("LineDetector", {});
        auto& lines_section = vision_section.get_child("LineDetector");
        //Hough
        if (lines_section.count("HoughLines") == 0) lines_section.add_child("HoughLines", {});
        auto& hough_lines = lines_section.get_child("HoughLines");
        hough_lines.put("max_line_gap",m_white_ball_vision_processor->get_line_detector_hough_lines_max_line_gap());
        hough_lines.put("min_line_length",m_white_ball_vision_processor->get_line_detector_hough_lines_min_line_length());
        hough_lines.put("rho",m_white_ball_vision_processor->get_line_detector_hough_lines_rho());
        hough_lines.put("theta",m_white_ball_vision_processor->get_line_detector_hough_lines_theta());
        hough_lines.put("threshold",m_white_ball_vision_processor->get_line_detector_hough_lines_threshold());
        //Line equal predicate
        if (lines_section.count("LineEqualPredicate") == 0) lines_section.add_child("LineEqualPredicate", {});
        auto& line_equal_predicate = lines_section.get_child("LineEqualPredicate");
        line_equal_predicate.put("angle_eps", m_white_ball_vision_processor->get_line_detector_line_equality_angle_eps());
        line_equal_predicate.put("error_px", m_white_ball_vision_processor->get_line_detector_line_equality_error_px());
        //Preprocessor
        if (lines_section.count("Preproc_new") == 0) lines_section.add_child("Preproc_new", {});
        auto& line_preprocessor = lines_section.get_child("Preproc_new");
//        auto kernel_size = line_preprocessor.get_optional<float>("kernel_size");
//        auto min_thresh = line_preprocessor.get_optional<float>("min_thresh");
        if (line_preprocessor.count("ColorThresh1") == 0) line_preprocessor.add_child("ColorThresh1", {});
        auto& line_preproc_color_thresh1 = line_preprocessor.get_child("ColorThresh1");
        if (line_preprocessor.count("ColorThresh2") == 0) line_preprocessor.add_child("ColorThresh2", {});
        auto& line_preproc_color_thresh2 = line_preprocessor.get_child("ColorThresh2");
        auto line_color_max = m_white_ball_vision_processor->get_line_preprocessor_threshold_color_hsv_max();
        auto line_color_min = m_white_ball_vision_processor->get_line_preprocessor_threshold_color_hsv_min();
        auto line_gabor_max = m_white_ball_vision_processor->get_line_preprocessor_threshold_gabor_bgr_max();
        auto line_gabor_min = m_white_ball_vision_processor->get_line_preprocessor_threshold_gabor_bgr_min();
        line_preproc_color_thresh1.put("min_1", line_color_min[0]);
        line_preproc_color_thresh1.put("min_2", line_color_min[1]);
        line_preproc_color_thresh1.put("min_3", line_color_min[2]);
        line_preproc_color_thresh1.put("max_1", line_color_max[0]);
        line_preproc_color_thresh1.put("max_2", line_color_max[1]);
        line_preproc_color_thresh1.put("max_3", line_color_max[2]);
        line_preproc_color_thresh2.put("min_1", line_gabor_min[0]);
        line_preproc_color_thresh2.put("min_2", line_gabor_min[1]);
        line_preproc_color_thresh2.put("min_3", line_gabor_min[2]);
        line_preproc_color_thresh2.put("max_1", line_gabor_max[0]);
        line_preproc_color_thresh2.put("max_2", line_gabor_max[1]);
        line_preproc_color_thresh2.put("max_3", line_gabor_max[2]);

        //FIELD CONFIGS
        if (vision_section.count("FieldDetector") == 0) vision_section.add_child("FieldDetector", {});
        auto& field_section = vision_section.get_child("FieldDetector");
//        auto field_kernel_size = field_section.get_optional<float>("kernel_size");
//        auto field_min_thresh = field_section.get_optional<float>("min_thresh");
        if (field_section.count("ColorThresh1") == 0) field_section.add_child("ColorThresh1", {});
        auto& field_color_thresh1 = field_section.get_child("ColorThresh1");
        if (field_section.count("ColorThresh2") == 0) field_section.add_child("ColorThresh2", {});
        auto& field_color_thresh2 = field_section.get_child("ColorThresh2");
        auto field_color_max = m_white_ball_vision_processor->get_field_preprocessor_threshold_gabor_bgr_max();
        auto field_color_min = m_white_ball_vision_processor->get_field_preprocessor_threshold_gabor_bgr_min();
        auto field_gabor_max = m_white_ball_vision_processor->get_field_preprocessor_threshold_color_hsv_max();
        auto field_gabor_min = m_white_ball_vision_processor->get_field_preprocessor_threshold_color_hsv_min();
        field_color_thresh1.put("min_1", field_color_min[0]);
        field_color_thresh1.put("min_2", field_color_min[1]);
        field_color_thresh1.put("min_3", field_color_min[2]);
        field_color_thresh1.put("max_1", field_color_max[0]);
        field_color_thresh1.put("max_2", field_color_max[1]);
        field_color_thresh1.put("max_3", field_color_max[2]);
        field_color_thresh2.put("min_1", field_gabor_min[0]);
        field_color_thresh2.put("min_2", field_gabor_min[1]);
        field_color_thresh2.put("min_3", field_gabor_min[2]);
        field_color_thresh2.put("max_1", field_gabor_max[0]);
        field_color_thresh2.put("max_2", field_gabor_max[1]);
        field_color_thresh2.put("max_3", field_gabor_max[2]);

        //BALL CONFIGS
        if (vision_section.count("BallDetector") == 0) vision_section.add_child("BallDetector", {});
        auto& ball_section = vision_section.get_child("BallDetector");

//        auto ball_white_ball = ball_section.get_optional<int>("white_ball");
        ball_section.put("median_blur_size", m_white_ball_vision_processor->get_ball_preprocessor_median_blur_size());

        if (ball_section.count("ColorThresh") == 0) ball_section.add_child("ColorThresh", {});
        auto& ball_color_thresh = ball_section.get_child("ColorThresh");
        if (field_section.count("GaborThresh") == 0) field_section.add_child("GaborThresh", {});
        auto& ball_gabor_thresh = ball_section.get_child("GaborThresh");
        auto ball_color_max = m_white_ball_vision_processor->get_ball_preprocessor_threshold_color_bgr_max();
        auto ball_color_min = m_white_ball_vision_processor->get_ball_preprocessor_threshold_color_bgr_min();
        auto ball_gabor_max = m_white_ball_vision_processor->get_ball_preprocessor_threshold_gabor_bgr_max();
        auto ball_gabor_min = m_white_ball_vision_processor->get_ball_preprocessor_threshold_gabor_bgr_min();
        ball_color_thresh.put("min_1", ball_color_min[0]);
        ball_color_thresh.put("min_2", ball_color_min[1]);
        ball_color_thresh.put("min_3", ball_color_min[2]);
        ball_color_thresh.put("max_1", ball_color_max[0]);
        ball_color_thresh.put("max_2", ball_color_max[1]);
        ball_color_thresh.put("max_3", ball_color_max[2]);
        ball_gabor_thresh.put("min_1", ball_gabor_min[0]);
        ball_gabor_thresh.put("min_2", ball_gabor_min[1]);
        ball_gabor_thresh.put("min_3", ball_gabor_min[2]);
        ball_gabor_thresh.put("max_1", ball_gabor_max[0]);
        ball_gabor_thresh.put("max_2", ball_gabor_max[1]);
        ball_gabor_thresh.put("max_3", ball_gabor_max[2]);
    }
    else {
        throw std::runtime_error("Ball Tracker configuration write fail: BallTracker nullptr");
    }
}