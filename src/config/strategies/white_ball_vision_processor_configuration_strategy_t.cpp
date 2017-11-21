/**
 *  @autor tekatod
 *  @date 11/7/17
 */
#include "config/strategies/white_ball_vision_processor_configuration_strategy_t.h"

using namespace drwn;

white_ball_vision_processor_configuration_strategy_t::white_ball_vision_processor_configuration_strategy_t(
        white_ball_vision_processor_t* white_ball_vision_processor, std::string section)
        : configuration_strategy_t(std::move(section)), m_white_ball_vision_processor(white_ball_vision_processor) {}

void white_ball_vision_processor_configuration_strategy_t::read_config(const boost::property_tree::ptree& prop) {
    if (m_white_ball_vision_processor != nullptr) {
        if (prop.count(this->get_section()) == 0) return; // Section doesn't exist

        auto& section = prop.get_child(this->get_section());
        //LINES CONFIGS
        //Hough
        auto max_line_gap = section.get_optional<float>("line_detector_hough_max_line_gap");
        auto min_line_length = section.get_optional<float>("line_detector_hough_min_line_length");
        auto rho = section.get_optional<float>("line_detector_hough_rho");
        auto theta = section.get_optional<float>("line_detector_hough_theta");
        auto threshold = section.get_optional<int>("line_detector_hough_threshold");
        //Line equal predicate
        auto angle_eps = section.get_optional<float>("line_detector_line_equality_angle_eps");
        auto error_px = section.get_optional<int>("line_detector_line_equality_error_px");
        //Preprocessor
//        auto kernel_size = line_preprocessor.get_optional<float>("kernel_size");
//        auto min_thresh = line_preprocessor.get_optional<float>("min_thresh");
        auto line_preproc_gabor_thresh_min_b = section.get_optional<float>("line_detector_gabor_thresh_min_b");
        auto line_preproc_gabor_thresh_min_g = section.get_optional<float>("line_detector_gabor_thresh_min_g");
        auto line_preproc_gabor_thresh_min_r = section.get_optional<float>("line_detector_gabor_thresh_min_r");
        auto line_preproc_gabor_thresh_max_b = section.get_optional<float>("line_detector_gabor_thresh_max_b");
        auto line_preproc_gabor_thresh_max_g = section.get_optional<float>("line_detector_gabor_thresh_max_g");
        auto line_preproc_gabor_thresh_max_r = section.get_optional<float>("line_detector_gabor_thresh_max_r");

        auto line_preproc_color_thresh_min_h = section.get_optional<float>("line_detector_color_thresh_min_h");
        auto line_preproc_color_thresh_min_s = section.get_optional<float>("line_detector_color_thresh_min_s");
        auto line_preproc_color_thresh_min_v = section.get_optional<float>("line_detector_color_thresh_min_v");
        auto line_preproc_color_thresh_max_h = section.get_optional<float>("line_detector_color_thresh_max_h");
        auto line_preproc_color_thresh_max_s = section.get_optional<float>("line_detector_color_thresh_max_s");
        auto line_preproc_color_thresh_max_v = section.get_optional<float>("line_detector_color_thresh_max_v");


        if (max_line_gap) m_white_ball_vision_processor->set_line_detector_hough_lines_max_line_gap(max_line_gap.get());
        if (min_line_length)
            m_white_ball_vision_processor->set_line_detector_hough_lines_min_line_length(min_line_length.get());
        if (rho) m_white_ball_vision_processor->set_line_detector_hough_lines_rho(rho.get());
        if (theta) m_white_ball_vision_processor->set_line_detector_hough_lines_theta(theta.get());
        if (threshold) m_white_ball_vision_processor->set_line_detector_hough_lines_threshold(threshold.get());
        if (angle_eps) m_white_ball_vision_processor->set_line_detector_line_equality_angle_eps(angle_eps.get());
        if (error_px) m_white_ball_vision_processor->set_line_detector_line_equality_error_px(error_px.get());
        if (line_preproc_gabor_thresh_min_b && line_preproc_gabor_thresh_min_g && line_preproc_gabor_thresh_min_r) {
            m_white_ball_vision_processor->set_line_preprocessor_threshold_gabor_bgr_min(cv::Scalar(
                    line_preproc_gabor_thresh_min_b.get(),
                    line_preproc_gabor_thresh_min_g.get(),
                    line_preproc_gabor_thresh_min_r.get()
            ));
        }
        if (line_preproc_gabor_thresh_max_b && line_preproc_gabor_thresh_max_g && line_preproc_gabor_thresh_max_r) {
            m_white_ball_vision_processor->set_line_preprocessor_threshold_gabor_bgr_max(cv::Scalar(
                    line_preproc_gabor_thresh_max_b.get(),
                    line_preproc_gabor_thresh_max_g.get(),
                    line_preproc_gabor_thresh_max_r.get()
            ));
        }
        if (line_preproc_color_thresh_min_h && line_preproc_color_thresh_min_s && line_preproc_color_thresh_min_v) {
            m_white_ball_vision_processor->set_line_preprocessor_threshold_color_hsv_min(cv::Scalar(
                    line_preproc_color_thresh_min_h.get(),
                    line_preproc_color_thresh_min_s.get(),
                    line_preproc_color_thresh_min_v.get()
            ));
        }
        if (line_preproc_color_thresh_max_h && line_preproc_color_thresh_max_s && line_preproc_color_thresh_max_v) {
            m_white_ball_vision_processor->set_line_preprocessor_threshold_color_hsv_max(cv::Scalar(
                    line_preproc_color_thresh_max_h.get(),
                    line_preproc_color_thresh_max_s.get(),
                    line_preproc_color_thresh_max_v.get()
            ));
        }

        //FIELD CONFIGS
//        auto field_kernel_size = field_section.get_optional<float>("kernel_size");
//        auto field_min_thresh = field_section.get_optional<float>("min_thresh");

        auto field_gabor_thresh_min_b = section.get_optional<float>("field_detector_gabor_thresh_min_b");
        auto field_gabor_thresh_min_g = section.get_optional<float>("field_detector_gabor_thresh_min_g");
        auto field_gabor_thresh_min_r = section.get_optional<float>("field_detector_gabor_thresh_min_r");
        auto field_gabor_thresh_max_b = section.get_optional<float>("field_detector_gabor_thresh_max_b");
        auto field_gabor_thresh_max_g = section.get_optional<float>("field_detector_gabor_thresh_max_g");
        auto field_gabor_thresh_max_r = section.get_optional<float>("field_detector_gabor_thresh_max_r");

        auto field_color_thresh_min_h = section.get_optional<float>("field_detector_color_thresh_min_h");
        auto field_color_thresh_min_s = section.get_optional<float>("field_detector_color_thresh_min_s");
        auto field_color_thresh_min_v = section.get_optional<float>("field_detector_color_thresh_min_v");
        auto field_color_thresh_max_h = section.get_optional<float>("field_detector_color_thresh_max_h");
        auto field_color_thresh_max_s = section.get_optional<float>("field_detector_color_thresh_max_s");
        auto field_color_thresh_max_v = section.get_optional<float>("field_detector_color_thresh_max_v");

        auto field_enabled = section.get_optional<bool>("field_detector_enabled");

        if (field_gabor_thresh_min_b && field_gabor_thresh_min_g && field_gabor_thresh_min_r) {
            m_white_ball_vision_processor->set_field_preprocessor_threshold_gabor_bgr_min(cv::Scalar(
                    field_gabor_thresh_min_b.get(),
                    field_gabor_thresh_min_g.get(),
                    field_gabor_thresh_min_r.get()
            ));
        }
        if (field_gabor_thresh_max_b && field_gabor_thresh_max_g && field_gabor_thresh_max_r ) {
            m_white_ball_vision_processor->set_field_preprocessor_threshold_gabor_bgr_max(cv::Scalar(
                    field_gabor_thresh_max_b.get(),
                    field_gabor_thresh_max_g.get(),
                    field_gabor_thresh_max_r.get()
            ));
        }
        if (field_color_thresh_min_h && field_color_thresh_min_s && field_color_thresh_min_v) {
            m_white_ball_vision_processor->set_field_preprocessor_threshold_color_hsv_min(cv::Scalar(
                    field_color_thresh_min_h.get(),
                    field_color_thresh_min_s.get(),
                    field_color_thresh_min_v.get()
            ));
        }
        if (field_color_thresh_max_h && field_color_thresh_max_s && field_color_thresh_max_v) {
            m_white_ball_vision_processor->set_field_preprocessor_threshold_color_hsv_max(cv::Scalar(
                    field_color_thresh_max_h.get(),
                    field_color_thresh_max_s.get(),
                    field_color_thresh_max_v.get()
            ));
        }
        if(field_enabled) m_white_ball_vision_processor->enable_field_processing(field_enabled.get());
        //BALL CONFIGS

//        auto ball_white_ball = ball_section.get_optional<int>("white_ball");
        auto ball_median_blur_size = section.get_optional<int>("ball_detector_median_blur_size");

        auto ball_detector_color_thresh_min_b = section.get_optional<float>("ball_detector_color_thresh_min_b");
        auto ball_detector_color_thresh_min_g = section.get_optional<float>("ball_detector_color_thresh_min_g");
        auto ball_detector_color_thresh_min_r = section.get_optional<float>("ball_detector_color_thresh_min_r");
        auto ball_detector_color_thresh_max_b = section.get_optional<float>("ball_detector_color_thresh_max_b");
        auto ball_detector_color_thresh_max_g = section.get_optional<float>("ball_detector_color_thresh_max_g");
        auto ball_detector_color_thresh_max_r = section.get_optional<float>("ball_detector_color_thresh_max_r");

        auto ball_detector_gabor_thresh_min_b = section.get_optional<float>("ball_detector_gabor_thresh_min_b");
        auto ball_detector_gabor_thresh_min_g = section.get_optional<float>("ball_detector_gabor_thresh_min_g");
        auto ball_detector_gabor_thresh_min_r = section.get_optional<float>("ball_detector_gabor_thresh_min_r");
        auto ball_detector_gabor_thresh_max_b = section.get_optional<float>("ball_detector_gabor_thresh_max_b");
        auto ball_detector_gabor_thresh_max_g = section.get_optional<float>("ball_detector_gabor_thresh_max_g");
        auto ball_detector_gabor_thresh_max_r = section.get_optional<float>("ball_detector_gabor_thresh_max_r");

        auto ball_detector_area_top = section.get_optional<int>("ball_detector_area_top");
        auto ball_detector_area_low = section.get_optional<double>("ball_detector_area_low");
        auto ball_detector_type = section.get_optional<int>("ball_detector_type");
        auto ball_detector_haar_config = section.get_optional<std::string>("ball_detector_cascade_config");

        if(ball_detector_area_low) m_white_ball_vision_processor->set_ball_detector_area_low(ball_detector_area_low.get());
        if(ball_detector_area_top) m_white_ball_vision_processor->set_ball_detector_area_top(ball_detector_area_top.get());
        if(ball_detector_type) m_white_ball_vision_processor->set_ball_detector_type(ball_detector_type.get());
        if(ball_detector_haar_config) m_white_ball_vision_processor->set_ball_detector_cascade_config(ball_detector_haar_config.get());

        if (ball_median_blur_size)
            m_white_ball_vision_processor->set_ball_detector_median_blur_size(ball_median_blur_size.get());

        if (ball_detector_color_thresh_min_b && ball_detector_color_thresh_min_g && ball_detector_color_thresh_min_r) {
            m_white_ball_vision_processor->set_ball_detector_threshold_color_bgr_min(cv::Scalar(
                    ball_detector_color_thresh_min_b.get(),
                    ball_detector_color_thresh_min_g.get(),
                    ball_detector_color_thresh_min_r.get()
            ));
        }
        if (ball_detector_color_thresh_max_b && ball_detector_color_thresh_max_g && ball_detector_color_thresh_max_r) {
            m_white_ball_vision_processor->set_ball_detector_threshold_color_bgr_max(cv::Scalar(
                    ball_detector_color_thresh_max_b.get(),
                    ball_detector_color_thresh_max_g.get(),
                    ball_detector_color_thresh_max_r.get()
            ));
        }
        if (ball_detector_gabor_thresh_min_b && ball_detector_gabor_thresh_min_g && ball_detector_gabor_thresh_min_r) {
            m_white_ball_vision_processor->set_ball_detector_threshold_gabor_bgr_min(cv::Scalar(
                    ball_detector_gabor_thresh_min_b.get(),
                    ball_detector_gabor_thresh_min_g.get(),
                    ball_detector_gabor_thresh_min_r.get()
            ));
        }
        if (ball_detector_gabor_thresh_max_b && ball_detector_gabor_thresh_max_g && ball_detector_gabor_thresh_max_r) {
            m_white_ball_vision_processor->set_ball_detector_threshold_gabor_bgr_max(cv::Scalar(
                    ball_detector_gabor_thresh_max_b.get(),
                    ball_detector_gabor_thresh_max_g.get(),
                    ball_detector_gabor_thresh_max_r.get()
            ));
        }
    } else {
        throw std::runtime_error("Vision configuration load fail: m_white_ball_vision_processor nullptr");
    }
}

void white_ball_vision_processor_configuration_strategy_t::write_config(boost::property_tree::ptree& prop) const {
    if (m_white_ball_vision_processor != nullptr) {
        if (prop.count(get_section()) == 0) prop.add_child(get_section(), {});
        auto& section = prop.get_child(this->get_section());

        //LINES
        //Hough
        section.put("line_detector_hough_max_line_gap", m_white_ball_vision_processor->get_line_detector_hough_lines_max_line_gap());
        section.put("line_detector_hough_min_line_length", m_white_ball_vision_processor->get_line_detector_hough_lines_min_line_length());
        section.put("line_detector_hough_rho", m_white_ball_vision_processor->get_line_detector_hough_lines_rho());
        section.put("line_detector_hough_theta", m_white_ball_vision_processor->get_line_detector_hough_lines_theta());
        section.put("line_detector_hough_threshold", m_white_ball_vision_processor->get_line_detector_hough_lines_threshold());
        //Line equal predicate
        section.put("line_detector_line_equality_angle_eps", m_white_ball_vision_processor->get_line_detector_line_equality_angle_eps());
        section.put("line_detector_line_equality_error_px", m_white_ball_vision_processor->get_line_detector_line_equality_error_px());
        //Preprocessor
//        auto kernel_size = line_preprocessor.get_optional<float>("kernel_size");
//        auto min_thresh = line_preprocessor.get_optional<float>("min_thresh");
        auto& line_color_hsv_min = m_white_ball_vision_processor->get_line_preprocessor_threshold_color_hsv_min();
        auto& line_color_hsv_max = m_white_ball_vision_processor->get_line_preprocessor_threshold_color_hsv_max();
        auto& line_gabor_bgr_min = m_white_ball_vision_processor->get_line_preprocessor_threshold_gabor_bgr_min();
        auto& line_gabor_bgr_max = m_white_ball_vision_processor->get_line_preprocessor_threshold_gabor_bgr_max();
        section.put("line_detector_color_thresh_min_h", line_color_hsv_min[0]);
        section.put("line_detector_color_thresh_min_s", line_color_hsv_min[1]);
        section.put("line_detector_color_thresh_min_v", line_color_hsv_min[2]);
        section.put("line_detector_color_thresh_max_h", line_color_hsv_max[0]);
        section.put("line_detector_color_thresh_max_s", line_color_hsv_max[1]);
        section.put("line_detector_color_thresh_max_v", line_color_hsv_max[2]);
        section.put("line_detector_gabor_thresh_min_b", line_gabor_bgr_min[0]);
        section.put("line_detector_gabor_thresh_min_g", line_gabor_bgr_min[1]);
        section.put("line_detector_gabor_thresh_min_r", line_gabor_bgr_min[2]);
        section.put("line_detector_gabor_thresh_max_b", line_gabor_bgr_max[0]);
        section.put("line_detector_gabor_thresh_max_g", line_gabor_bgr_max[1]);
        section.put("line_detector_gabor_thresh_max_r", line_gabor_bgr_max[2]);

        //FIELD CONFIGS
//        auto field_kernel_size = field_section.get_optional<float>("kernel_size");
//        auto field_min_thresh = field_section.get_optional<float>("min_thresh");
        auto field_color_max = m_white_ball_vision_processor->get_field_preprocessor_threshold_color_hsv_max();
        auto field_color_min = m_white_ball_vision_processor->get_field_preprocessor_threshold_color_hsv_min();
        auto field_gabor_max = m_white_ball_vision_processor->get_field_preprocessor_threshold_gabor_bgr_max();
        auto field_gabor_min = m_white_ball_vision_processor->get_field_preprocessor_threshold_gabor_bgr_min();
        section.put("field_detector_color_thresh_min_h", field_color_min[0]);
        section.put("field_detector_color_thresh_min_s", field_color_min[1]);
        section.put("field_detector_color_thresh_min_v", field_color_min[2]);
        section.put("field_detector_color_thresh_max_h", field_color_max[0]);
        section.put("field_detector_color_thresh_max_s", field_color_max[1]);
        section.put("field_detector_color_thresh_max_v", field_color_max[2]);
        section.put("field_detector_gabor_thresh_min_b", field_gabor_min[0]);
        section.put("field_detector_gabor_thresh_min_g", field_gabor_min[1]);
        section.put("field_detector_gabor_thresh_min_r", field_gabor_min[2]);
        section.put("field_detector_gabor_thresh_max_b", field_gabor_max[0]);
        section.put("field_detector_gabor_thresh_max_g", field_gabor_max[1]);
        section.put("field_detector_gabor_thresh_max_r", field_gabor_max[2]);
        section.put("field_detector_enbaled", m_white_ball_vision_processor->is_field_processing_enabled());

        //BALL CONFIGS

//        auto ball_white_ball = ball_section.get_optional<int>("white_ball");
        section.put("ball_detector_median_blur_size", m_white_ball_vision_processor->get_ball_detector_median_blur_size());

        auto ball_color_max = m_white_ball_vision_processor->get_ball_detector_threshold_color_bgr_max();
        auto ball_color_min = m_white_ball_vision_processor->get_ball_detector_threshold_color_bgr_min();
        auto ball_gabor_max = m_white_ball_vision_processor->get_ball_detector_threshold_gabor_bgr_max();
        auto ball_gabor_min = m_white_ball_vision_processor->get_ball_detector_threshold_gabor_bgr_min();
        section.put("ball_detector_color_thresh_min_b", ball_color_min[0]);
        section.put("ball_detector_color_thresh_min_g", ball_color_min[1]);
        section.put("ball_detector_color_thresh_min_r", ball_color_min[2]);
        section.put("ball_detector_color_thresh_max_b", ball_color_max[0]);
        section.put("ball_detector_color_thresh_max_g", ball_color_max[1]);
        section.put("ball_detector_color_thresh_max_r", ball_color_max[2]);
        section.put("ball_detector_gabor_thresh_min_b", ball_gabor_min[0]);
        section.put("ball_detector_gabor_thresh_min_g", ball_gabor_min[1]);
        section.put("ball_detector_gabor_thresh_min_r", ball_gabor_min[2]);
        section.put("ball_detector_gabor_thresh_max_b", ball_gabor_max[0]);
        section.put("ball_detector_gabor_thresh_max_g", ball_gabor_max[1]);
        section.put("ball_detector_gabor_thresh_max_r", ball_gabor_max[2]);
        section.put("ball_detector_area_low", m_white_ball_vision_processor->get_ball_detector_area_low());
        section.put("ball_detector_area_top", m_white_ball_vision_processor->get_ball_detector_area_top());
        section.put("ball_detector_type", m_white_ball_vision_processor->get_ball_detector_type());
        section.put("ball_detector_cascade_config", m_white_ball_vision_processor->get_ball_detector_cascade_config());

    } else {
        throw std::runtime_error("Ball Tracker configuration write fail: BallTracker nullptr");
    }
}

white_ball_vision_processor_t*
white_ball_vision_processor_configuration_strategy_t::get_white_ball_vision_processor() const {
    return m_white_ball_vision_processor;
}

void white_ball_vision_processor_configuration_strategy_t::set_white_ball_vision_processor(
        white_ball_vision_processor_t* white_ball_vision_processor) {
    m_white_ball_vision_processor = white_ball_vision_processor;
}
