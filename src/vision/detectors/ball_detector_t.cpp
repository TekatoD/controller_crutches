////
//// Created by nikitas on 4/1/16.
////
//
//
//#include <vision/detectors/ball_detector_t.h>
//#include <array>
//#include <log/trivial_logger_t.h>
//
//using namespace drwn;
//
//ball_detector_t::ball_detector_t()
//        : BaseDetector("BallDetector") {}
//
//cv::Rect ball_detector_t::detect_coloured_ball(const cv::Mat& preprocImage) {
//
//}
//
//
//
//cv::Mat ball_detector_t::preproccess(const cv::Mat& image) {
//
//}
//
//void ball_detector_t::load(const boost::property_tree::ptree& config) {
//    const boost::property_tree::ptree ball_config = config.get_child(DetectorName());
//    m_conf.median_blur_size = ball_config.get<int>("median_blur_size");
//    m_conf.white_ball = ball_config.get<bool>("white_ball");
//    m_conf.ColorThresh.min_1 = ball_config.get<uchar>("ColorThresh.min_1");
//    m_conf.ColorThresh.min_2 = ball_config.get<uchar>("ColorThresh.min_2");
//    m_conf.ColorThresh.min_3 = ball_config.get<uchar>("ColorThresh.min_3");
//    m_conf.ColorThresh.max_1 = ball_config.get<uchar>("ColorThresh.max_1");
//    m_conf.ColorThresh.max_2 = ball_config.get<uchar>("ColorThresh.max_2");
//    m_conf.ColorThresh.max_3 = ball_config.get<uchar>("ColorThresh.max_3");
//    m_conf.GaborThresh.min_1 = ball_config.get<uchar>("GaborThresh.min_1");
//    m_conf.GaborThresh.min_2 = ball_config.get<uchar>("GaborThresh.min_2");
//    m_conf.GaborThresh.min_3 = ball_config.get<uchar>("GaborThresh.min_3");
//    m_conf.GaborThresh.max_1 = ball_config.get<uchar>("GaborThresh.max_1");
//    m_conf.GaborThresh.max_2 = ball_config.get<uchar>("GaborThresh.max_2");
//    m_conf.GaborThresh.max_3 = ball_config.get<uchar>("GaborThresh.max_3");
//}
//
//
//boost::property_tree::ptree ball_detector_t::get_params() {
//    boost::property_tree::ptree ball_config, ptree;
//
//    ball_config.put("median_blur_size", m_conf.median_blur_size);
//    ball_config.put("ColorThresh.min_1", m_conf.ColorThresh.min_1);
//    ball_config.put("ColorThresh.min_2", m_conf.ColorThresh.min_2);
//    ball_config.put("ColorThresh.min_3", m_conf.ColorThresh.min_3);
//    ball_config.put("ColorThresh.max_1", m_conf.ColorThresh.max_1);
//    ball_config.put("ColorThresh.max_2", m_conf.ColorThresh.max_2);
//    ball_config.put("ColorThresh.max_3", m_conf.ColorThresh.max_3);
//    ball_config.put("GaborThresh.min_1", m_conf.GaborThresh.min_1);
//    ball_config.put("GaborThresh.min_2", m_conf.GaborThresh.min_2);
//    ball_config.put("GaborThresh.min_3", m_conf.GaborThresh.min_3);
//    ball_config.put("GaborThresh.max_1", m_conf.GaborThresh.max_1);
//    ball_config.put("GaborThresh.max_2", m_conf.GaborThresh.max_2);
//    ball_config.put("GaborThresh.max_3", m_conf.GaborThresh.max_3);
//
//    ptree.put_child(DetectorName(), ball_config);
//    return ptree;
//}
//
//bool ball_detector_t::IsWhite() const noexcept {
//    return m_conf.white_ball;
//}
//
//
//int ball_detector_t::GetMedianBlurSize() const {
//    return m_median_blur_size;
//}
//
//int ball_detector_t::SetMedianBlurSize(int median_blur_sie) {
//    if (m_debug) {
//        LOG_DEBUG << "BALL DETECTOR: median_blur_size = " << median_blur_sie;
//    }
//    return m_median_blur_size = median_blur_sie;
//}
//
//
//bool ball_detector_t::IsDebugEnabled() {
//    return m_debug;
//}
//
//void ball_detector_t::EnableDebug(bool debug) {
//    m_debug = debug;
//}
//
//const cv::Scalar& ball_detector_t::GetMinGaborColor() const {
//    return m_min_gabor_bgr_color;
//}
//
//void ball_detector_t::SetMinGaborColor(const cv::Scalar& min_gabor_color) {
//    ball_detector_t::m_min_gabor_bgr_color = min_gabor_color;
//}
//
//const cv::Scalar& ball_detector_t::GetMaxGaborColor() const {
//    return m_max_gabor_bgr_color;
//}
//
//void ball_detector_t::SetMaxGaborColor(const cv::Scalar& max_gabor_color) {
//    ball_detector_t::m_max_gabor_bgr_color = max_gabor_color;
//}
//
//const cv::Scalar& ball_detector_t::GetM_min_color() const {
//    return m_min_bgr_color;
//}
//
//void ball_detector_t::SetMinColor(const cv::Scalar& min_color) {
//    ball_detector_t::m_min_bgr_color = min_color;
//}
//
//const cv::Scalar& ball_detector_t::GetMaxColor() const {
//    return m_max_bgr_color;
//}
//
//void ball_detector_t::SetMaxColor(const cv::Scalar& max_color) {
//    ball_detector_t::m_max_bgr_color = max_color;
//}
//
