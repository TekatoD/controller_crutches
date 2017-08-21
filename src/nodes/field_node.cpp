#include <iostream>
#include <memory>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <Vision.h>
#include <VisionUtils.h>

int main(int argc, char** argv)
{
    ant::Vision vision("../res/vision_cfg/");

    if (argc != 2) {
        std::cout << "Provide image as argument" << std::endl;
        return -1;
    }
    
    cv::Mat frame;
    frame = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);

    if (!frame.data) {
        return -1;
    }

    cv::namedWindow("Field", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Vision result", cv::WINDOW_AUTOSIZE);

    cv::imshow("Field", frame);
    
    std::chrono::time_point<std::chrono::system_clock> start, end;
    
    start = std::chrono::system_clock::now();
    ant::vision_utils::rot90(frame, 0);
    vision.setFrame(frame);
    std::vector<cv::Vec4i> lines = vision.lineDetect();
    end = std::chrono::system_clock::now();
    
    std::chrono::duration<double> dur = end - start;
    std::cout << "lineDetect duration: " << dur.count() << std::endl;

    for (auto& line : lines) {
        cv::Point p1(line[0], line[1]);
        cv::Point p2(line[2], line[3]);

        cv::line(frame, p1, p2, cv::Scalar(0, 0, 255), 5);
    }
    
    /*
    cv::Rect ball = vision.ballDetect();
    std::vector<cv::Vec3d> anglems = vision.angleDetect();
    */
    
    cv::imshow("Vision result", frame);
    cv::waitKey(0);
    return 0;
}
