#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "diyrobot/vision.hpp"

int main(int argc, char** argv) {
  const int index = argc > 1 ? std::stoi(argv[1]) : 0;
  cv::VideoCapture camera(index);
  if (!camera.isOpened()) {
    std::cerr << "cannot open camera " << index << '\n';
    return 1;
  }
  diyrobot::ColorTargetDetector detector({170, 10, 80, 255, 60, 255}, 600, 120000);
  for (cv::Mat frame;;) {
    camera >> frame;
    if (frame.empty()) {
      break;
    }
    diyrobot::Image image(static_cast<unsigned>(frame.cols), static_cast<unsigned>(frame.rows));
    for (int y = 0; y < frame.rows; ++y) {
      for (int x = 0; x < frame.cols; ++x) {
        const auto& p = frame.at<cv::Vec3b>(y, x);
        image.at(static_cast<unsigned>(x), static_cast<unsigned>(y)) = {p[0], p[1], p[2]};
      }
    }
    if (auto target = detector.detect_one(image)) {
      cv::rectangle(frame, {target->left, target->top, target->width, target->height}, {0, 255, 0},
                    2);
      cv::circle(frame, {target->x, target->y}, 5, {0, 0, 255}, -1);
    }
    cv::imshow("diyRobotCpp D4 - ESC to exit", frame);
    if ((cv::waitKey(1) & 0xff) == 27) {
      break;
    }
  }
}
