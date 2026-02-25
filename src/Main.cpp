#include "CalibrationCamera.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <string>
#include <vector>

namespace {
constexpr int kBoardScale = 15;
constexpr int kBoardHeight = 9;
constexpr int kBoardWidth = 11;
}  // namespace

int main() {
    const std::string cameraImagePath =
        "E:/Code/C++_Code/gamma=3标定/gamma=3标定/camera/";

    CalibrationCam camera;
    std::vector<std::vector<cv::Point2f>> imagePoints =
        camera.runCalibration(cameraImagePath, kBoardWidth, kBoardHeight, kBoardScale);

    return imagePoints.empty() ? 1 : 0;
}
