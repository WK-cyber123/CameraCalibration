#include "CalibrationCamera.h"

#include <algorithm>
#include <fstream>
#include <iostream>

namespace {

void writePointsToTxt(const std::vector<std::vector<cv::Point2f>>& allPoints,
                      const std::string& filename) {
    std::ofstream outFile(filename);
    if (!outFile.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }

    for (const auto& points : allPoints) {
        for (const auto& point : points) {
            outFile << point.x << ", " << point.y << '\n';
        }
    }
}

void sortByRowsAndCols(std::vector<cv::Point2f>& points,
                       int rows,
                       int cols) {
    if (points.size() != static_cast<size_t>(rows * cols)) {
        std::sort(points.begin(), points.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
            return std::tie(a.y, a.x) < std::tie(b.y, b.x);
        });
        return;
    }

    std::sort(points.begin(), points.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
        return a.y < b.y;
    });

    for (int row = 0; row < rows; ++row) {
        const auto begin = points.begin() + row * cols;
        const auto end = begin + cols;
        std::sort(begin, end, [](const cv::Point2f& a, const cv::Point2f& b) {
            return a.x < b.x;
        });
    }
}

}  // namespace

std::vector<cv::Mat> CalibrationCam::readImages(const std::string& imageFolderPath) {
    cv::glob(imageFolderPath, imagePaths_, false);
    if (imagePaths_.empty()) {
        std::cerr << "No calibration images found in path: " << imageFolderPath << std::endl;
        return {};
    }

    images_.clear();
    for (const auto& imagePath : imagePaths_) {
        cv::Mat image = cv::imread(imagePath, cv::IMREAD_GRAYSCALE);
        if (image.empty()) {
            std::cerr << "Skip invalid image: " << imagePath << std::endl;
            continue;
        }
        images_.push_back(image);
    }

    return images_;
}

std::vector<std::vector<cv::Point2f>> CalibrationCam::findCorners(
    const std::vector<cv::Mat>& chessboard,
    cv::Size patternSize) {
    cv::SimpleBlobDetector::Params params;
    params.maxThreshold = 255;
    params.maxArea = 10e4;
    params.minArea = 25;
    params.filterByArea = true;
    params.blobColor = 255;

    auto blobDetector = cv::SimpleBlobDetector::create(params);
    std::vector<std::vector<cv::Point2f>> allCorners;

    for (const auto& image : chessboard) {
        std::vector<cv::Point2f> corners;
        const bool found = cv::findCirclesGrid(
            image, patternSize, corners, cv::CALIB_CB_SYMMETRIC_GRID, blobDetector);
        if (found) {
            allCorners.push_back(corners);
        }
    }

    writePointsToTxt(allCorners, "center_pixel.txt");
    return allCorners;
}

std::vector<std::vector<cv::Point2f>> CalibrationCam::findCornersSub(
    const std::vector<cv::Mat>& chessboard,
    cv::Size patternSize) {
    auto corners = findCorners(chessboard, patternSize);
    for (auto& points : corners) {
        sortByRowsAndCols(points, patternSize.height, patternSize.width);
    }
    writePointsToTxt(corners, "center_sub.txt");
    return corners;
}

std::vector<cv::Mat> CalibrationCam::drawCorners(
    std::vector<cv::Mat> chessboard,
    cv::Size patternSize,
    std::vector<std::vector<cv::Point2f>> foundCorners) {
    const size_t count = std::min(chessboard.size(), foundCorners.size());
    for (size_t i = 0; i < count; ++i) {
        cv::drawChessboardCorners(chessboard[i], patternSize, foundCorners[i], true);
    }
    return chessboard;
}

std::vector<std::vector<cv::Point3f>> CalibrationCam::calculateWorldCorners(
    std::vector<std::vector<cv::Point3f>>& worldCorners,
    int lengthCols,
    int lengthRows,
    cv::Size patternSize) {
    const cv::Size squareSize(lengthCols, lengthRows);
    std::vector<cv::Point3f> oneFramePoints;
    oneFramePoints.reserve(patternSize.area());

    for (int row = 0; row < patternSize.height; ++row) {
        for (int col = 0; col < patternSize.width; ++col) {
            oneFramePoints.emplace_back(
                static_cast<float>(col * squareSize.width),
                static_cast<float>(row * squareSize.height),
                0.0F);
        }
    }

    worldCorners.assign(calibrationImages.size(), oneFramePoints);
    return worldCorners;
}

void CalibrationCam::saveCalibrationResult(
    std::vector<cv::Mat> image,
    std::vector<std::vector<cv::Point3f>> worldCorners,
    std::vector<std::vector<cv::Point2f>> points,
    cv::Mat cameraMatrix,
    cv::Mat distCoeffs,
    std::vector<cv::Mat> tvecs,
    std::vector<cv::Mat> rvecs) {
    if (image.empty() || worldCorners.empty() || points.empty()) {
        std::cerr << "Calibration input is empty." << std::endl;
        return;
    }

    resultImageSize_ = image.front().size();
    const double error = cv::calibrateCamera(
        worldCorners, points, resultImageSize_, cameraMatrix, distCoeffs, rvecs, tvecs, cv::CALIB_FIX_K3);

    std::ofstream fout("camera_calibration_result.txt");
    fout << "Reprojection error: " << error << '\n';
    fout << "Camera matrix:\n" << cameraMatrix << "\n\n";
    fout << "Distortion coefficients:\n" << distCoeffs << "\n\n";

    cameraMatrixOutput = cameraMatrix;
    distortionCoefficients = distCoeffs;

    rotationMatrices.clear();
    translationVectors.clear();
    for (size_t i = 0; i < image.size() && i < rvecs.size() && i < tvecs.size(); ++i) {
        cv::Mat rotationMatrix;
        cv::Rodrigues(rvecs[i], rotationMatrix);
        rotationMatrices.push_back(rotationMatrix);
        translationVectors.push_back(tvecs[i]);
    }
}

std::vector<std::vector<cv::Point2f>> CalibrationCam::runCalibration(
    const std::string& imageFolderPath,
    int boardWidth,
    int boardHeight,
    int boardScale) {
    const cv::Size boardSize(boardWidth, boardHeight);

    calibrationImages = readImages(imageFolderPath);
    detectedPoints_ = findCorners(calibrationImages, boardSize);
    cornerDebugImages_ = drawCorners(calibrationImages, boardSize, detectedPoints_);
    worldCorners = calculateWorldCorners(worldPoints_, boardScale, boardScale, boardSize);

    saveCalibrationResult(
        calibrationImages, worldCorners, detectedPoints_, cameraMatrix_, distCoeffs_, tvecsMat_, rvecsMat_);
    return detectedPoints_;
}
