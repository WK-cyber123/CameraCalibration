#pragma once

#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>

#include <string>
#include <vector>

#include "CannyDevernay.h"

class CalibrationCam {
public:
    std::vector<cv::Mat> readImages(const std::string& imageFolderPath);
    std::vector<std::vector<cv::Point2f>> findCorners(
        const std::vector<cv::Mat>& chessboard,
        cv::Size patternSize);
    std::vector<std::vector<cv::Point2f>> findCornersSub(
        const std::vector<cv::Mat>& chessboard,
        cv::Size patternSize);
    std::vector<cv::Mat> drawCorners(
        std::vector<cv::Mat> chessboard,
        cv::Size patternSize,
        std::vector<std::vector<cv::Point2f>> findCorners);
    std::vector<std::vector<cv::Point3f>> calculateWorldCorners(
        std::vector<std::vector<cv::Point3f>>& worldCorners,
        int lengthCols,
        int lengthRows,
        cv::Size patternSize);
    std::vector<std::vector<cv::Point2f>> runCalibration(
        const std::string& imageFolderPath,
        int boardWidth,
        int boardHeight,
        int boardScale);
    void saveCalibrationResult(
        std::vector<cv::Mat> image,
        std::vector<std::vector<cv::Point3f>> worldCorners,
        std::vector<std::vector<cv::Point2f>> points,
        cv::Mat cameraMatrix,
        cv::Mat distCoeffs,
        std::vector<cv::Mat> tvecs,
        std::vector<cv::Mat> rvecs);

    std::vector<cv::Mat> calibrationImages;
    std::vector<std::vector<cv::Point3f>> worldCorners;

    cv::Mat cameraMatrixOutput;
    cv::Mat distortionCoefficients;
    std::vector<cv::Mat> rotationMatrices;
    std::vector<cv::Mat> translationVectors;

private:
    std::vector<cv::Mat> images_;
    std::vector<cv::Mat> cornerDebugImages_;
    cv::Size imageSize_;
    bool foundPattern_ = false;
    std::vector<cv::String> imagePaths_;
    cv::Size resultImageSize_;

    std::vector<std::vector<cv::Point2f>> detectedPoints_;
    std::vector<std::vector<cv::Point2f>> subpixelPoints_;
    std::vector<std::vector<cv::Point3f>> worldPoints_;

    cv::Mat cameraMatrix_;
    cv::Mat distCoeffs_;
    std::vector<cv::Mat> tvecsMat_;
    std::vector<cv::Mat> rvecsMat_;
};
