/******************************************************************************
 * @brief Atomic functional library for detecting and matching PCB fiducial markers.
 * Specifically tuned for standard circular KiCAD fiducials.
 *
 * @file FiducialDetector.hpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-11-27
 *
 * @copyright Copyright RoveSoSeniorDesign 2025 - All Rights Reserved
 ******************************************************************************/

#ifndef FIDUCIAL_DETECTOR_HPP
#define FIDUCIAL_DETECTOR_HPP

#include <cmath>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

namespace FiducialDetector
{
    /******************************************************************************
     * @brief Configuration parameters for fiducial detection.
     ******************************************************************************/
    struct FiducialConfig
    {
            double dMinArea         = 50.0;       // Minimum area of the contour to be considered.
            double dMaxArea         = 10000.0;    // Maximum area of the contour.
            double dMinCircularity  = 0.85;       // 1.0 is a perfect circle. Fiducials are usually > 0.85.
            int nThresholdBlockSize = 11;         // Block size for adaptive thresholding (must be odd).
            int nThresholdC         = 2;          // Constant subtracted from mean in adaptive thresholding.
    };

    /******************************************************************************
     * @brief Detects circular fiducial markers in an image using contour analysis.
     *
     * @param cvImage - The input image (BGR or Grayscale).
     * @param stConfig - Configuration for area and shape filtering.
     * @return std::vector<cv::Point2f> - A list of center points of detected fiducials.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-11-27
     ******************************************************************************/
    inline std::vector<cv::Point2f> DetectFiducials(const cv::Mat& cvImage, const FiducialConfig& stConfig = FiducialConfig())
    {
        std::vector<cv::Point2f> vFiducialCenters;

        // 1. Preprocessing
        cv::Mat cvGray;
        if (cvImage.channels() == 3)
        {
            cv::cvtColor(cvImage, cvGray, cv::COLOR_BGR2GRAY);
        }
        else
        {
            cvGray = cvImage.clone();
        }

        // Blur to reduce noise (vias, silkscreen noise).
        cv::GaussianBlur(cvGray, cvGray, cv::Size(5, 5), 0);

        // 2. Adaptive Thresholding
        // Using adaptive thresholding handles uneven lighting better than fixed global thresholding.
        cv::Mat cvBinary;
        cv::adaptiveThreshold(cvGray, cvBinary, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, stConfig.nThresholdBlockSize, stConfig.nThresholdC);

        // 3. Contour Detection
        std::vector<std::vector<cv::Point>> vContours;
        cv::findContours(cvBinary, vContours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

        // 4. Filter Contours
        for (const auto& contour : vContours)
        {
            // Calculate Area
            double dArea = cv::contourArea(contour);
            if (dArea < stConfig.dMinArea || dArea > stConfig.dMaxArea)
            {
                continue;
            }

            // Calculate Perimeter and Circularity
            // Circularity = (4 * PI * Area) / (Perimeter^2)
            double dPerimeter = cv::arcLength(contour, true);
            if (dPerimeter == 0)
                continue;

            double dCircularity = (4 * CV_PI * dArea) / (dPerimeter * dPerimeter);

            // Check if it looks like a circle (Fiducial).
            if (dCircularity >= stConfig.dMinCircularity)
            {
                // Compute centroid using moments.
                cv::Moments stMoments = cv::moments(contour);
                if (stMoments.m00 != 0)
                {
                    float fCx = static_cast<float>(stMoments.m10 / stMoments.m00);
                    float fCy = static_cast<float>(stMoments.m01 / stMoments.m00);
                    vFiducialCenters.emplace_back(fCx, fCy);
                }
            }
        }

        return vFiducialCenters;
    }
}    // namespace FiducialDetector

#endif    // FIDUCIAL_DETECTOR_HPP
