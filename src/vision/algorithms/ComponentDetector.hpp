/******************************************************************************
 * @brief Atomic functional library for detecting the pose (position and rotation)
 * of generic surface mount components. Useful for bottom-vision alignment.
 *
 * @file ComponentDetector.hpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-11-27
 *
 * @copyright Copyright RoveSoSeniorDesign 2025 - All Rights Reserved
 ******************************************************************************/

#ifndef COMPONENT_DETECTOR_HPP
#define COMPONENT_DETECTOR_HPP

#include <opencv2/opencv.hpp>
#include <optional>
#include <vector>

namespace ComponentDetector
{
    /******************************************************************************
     * @brief Result struct containing component pose data.
     ******************************************************************************/
    struct ComponentPose
    {
            bool bFound = false;
            cv::Point2d cvCenterOffset;       // Offset from the image center (in pixels).
            double dRotationDegrees;          // Rotation of the component relative to the image axis.
            cv::RotatedRect cvBoundingBox;    // The computed bounding box.
    };

    /******************************************************************************
     * @brief Detects the offset and rotation of a generic component using blob analysis
     * and minimum area rectangles.
     *
     * @param cvImage - The input image containing the component.
     * @param nThreshold - Threshold value (0-255) to separate component from background.
     * Use -1 to enable Otsu's automatic thresholding.
     * @param bInvert - Set true if component is lighter than background, false otherwise.
     * @return ComponentPose - The calculated pose and bounding box.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-11-27
     ******************************************************************************/
    inline ComponentPose DetectComponentPose(const cv::Mat& cvImage, int nThreshold = -1, bool bInvert = false)
    {
        ComponentPose stResult;

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

        // 2. Thresholding
        cv::Mat cvBinary;
        int nType = cv::THRESH_BINARY;
        if (bInvert)
            nType = cv::THRESH_BINARY_INV;

        if (nThreshold == -1)
        {
            nType |= cv::THRESH_OTSU;
            cv::threshold(cvGray, cvBinary, 0, 255, nType);
        }
        else
        {
            cv::threshold(cvGray, cvBinary, nThreshold, 255, nType);
        }

        // 3. Find Largest Contour (The Component)
        std::vector<std::vector<cv::Point>> vContours;
        cv::findContours(cvBinary, vContours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (vContours.empty())
        {
            return stResult;
        }

        // Find contour with max area.
        auto itMaxContour = std::max_element(vContours.begin(),
                                             vContours.end(),
                                             [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) { return cv::contourArea(a) < cv::contourArea(b); });

        // 4. Compute Pose (Min Area Rect)
        stResult.cvBoundingBox = cv::minAreaRect(*itMaxContour);
        stResult.bFound        = true;

        // Calculate Rotation
        // OpenCV minAreaRect angle is in range [-90, 0).
        double dAngle         = stResult.cvBoundingBox.angle;
        cv::Size2f cvRectSize = stResult.cvBoundingBox.size;

        // Normalize angle based on width/height dominance to ensure consistent orientation
        if (cvRectSize.width < cvRectSize.height)
        {
            dAngle += 90.0;
        }
        stResult.dRotationDegrees = dAngle;

        // Calculate Offset from Image Center
        cv::Point2d cvImageCenter(cvImage.cols / 2.0, cvImage.rows / 2.0);
        stResult.cvCenterOffset = stResult.cvBoundingBox.center - static_cast<cv::Point2f>(cvImageCenter);

        return stResult;
    }
}    // namespace ComponentDetector

#endif    // COMPONENT_DETECTOR_HPP
