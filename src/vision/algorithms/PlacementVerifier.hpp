/******************************************************************************
 * @brief Atomic functional library for verifying if a component has been
 * placed successfully using difference imaging.
 *
 * @file PlacementVerifier.hpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-11-27
 *
 * @copyright Copyright RoveSoSeniorDesign 2025 - All Rights Reserved
 ******************************************************************************/

#ifndef PLACEMENT_VERIFIER_HPP
#define PLACEMENT_VERIFIER_HPP

#include <opencv2/opencv.hpp>

namespace PlacementVerifier
{
    /******************************************************************************
     * @brief Verifies placement by calculating the absolute difference between an
     * empty board state and a placed board state within a specific ROI.
     *
     * @param cvBeforeImage - Image of the board BEFORE placement.
     * @param cvAfterImage - Image of the board AFTER placement.
     * @param cvROI - The Region of Interest where the component should be.
     * @param dDifferenceThreshold - Minimum pixel intensity change to register as 'different'.
     * @param dMinChangePercentage - Minimum % of ROI area that must change to confirm placement.
     * @param cvDebugOutput - (Optional) Output mat to visualize the difference.
     * @return true - Component placement detected.
     * @return false - No significant change detected (Missed placement).
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-11-27
     ******************************************************************************/
    inline bool VerifyPlacement(const cv::Mat& cvBeforeImage,
                                const cv::Mat& cvAfterImage,
                                const cv::Rect& cvROI,
                                double dDifferenceThreshold = 30.0,
                                double dMinChangePercentage = 0.05,
                                cv::Mat* cvDebugOutput      = nullptr)
    {
        // 1. Validation
        if (cvBeforeImage.empty() || cvAfterImage.empty())
            return false;

        // Ensure ROI is within bounds.
        cv::Rect cvSafeROI = cvROI & cv::Rect(0, 0, cvBeforeImage.cols, cvBeforeImage.rows);

        // 2. Crop to ROI
        cv::Mat cvCropBefore = cvBeforeImage(cvSafeROI);
        cv::Mat cvCropAfter  = cvAfterImage(cvSafeROI);

        // 3. Preprocessing (Grayscale + Blur)
        cv::Mat cvGrayBefore, cvGrayAfter;
        if (cvCropBefore.channels() == 3)
            cv::cvtColor(cvCropBefore, cvGrayBefore, cv::COLOR_BGR2GRAY);
        else
            cvGrayBefore = cvCropBefore.clone();

        if (cvCropAfter.channels() == 3)
            cv::cvtColor(cvCropAfter, cvGrayAfter, cv::COLOR_BGR2GRAY);
        else
            cvGrayAfter = cvCropAfter.clone();

        cv::GaussianBlur(cvGrayBefore, cvGrayBefore, cv::Size(5, 5), 0);
        cv::GaussianBlur(cvGrayAfter, cvGrayAfter, cv::Size(5, 5), 0);

        // 4. Calculate Absolute Difference
        cv::Mat cvDiff;
        cv::absdiff(cvGrayBefore, cvGrayAfter, cvDiff);

        // 5. Threshold the Difference
        cv::Mat cvMask;
        cv::threshold(cvDiff, cvMask, dDifferenceThreshold, 255, cv::THRESH_BINARY);

        // 6. Calculate Change Percentage
        int nNonZeroPixels    = cv::countNonZero(cvMask);
        double dTotalPixels   = cvSafeROI.width * cvSafeROI.height;
        double dChangePercent = static_cast<double>(nNonZeroPixels) / dTotalPixels;

        // Optional Debug Output
        if (cvDebugOutput)
        {
            *cvDebugOutput = cvMask.clone();
        }

        // 7. Verify
        // If the changed area is greater than the minimum percentage, we assume placement occurred.
        return dChangePercent >= dMinChangePercentage;
    }
}    // namespace PlacementVerifier

#endif    // PLACEMENT_VERIFIER_HPP
