/******************************************************************************
 * @brief Atomic functional library for calibrating cameras from a set of images.
 * Supports both Standard Pinhole and Fisheye lens models.
 *
 * @file CameraCalibration.hpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-11-24
 *
 * @copyright Copyright RoveSoSeniorDesign 2025 - All Rights Reserved
 ******************************************************************************/

#ifndef CAMERA_CALIBRATION_HPP
#define CAMERA_CALIBRATION_HPP

#include "../../util/vision/CameraModels.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

namespace PixelTo3D
{
    /******************************************************************************
     * @brief Internal Helper: Generates the 3D object points for the chessboard.
     * These are the "real world" coordinates of the corners (z=0).
     *
     * @param stBoardSize - The number of inner corners per a chessboard row and column.
     * @param fSquareSize - The physical size of the square in your desired units (mm, cm, etc).
     * @return std::vector<cv::Point3f> - A vector of 3D points.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-11-24
     ******************************************************************************/
    inline std::vector<cv::Point3f> GenerateObjectPoints(const cv::Size& cvBoardSize, const float fSquareSize)
    {
        std::vector<cv::Point3f> vObjectPoints;
        for (int i = 0; i < cvBoardSize.height; ++i)
        {
            for (int j = 0; j < cvBoardSize.width; ++j)
            {
                vObjectPoints.emplace_back(j * fSquareSize, i * fSquareSize, 0);
            }
        }
        return vObjectPoints;
    }

    /******************************************************************************
     * @brief Runs corner detection and camera calibration on a vector of images.
     * Automatically handles standard Pinhole or Fisheye logic based on the input model.
     *
     * @param vCalibrationImages - A vector of cv::Mats containing the checkerboard patterns.
     * @param cvBoardSize - The width and height of the *internal* corners (e.g., 9x6).
     * @param fSquareSize - The size of a checkerboard square in real-world units (e.g., 25.0f mm).
     * @param eLensModel - The distortion model to use (PINHOLE or FISHEYE).
     * @return CameraConfig - The computed Intrinsic matrix (K) and Distortion coefficients (D).
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-11-24
     ******************************************************************************/
    inline CameraConfig CalibrateCameraFromImages(const std::vector<cv::Mat>& vCalibrationImages,
                                                  const cv::Size& cvBoardSize,
                                                  const float fSquareSize,
                                                  const DistortionModel eLensModel)
    {
        // Output container.
        CameraConfig stResultConfig;
        stResultConfig.eModel = eLensModel;

        // Validation check.
        if (vCalibrationImages.empty())
        {
            std::cerr << "[CameraCalibration] Error: No images provided for calibration." << std::endl;
            return stResultConfig;
        }

        // 1. Prepare Object Points (Real world 3D coords of corners).
        // These are constant for every image (assuming the board doesn't deform).
        std::vector<cv::Point3f> vSinglePatternObjPoints = GenerateObjectPoints(cvBoardSize, fSquareSize);

        // Containers for calibration data.
        std::vector<std::vector<cv::Point2f>> vImagePoints;     // 2D points in image plane.
        std::vector<std::vector<cv::Point3f>> vObjectPoints;    // 3D points in real world space.
        cv::Size cvImageSize = vCalibrationImages[0].size();

        // 2. Detect Corners in all images.
        int nSuccessCount = 0;
        for (const auto& cvImage : vCalibrationImages)
        {
            // Ensure images are the same size.
            if (cvImage.size() != cvImageSize)
            {
                std::cerr << "[CameraCalibration] Warning: Skipping image with mismatched resolution." << std::endl;
                continue;
            }

            // Convert to grayscale.
            cv::Mat cvGray;
            if (cvImage.channels() == 3)
            {
                cv::cvtColor(cvImage, cvGray, cv::COLOR_BGR2GRAY);
            }
            else
            {
                cvGray = cvImage.clone();
            }

            // Find corners.
            std::vector<cv::Point2f> vCorners;
            // Use adaptive threshold + fast check + normalize image for robust detection.
            int nFlags  = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK;
            bool bFound = cv::findChessboardCorners(cvGray, cvBoardSize, vCorners, nFlags);

            if (bFound)
            {
                // Refine corner locations for sub-pixel accuracy.
                cv::cornerSubPix(cvGray, vCorners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));

                // Add to our lists.
                vImagePoints.push_back(vCorners);
                vObjectPoints.push_back(vSinglePatternObjPoints);
                nSuccessCount++;
            }
        }

        std::cout << "[CameraCalibration] Successfully detected corners in " << nSuccessCount << " / " << vCalibrationImages.size() << " images." << std::endl;

        if (nSuccessCount < 1)
        {
            std::cerr << "[CameraCalibration] Error: Not enough valid patterns found to calibrate." << std::endl;
            return stResultConfig;
        }

        // 3. Run Calibration based on Model.
        std::vector<cv::Mat> vRvecs, vTvecs;
        double dReprojectionError = 0.0;

        // Store original dimensions for Fisheye scaling later.
        stResultConfig.cvFisheyeOrigDim = cvImageSize;

        if (eLensModel == PINHOLE)
        {
            // Initialize matrices.
            stResultConfig.cvK = cv::Mat::eye(3, 3, CV_64F);
            stResultConfig.cvD = cv::Mat::zeros(8, 1, CV_64F);

            // Standard OpenCV Calibration.
            dReprojectionError = cv::calibrateCamera(vObjectPoints, vImagePoints, cvImageSize, stResultConfig.cvK, stResultConfig.cvD, vRvecs, vTvecs);
        }
        else    // FISHEYE
        {
            // Initialize matrices.
            stResultConfig.cvK = cv::Mat::eye(3, 3, CV_64F);
            stResultConfig.cvD = cv::Mat::zeros(4, 1, CV_64F);

            // Flags for fisheye: Fix Skew is usually recommended, Recompute Extrinsic needed for convergence.
            int nFisheyeFlags = cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC | cv::fisheye::CALIB_CHECK_COND | cv::fisheye::CALIB_FIX_SKEW;

            // Fisheye Calibration.
            // Note: objectPoints for fisheye often requires vector<vector<Point3d>> in older OpenCV,
            // but modern OpenCV handles Point3f usually. We stick to 3f for consistency but convert if needed.
            // However, cv::fisheye::calibrate signature expects InputArrayOfArrays.

            // Need to strictly match type for some OpenCV versions (convert 3f -> 3d if compilation fails, but standard is flexible).
            // For safety in strict environments, we ensure types match.
            std::vector<std::vector<cv::Point3d>> vObjectPointsDouble;
            std::vector<std::vector<cv::Point2d>> vImagePointsDouble;

            // Convert for Fisheye precision requirements.
            for (const auto& pts : vObjectPoints)
            {
                std::vector<cv::Point3d> dPts;
                for (const auto& p : pts)
                    dPts.emplace_back(p.x, p.y, p.z);
                vObjectPointsDouble.push_back(dPts);
            }
            for (const auto& pts : vImagePoints)
            {
                std::vector<cv::Point2d> dPts;
                for (const auto& p : pts)
                    dPts.emplace_back(p.x, p.y);
                vImagePointsDouble.push_back(dPts);
            }

            dReprojectionError =
                cv::fisheye::calibrate(vObjectPointsDouble, vImagePointsDouble, cvImageSize, stResultConfig.cvK, stResultConfig.cvD, vRvecs, vTvecs, nFisheyeFlags);
        }

        std::cout << "[CameraCalibration] Calibration Complete. RMS Error: " << dReprojectionError << std::endl;

        return stResultConfig;
    }
}    // namespace PixelTo3D

#endif    // CAMERA_CALIBRATION_HPP
