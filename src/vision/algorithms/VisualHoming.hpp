/******************************************************************************
 * @brief Atomic functional library for Visual Homing using ArUco markers.
 * Detects a specific marker on the machine bed to establish a coordinate reference.
 * Updated for OpenCV 4.10.0+ API compliance.
 *
 * @file VisualHoming.hpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-11-27
 *
 * @copyright Copyright RoveSoSeniorDesign 2025 - All Rights Reserved
 ******************************************************************************/

#ifndef VISUAL_HOMING_HPP
#define VISUAL_HOMING_HPP

#include "../../util/vision/CameraModels.hpp"

/// \cond
#include <cmath>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

/// \endcond

namespace VisualHoming
{
    /******************************************************************************
     * @brief Result struct containing the 3D pose of the home marker.
     ******************************************************************************/
    struct HomingResult
    {
            bool bFound   = false;         // Whether the target marker was detected.
            int nMarkerId = -1;            // The ID of the found marker.
            cv::Vec3d cvTranslationVec;    // X, Y, Z offset from Camera to Marker center.
            cv::Vec3d cvRotationVec;       // Rotation vector (Rodrigues) of the marker.
            double dYawDegrees = 0.0;      // Rotation around the Z-axis.
    };

    /******************************************************************************
     * @brief Helper to generate the 3D object points for a single marker centered at (0,0,0).
     * Defines corners: TopLeft, TopRight, BottomRight, BottomLeft.
     * Z axis points out of the marker. Y axis points up.
     *
     * @param fMarkerSize - Size of the marker side.
     * @return std::vector<cv::Point3f> - The 4 corners in 3D space.
     ******************************************************************************/
    inline std::vector<cv::Point3f> GetMarkerObjectPoints(float fMarkerSize)
    {
        float fHalfSize = fMarkerSize / 2.0f;
        return {
            cv::Point3f(-fHalfSize, fHalfSize, 0),    // Top-Left
            cv::Point3f(fHalfSize, fHalfSize, 0),     // Top-Right
            cv::Point3f(fHalfSize, -fHalfSize, 0),    // Bottom-Right
            cv::Point3f(-fHalfSize, -fHalfSize, 0)    // Bottom-Left
        };
    }

    /******************************************************************************
     * @brief Detects a specific ArUco marker to serve as a visual home/reference point.
     * Computes the 3D translation and rotation of the marker relative to the camera.
     *
     * @param cvImage - The input image from the camera.
     * @param stCamConfig - Intrinsic camera parameters (K and D) for accurate pose estimation.
     * @param fMarkerSize - The physical size of the ArUco marker side (e.g., 50.0 mm).
     * @param nTargetMarkerId - The specific ID of the marker to search for (e.g., ID 0).
     * @param eDictionary - The ArUco dictionary to use (default: DICT_4X4_50).
     * @param cvDebugOutput - (Optional) Pointer to a mat to draw detected axis on.
     * @return HomingResult - The computed pose and finding status.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-11-27
     ******************************************************************************/
    inline HomingResult FindHomeMarker(const cv::Mat& cvImage,
                                       const CameraConfig& stCamConfig,
                                       float fMarkerSize,
                                       int nTargetMarkerId                             = 0,
                                       cv::aruco::PredefinedDictionaryType eDictionary = cv::aruco::DICT_4X4_50,
                                       cv::Mat* cvDebugOutput                          = nullptr)
    {
        HomingResult stResult;

        // 1. Validation
        if (cvImage.empty())
            return stResult;

        // 2. Setup ArUco Data Structures (Updated for OpenCV 4.x API)
        // Note: getPredefinedDictionary now returns a Dictionary object, not a Ptr.
        cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(eDictionary);
        // Note: DetectorParameters is now a struct/class, no 'create' factory method.
        cv::aruco::DetectorParameters parameters = cv::aruco::DetectorParameters();

        // 3. Detect Markers
        std::vector<int> vMarkerIds;
        std::vector<std::vector<cv::Point2f>> vMarkerCorners;
        std::vector<std::vector<cv::Point2f>> vRejectedCandidates;

        // In OpenCV 4.10, usually using ArucoDetector class is preferred, but specific headers might vary.
        // We will use the standalone function if available, or ArucoDetector.
        // We will try the ArucoDetector class method which is standard in 4.8+.
        cv::aruco::ArucoDetector detector(dictionary, parameters);
        detector.detectMarkers(cvImage, vMarkerCorners, vMarkerIds, vRejectedCandidates);

        // 4. Search for Target ID
        int nIndex = -1;
        for (size_t i = 0; i < vMarkerIds.size(); ++i)
        {
            if (vMarkerIds[i] == nTargetMarkerId)
            {
                nIndex = static_cast<int>(i);
                break;
            }
        }

        // If not found, return empty result.
        if (nIndex == -1)
            return stResult;

        stResult.bFound    = true;
        stResult.nMarkerId = nTargetMarkerId;

        // 5. Estimate Pose using solvePnP
        // estimatePoseSingleMarkers is deprecated in 4.10. We use solvePnP directly.
        std::vector<cv::Point3f> vObjectPoints = GetMarkerObjectPoints(fMarkerSize);
        std::vector<cv::Point2f> vImagePoints  = vMarkerCorners[nIndex];

        // Solve for pose
        // Use IPPE_SQUARE for better accuracy on flat square markers if available, otherwise ITERATIVE.
        bool bSolved = cv::solvePnP(vObjectPoints,
                                    vImagePoints,
                                    stCamConfig.cvK,
                                    stCamConfig.cvD,
                                    stResult.cvRotationVec,
                                    stResult.cvTranslationVec,
                                    false,
                                    cv::SOLVEPNP_IPPE_SQUARE);

        if (bSolved)
        {
            // 6. Calculate Yaw (Rotation around Z-axis of the marker)
            // Convert Rodrigues vector to Rotation Matrix.
            cv::Mat cvRotMat;
            cv::Rodrigues(stResult.cvRotationVec, cvRotMat);

            // Extract Yaw from Rotation Matrix.
            // Calculation: atan2(R[1][0], R[0][0]) gives rotation in the XY plane.
            stResult.dYawDegrees = std::atan2(cvRotMat.at<double>(1, 0), cvRotMat.at<double>(0, 0)) * (180.0 / CV_PI);
        }

        // 7. Optional Debug Drawing
        if (cvDebugOutput)
        {
            *cvDebugOutput = cvImage.clone();
            if (stResult.bFound)
            {
                cv::aruco::drawDetectedMarkers(*cvDebugOutput, vMarkerCorners, vMarkerIds);

                // Draw 3D axis on the marker center.
                // drawFrameAxes replaces drawAxis in newer OpenCV.
                cv::drawFrameAxes(*cvDebugOutput, stCamConfig.cvK, stCamConfig.cvD, stResult.cvRotationVec, stResult.cvTranslationVec, fMarkerSize * 0.5f);
            }
        }

        return stResult;
    }
}    // namespace VisualHoming

#endif    // VISUAL_HOMING_HPP
