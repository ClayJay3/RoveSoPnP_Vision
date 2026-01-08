/******************************************************************************
 * @brief Atomic functional library for Visual Homing using ArUco markers.
 * Detects specific markers on the machine bed to establish a coordinate reference.
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
#include <map>
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
     * @brief Defines a known marker placed on the bed for localization.
     ******************************************************************************/
    struct KnownMarker
    {
            int nId;                    // The ArUco ID.
            cv::Point3f cvCenterPos;    // The World Coordinates (X, Y, Z) of the marker center.
            float fSize;                // The physical size of the marker side.
            bool bRotated90 = false;    // If true, assumes marker is rotated 90 deg relative to bed axes.
    };

    /******************************************************************************
     * @brief Result struct containing the Global 3D pose of the CAMERA.
     ******************************************************************************/
    struct LocalizationResult
    {
            bool bSuccess    = false;           // Whether localization was successful (PnP converged).
            int nMarkersUsed = 0;               // Number of markers used for the solution.
            cv::Point3d cvCameraPosition;       // World X, Y, Z position of the Camera.
            cv::Vec3d cvCameraRotationEuler;    // World Roll, Pitch, Yaw of the Camera (degrees).
            cv::Mat cvCameraRotMat;             // Rotation Matrix (Camera -> World).
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
     * @brief Helper to calculate the 4 corners of a marker in the GLOBAL frame.
     * Assumes standard alignment (Top edge is +Y, Right edge is +X) unless rotated.
     *
     * @param stMarker - The known marker definition.
     * @return std::vector<cv::Point3f> - The 4 corners in Global 3D space.
     ******************************************************************************/
    inline std::vector<cv::Point3f> GetGlobalMarkerCorners(const KnownMarker& stMarker)
    {
        float fHalf = stMarker.fSize / 2.0f;
        float fX    = stMarker.cvCenterPos.x;
        float fY    = stMarker.cvCenterPos.y;
        float fZ    = stMarker.cvCenterPos.z;

        // Standard Order: Top-Left, Top-Right, Bottom-Right, Bottom-Left
        // Assumes Marker +Y is World +Y, Marker +X is World +X.
        // If your markers are rotated, you would apply a rotation matrix here.
        return {
            cv::Point3f(fX - fHalf, fY + fHalf, fZ),    // TL
            cv::Point3f(fX + fHalf, fY + fHalf, fZ),    // TR
            cv::Point3f(fX + fHalf, fY - fHalf, fZ),    // BR
            cv::Point3f(fX - fHalf, fY - fHalf, fZ)     // BL
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

        // 2. Setup ArUco Data Structures
        cv::aruco::Dictionary dictionary         = cv::aruco::getPredefinedDictionary(eDictionary);
        cv::aruco::DetectorParameters parameters = cv::aruco::DetectorParameters();

        // 3. Detect Markers
        std::vector<int> vMarkerIds;
        std::vector<std::vector<cv::Point2f>> vMarkerCorners;
        std::vector<std::vector<cv::Point2f>> vRejectedCandidates;

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

        if (nIndex == -1)
            return stResult;

        stResult.bFound    = true;
        stResult.nMarkerId = nTargetMarkerId;

        // 5. Estimate Pose using solvePnP
        std::vector<cv::Point3f> vObjectPoints = GetMarkerObjectPoints(fMarkerSize);
        std::vector<cv::Point2f> vImagePoints  = vMarkerCorners[nIndex];

        bool bSolved                           = cv::solvePnP(vObjectPoints,
                                    vImagePoints,
                                    stCamConfig.cvK,
                                    stCamConfig.cvD,
                                    stResult.cvRotationVec,
                                    stResult.cvTranslationVec,
                                    false,
                                    cv::SOLVEPNP_IPPE_SQUARE);

        if (bSolved)
        {
            cv::Mat cvRotMat;
            cv::Rodrigues(stResult.cvRotationVec, cvRotMat);
            stResult.dYawDegrees = std::atan2(cvRotMat.at<double>(1, 0), cvRotMat.at<double>(0, 0)) * (180.0 / CV_PI);
        }

        if (cvDebugOutput)
        {
            *cvDebugOutput = cvImage.clone();
            if (stResult.bFound)
            {
                cv::aruco::drawDetectedMarkers(*cvDebugOutput, vMarkerCorners, vMarkerIds);
                cv::drawFrameAxes(*cvDebugOutput, stCamConfig.cvK, stCamConfig.cvD, stResult.cvRotationVec, stResult.cvTranslationVec, fMarkerSize * 0.5f);
            }
        }

        return stResult;
    }

    /******************************************************************************
     * @brief Performs 3D localization of the CAMERA in the WORLD frame by fusing
     * multiple visible ArUco markers.
     *
     * @param cvImage - Input image.
     * @param stCamConfig - Camera intrinsics.
     * @param vKnownMarkers - A list of markers with their defined world positions.
     * @param eDictionary - ArUco dictionary type.
     * @param cvDebugOutput - (Optional) Debug image output.
     * @return LocalizationResult - The calculated position and rotation of the camera.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-11-28
     ******************************************************************************/
    inline LocalizationResult LocalizeCamera(const cv::Mat& cvImage,
                                             const CameraConfig& stCamConfig,
                                             const std::vector<KnownMarker>& vKnownMarkers,
                                             cv::aruco::PredefinedDictionaryType eDictionary = cv::aruco::DICT_4X4_50,
                                             cv::Mat* cvDebugOutput                          = nullptr)
    {
        LocalizationResult stResult;

        // 1. Validation
        if (cvImage.empty() || vKnownMarkers.empty())
            return stResult;

        // 2. Setup ArUco Detector
        cv::aruco::Dictionary dictionary         = cv::aruco::getPredefinedDictionary(eDictionary);
        cv::aruco::DetectorParameters parameters = cv::aruco::DetectorParameters();

        // 3. Detect Markers
        std::vector<int> vIds;
        std::vector<std::vector<cv::Point2f>> vCorners;
        std::vector<std::vector<cv::Point2f>> vRejected;

        cv::aruco::ArucoDetector detector(dictionary, parameters);
        detector.detectMarkers(cvImage, vCorners, vIds, vRejected);

        // 4. Match Detected Markers to Known World Markers
        std::vector<cv::Point3f> vWorldPoints;    // 3D Object Points
        std::vector<cv::Point2f> vImagePoints;    // 2D Image Points

        // Create a fast lookup map for known markers
        std::map<int, const KnownMarker*> mapKnown;
        for (const auto& marker : vKnownMarkers)
        {
            mapKnown[marker.nId] = &marker;
        }

        int nMarkersUsed = 0;
        for (size_t i = 0; i < vIds.size(); ++i)
        {
            int nId = vIds[i];

            // If this detected marker is in our known list
            if (mapKnown.find(nId) != mapKnown.end())
            {
                // Get the Global 3D corners for this marker
                std::vector<cv::Point3f> vGlobalCorners = GetGlobalMarkerCorners(*mapKnown[nId]);

                // Append to our PnP arrays
                vWorldPoints.insert(vWorldPoints.end(), vGlobalCorners.begin(), vGlobalCorners.end());
                vImagePoints.insert(vImagePoints.end(), vCorners[i].begin(), vCorners[i].end());

                nMarkersUsed++;
            }
        }

        stResult.nMarkersUsed = nMarkersUsed;

        // 5. Solve Perspective-n-Point (PnP)
        // We need at least 4 points (1 marker) for a good solution, though SQPnP/IPPE can do less.
        if (vWorldPoints.size() < 4)
        {
            return stResult;
        }

        cv::Mat rvec, tvec;
        // Use SOLVEPNP_SQPNP for robustness with multiple points, or ITERATIVE.
        bool bSolved = cv::solvePnP(vWorldPoints, vImagePoints, stCamConfig.cvK, stCamConfig.cvD, rvec, tvec, false, cv::SOLVEPNP_SQPNP);

        if (bSolved)
        {
            stResult.bSuccess = true;

            // 6. Convert to Camera Coordinates
            // solvePnP returns the transform from World -> Camera.
            // P_camera = R * P_world + t
            //
            // We want the Camera Position in World Coordinates (C_world).
            // This corresponds to P_camera = (0,0,0).
            // 0 = R * C_world + t  =>  C_world = -R_inv * t  =>  C_world = -R_transpose * t

            cv::Mat R;
            cv::Rodrigues(rvec, R);
            stResult.cvCameraRotMat   = R.t();    // Camera -> World rotation

            cv::Mat cvCamPosMat       = -R.t() * tvec;
            stResult.cvCameraPosition = cv::Point3d(cvCamPosMat);

            // Calculate Euler Angles (Roll, Pitch, Yaw) from the Camera->World rotation matrix
            // Note: This convention depends on how you want to read angles.
            // Standard OpenCV: Pitch (X), Yaw (Y), Roll (Z)
            // But usually we extract them using a standard method (e.g. RQDecomp or simple trig)
            double sy     = std::sqrt(stResult.cvCameraRotMat.at<double>(0, 0) * stResult.cvCameraRotMat.at<double>(0, 0) +
                                  stResult.cvCameraRotMat.at<double>(1, 0) * stResult.cvCameraRotMat.at<double>(1, 0));

            bool singular = sy < 1e-6;
            double x, y, z;

            if (!singular)
            {
                x = atan2(stResult.cvCameraRotMat.at<double>(2, 1), stResult.cvCameraRotMat.at<double>(2, 2));
                y = atan2(-stResult.cvCameraRotMat.at<double>(2, 0), sy);
                z = atan2(stResult.cvCameraRotMat.at<double>(1, 0), stResult.cvCameraRotMat.at<double>(0, 0));
            }
            else
            {
                x = atan2(-stResult.cvCameraRotMat.at<double>(1, 2), stResult.cvCameraRotMat.at<double>(1, 1));
                y = atan2(-stResult.cvCameraRotMat.at<double>(2, 0), sy);
                z = 0;
            }

            // Convert to degrees
            stResult.cvCameraRotationEuler = cv::Vec3d(x, y, z) * (180.0 / CV_PI);
        }

        // 7. Debug Output
        if (cvDebugOutput)
        {
            *cvDebugOutput = cvImage.clone();
            cv::aruco::drawDetectedMarkers(*cvDebugOutput, vCorners, vIds);

            if (stResult.bSuccess)
            {
                // Draw the World Origin (0,0,0) projected onto the camera view
                cv::drawFrameAxes(*cvDebugOutput, stCamConfig.cvK, stCamConfig.cvD, rvec, tvec, 100.0f);

                // Print position on screen
                std::string posText = cv::format("Cam Pos: [%.2f, %.2f, %.2f]", stResult.cvCameraPosition.x, stResult.cvCameraPosition.y, stResult.cvCameraPosition.z);
                cv::putText(*cvDebugOutput, posText, cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
            }
        }

        return stResult;
    }
}    // namespace VisualHoming

#endif    // VISUAL_HOMING_HPP
