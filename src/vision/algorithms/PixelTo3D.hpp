/******************************************************************************
 * @brief Atomic functional library for projecting 2D pixels to 3D world coordinates.
 * Contains logic for Extrinsics, Ray Casting, and Plane Intersection.
 *
 * @file PixelTo3D.hpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-11-24
 *
 * @copyright Copyright RoveSoSeniorDesign 2025 - All Rights Reserved
 ******************************************************************************/

#ifndef PIXEL_TO_3D_HPP
#define PIXEL_TO_3D_HPP

#include "../../util/vision/CameraModels.hpp"

/// \cond
#include <cmath>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <optional>

/// \endcond

namespace PixelTo3D
{
    /******************************************************************************
     * @brief Internal Helper: Convert Euler angles (RPY) to a Rotation Matrix.
     * Rotation Order: Rz * Ry * Rx.
     *
     * @param dRollDeg - Roll in degrees.
     * @param dPitchDeg - Pitch in degrees.
     * @param dYawDeg - Yaw in degrees.
     * @return cv::Mat - 3x3 Rotation Matrix.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-11-24
     ******************************************************************************/
    inline cv::Mat RpyToRotMat(const double dRollDeg, const double dPitchDeg, const double dYawDeg)
    {
        double dR    = dRollDeg * CV_PI / 180.0;
        double dP    = dPitchDeg * CV_PI / 180.0;
        double dY    = dYawDeg * CV_PI / 180.0;

        cv::Mat cvRx = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, std::cos(dR), -std::sin(dR), 0, std::sin(dR), std::cos(dR));

        cv::Mat cvRy = (cv::Mat_<double>(3, 3) << std::cos(dP), 0, std::sin(dP), 0, 1, 0, -std::sin(dP), 0, std::cos(dP));

        cv::Mat cvRz = (cv::Mat_<double>(3, 3) << std::cos(dY), -std::sin(dY), 0, std::sin(dY), std::cos(dY), 0, 0, 0, 1);

        return cvRz * cvRy * cvRx;
    }

    /******************************************************************************
     * @brief Internal Helper: Scales the camera matrix K if the image resolution
     * differs from the calibration resolution. Essential for Fisheye logic.
     *
     * @param cvK - Original Camera Matrix.
     * @param cvOldSize - Original resolution.
     * @param cvNewSize - Current resolution.
     * @return cv::Mat - Scaled Camera Matrix.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-11-24
     ******************************************************************************/
    inline cv::Mat ScaleK(const cv::Mat& cvK, const cv::Size cvOldSize, const cv::Size cvNewSize)
    {
        cv::Mat cvKs = cvK.clone();
        cvKs.convertTo(cvKs, CV_64F);

        double dSx = (double) cvNewSize.width / (double) cvOldSize.width;
        double dSy = (double) cvNewSize.height / (double) cvOldSize.height;

        cvKs.at<double>(0, 0) *= dSx;
        cvKs.at<double>(0, 2) *= dSx;
        cvKs.at<double>(1, 1) *= dSy;
        cvKs.at<double>(1, 2) *= dSy;

        return cvKs;
    }

    /******************************************************************************
     * @brief Calculates the 3D world offset (in mm) of a specific pixel relative to the camera.
     * This function encapsulates the full pipeline:
     * 1. Undistortion Matrix calculation.
     * 2. Ray Casting (Pixel -> Camera Frame).
     * 3. Extrinsics Transform (Camera Frame -> World Frame).
     * 4. Ray-Plane Intersection.
     *
     * @param cvFrame - The image frame (used only for determining width/height).
     * @param stConfig - Camera calibration (K, D, Model).
     * @param cvCamPos - Camera position in World (X, Y, Z) mm.
     * @param cvCamRPY - Camera rotation (Roll, Pitch, Yaw) in degrees.
     * @param cvPixel - The target pixel coordinates (u, v).
     * NOTE: Assumes the visual reference is the UNDISTORTED image.
     * @param stTargetPlane - (Optional) The plane to intersect with. Defaults to Ground (Z=0).
     * @return std::optional<cv::Point3d> - The X,Y,Z offset from camera. Nullopt if no intersection.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-11-24
     ******************************************************************************/
    inline std::optional<cv::Point3d> ComputePixelTo3DOffset(const cv::Mat& cvFrame,
                                                             const CameraConfig& stConfig,
                                                             const cv::Point3d& cvCamPos,
                                                             const cv::Vec3d& cvCamRPY,
                                                             const cv::Point2d& cvPixel,
                                                             const Plane& stTargetPlane = Plane())
    {
        cv::Size cvImgSize = cvFrame.size();

        // ---------------------------------------------------------
        // 1. Determine Effective Camera Matrix (K_used)
        // ---------------------------------------------------------
        cv::Mat cvKUsed;
        if (stConfig.eModel == PINHOLE)
        {
            // "alpha=1" ensures we keep all pixels, matching standard behavior.
            cvKUsed = cv::getOptimalNewCameraMatrix(stConfig.cvK, stConfig.cvD, cvImgSize, 1, cvImgSize);
        }
        else
        {
            // Fisheye scaling logic.
            if (!stConfig.cvFisheyeOrigDim.empty() && stConfig.cvFisheyeOrigDim != cvImgSize)
            {
                cvKUsed = ScaleK(stConfig.cvK, stConfig.cvFisheyeOrigDim, cvImgSize);
            }
            else
            {
                cvKUsed = stConfig.cvK.clone();
            }
        }

        cvKUsed.convertTo(cvKUsed, CV_64F);

        // ---------------------------------------------------------
        // 2. Compute Ray in Camera Coordinates
        //    Math: (u - cx) / fx, (v - cy) / fy, 1.0.
        // ---------------------------------------------------------
        double dFx       = cvKUsed.at<double>(0, 0);
        double dFy       = cvKUsed.at<double>(1, 1);
        double dCx       = cvKUsed.at<double>(0, 2);
        double dCy       = cvKUsed.at<double>(1, 2);

        cv::Mat cvRayCam = (cv::Mat_<double>(3, 1) << (cvPixel.x - dCx) / dFx, (cvPixel.y - dCy) / dFy, 1.0);

        // Normalize the ray vector.
        double dNorm = cv::norm(cvRayCam);
        if (dNorm > 1e-9)
        {
            cvRayCam /= dNorm;
        }

        // ---------------------------------------------------------
        // 3. Transform Ray to World Coordinates
        // ---------------------------------------------------------
        cv::Mat cvRotMat      = RpyToRotMat(cvCamRPY[0], cvCamRPY[1], cvCamRPY[2]);

        cv::Mat cvRayWorldMat = cvRotMat * cvRayCam;
        cv::Point3d cvRayDir(cvRayWorldMat.at<double>(0), cvRayWorldMat.at<double>(1), cvRayWorldMat.at<double>(2));

        // ---------------------------------------------------------
        // 4. Intersect Ray with Plane
        //    Math: t = (plane_pt - ray_origin) . plane_normal / (ray_dir . plane_normal)
        //   .
        // ---------------------------------------------------------
        cv::Point3d cvRayOrigin = cvCamPos;
        double dDenom           = cvRayDir.dot(stTargetPlane.cvNormal);

        // Check if parallel or very close to parallel to avoid division by zero.
        if (std::abs(dDenom) < 1e-12)
        {
            return std::nullopt;
        }

        double dT = stTargetPlane.cvNormal.dot(stTargetPlane.cvPoint - cvRayOrigin) / dDenom;

        // Check if intersection is behind the camera (negative t).
        if (dT <= 1e-9)
        {
            return std::nullopt;
        }

        // Calculate the absolute world intersection point.
        cv::Point3d cvIntersection = cvRayOrigin + (cvRayDir * dT);

        // Return the Offset (Intersection - Camera Position).
        return cvIntersection - cvCamPos;
    }
}    // namespace PixelTo3D

#endif
