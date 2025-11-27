/******************************************************************************
 * @brief Defines camera models and related utilities.
 *
 * @file CameraModels.hpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-11-24
 *
 * @copyright Copyright RoveSoSeniorDesign 2025 - All Rights Reserved
 ******************************************************************************/

#ifndef CAMERA_MODELS_HPP
#define CAMERA_MODELS_HPP

#include <opencv2/opencv.hpp>

/******************************************************************************
 * @brief Enum to define which mathematical model to use for undistortion.
 ******************************************************************************/
enum DistortionModel
{
    PINHOLE,
    FISHEYE
};

/******************************************************************************
 * @brief Configuration struct containing intrinsic camera parameters.
 ******************************************************************************/
struct CameraConfig
{
    public:
        DistortionModel eModel = PINHOLE;
        cv::Mat cvK;                  // Intrinsic Matrix (3x3).
        cv::Mat cvD;                  // Distortion Coefficients.
        cv::Size cvFisheyeOrigDim;    // Required only for Fisheye scaling (from DIM in json).
};

/******************************************************************************
 * @brief Struct defining a 3D plane in the world.
 ******************************************************************************/
struct Plane
{
    public:
        cv::Point3d cvPoint  = {0, 0, 0};
        cv::Point3d cvNormal = {0, 0, 1};    // Default: Ground plane (Z up).
};

#endif    // CAMERA_MODELS_HPP
