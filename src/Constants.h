/******************************************************************************
 * @brief Declares constants for the RoveSoPNP.
 *
 * @file Constants.h
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-04-06
 *
 * @copyright Copyright RoveSoSeniorDesign 2025 - All Rights Reserved
 ******************************************************************************/

#ifndef AUTONOMY_CONSTANTS_H
#define AUTONOMY_CONSTANTS_H

#include "./interfaces/Camera.hpp"

/// \cond
#include <opencv2/opencv.hpp>
#include <quill/core/LogLevel.h>

/// \endcond

/******************************************************************************
 * @brief Namespace containing all constants for RoveSoPNP. Including
 *      AutonomyGlobals.h will also include this namespace.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2025-08-05
 ******************************************************************************/
namespace constants
{
    ///////////////////////////////////////////////////////////////////////////
    //// General Constants.
    ///////////////////////////////////////////////////////////////////////////

    // Logging constants.
    extern const std::string LOGGING_OUTPUT_PATH_ABSOLUTE;
    extern const quill::LogLevel CONSOLE_MIN_LEVEL;
    extern const quill::LogLevel FILE_MIN_LEVEL;
    extern const quill::LogLevel ROVECOMM_MIN_LEVEL;
    extern const quill::LogLevel CONSOLE_DEFAULT_LEVEL;
    extern const quill::LogLevel FILE_DEFAULT_LEVEL;
    extern const quill::LogLevel ROVECOMM_DEFAULT_LEVEL;

    // Logging color constants.
    extern const std::string szTraceL3Color;
    extern const std::string szTraceL2Color;
    extern const std::string szTraceL1Color;
    extern const std::string szDebugColor;
    extern const std::string szInfoColor;
    extern const std::string szNoticeColor;
    extern const std::string szWarningColor;
    extern const std::string szErrorColor;
    extern const std::string szCriticalColor;
    extern const std::string szBacktraceColor;
    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////
    //// Recording Handler Adjustments.
    ///////////////////////////////////////////////////////////////////////////

    // Recording adjustments.
    extern const int RECORDER_FPS;
    // Camera recording toggles.
    extern const bool GANTRYCAM_ENABLE_RECORDING;
    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////
    //// Camera Constants.
    ///////////////////////////////////////////////////////////////////////////

    // BasicCam Basic Config.
    extern const cv::InterpolationFlags BASICCAM_RESIZE_INTERPOLATION_METHOD;
    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////
    //// Camera Handler Adjustments.
    ///////////////////////////////////////////////////////////////////////////

    // Gantry Cam.
    extern const int GANTRYCAM_RESOLUTIONX;
    extern const int GANTRYCAM_RESOLUTIONY;
    extern const int GANTRYCAM_FPS;
    extern const int GANTRYCAM_HORIZONTAL_FOV;
    extern const int GANTRYCAM_VERTICAL_FOV;
    extern const int GANTRYCAM_FRAME_RETRIEVAL_THREADS;
    extern const int GANTRYCAM_INDEX;
    extern const PIXEL_FORMATS GANTRYCAM_PIXELTYPE;
    ///////////////////////////////////////////////////////////////////////////
} // namespace constants

#endif // AUTONOMY_CONSTANTS_H
