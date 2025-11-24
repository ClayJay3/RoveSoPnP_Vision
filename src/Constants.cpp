/******************************************************************************
 * @brief Defines constants for the RoveSoPNP.
 *
 * @file AutonomyConstants.cpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-04-06
 *
 * @copyright Copyright RoveSoSeniorDesign 2025 - All Rights Reserved
 ******************************************************************************/

#include "./Constants.h"

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
    const std::string LOGGING_OUTPUT_PATH_ABSOLUTE = "../vision_logs/";           // The absolute path to write output logging and video files to.
    const quill::LogLevel CONSOLE_MIN_LEVEL        = quill::LogLevel::TraceL3;    // The minimum logging level that is allowed to send to the console log stream.
    const quill::LogLevel FILE_MIN_LEVEL           = quill::LogLevel::TraceL3;    // The minimum logging level that is allowed to send to the file log streams.
    const quill::LogLevel ROVECOMM_MIN_LEVEL       = quill::LogLevel::Info;       // The minimum logging level that is allowed to send to the RoveComm log stream.
    const quill::LogLevel CONSOLE_DEFAULT_LEVEL    = quill::LogLevel::Info;       // The default logging level for console stream.
    const quill::LogLevel FILE_DEFAULT_LEVEL       = quill::LogLevel::TraceL3;    // The default logging level for file streams.
    const quill::LogLevel ROVECOMM_DEFAULT_LEVEL   = quill::LogLevel::Info;       // The default logging level for RoveComm stream.

    // Logging color constants.
    const std::string szTraceL3Color   = "\033[30m";           // Standard Grey
    const std::string szTraceL2Color   = "\033[30m";           // Standard Grey
    const std::string szTraceL1Color   = "\033[30m";           // Standard Grey
    const std::string szDebugColor     = "\033[36m";           // Standard Cyan
    const std::string szInfoColor      = "\033[32m";           // Standard Green
    const std::string szNoticeColor    = "\033[97m\033[1m";    // Bright Bold White
    const std::string szWarningColor   = "\033[93m\033[1m";    // Bright Bold Yellow
    const std::string szErrorColor     = "\033[91m\033[1m";    // Bright Bold Red
    const std::string szCriticalColor  = "\033[95m\033[1m";    // Bright Bold Magenta
    const std::string szBacktraceColor = "\033[30m";           // Standard Grey

    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////
    //// Recording Handler Adjustments.
    ///////////////////////////////////////////////////////////////////////////

    // Recording adjustments.
    const int RECORDER_FPS = 15;    // The FPS all recordings should run at.
    // Camera recording toggles.
    const bool GANTRYCAM_ENABLE_RECORDING = true;    // Whether or not to record the gantry USB camera.

    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////
    //// Camera Constants.
    ///////////////////////////////////////////////////////////////////////////

    // BasicCam Basic Config.
    const cv::InterpolationFlags BASICCAM_RESIZE_INTERPOLATION_METHOD = cv::InterpolationFlags::INTER_LINEAR;    // The algorithm used to fill in pixels when resizing.

    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////
    //// Camera Handler Adjustments.
    ///////////////////////////////////////////////////////////////////////////

    // Ground Basic Cam.
    const int GANTRYCAM_RESOLUTIONX             = 1280;            // The horizontal pixel resolution to resize the basiccam images to.
    const int GANTRYCAM_RESOLUTIONY             = 720;             // The vertical pixel resolution to resize the basiccam images to.
    const int GANTRYCAM_FPS                     = 30;              // The FPS to use for the basiccam.
    const int GANTRYCAM_HORIZONTAL_FOV          = 110;             // The horizontal FOV of the camera. Useful for future calculations.
    const int GANTRYCAM_VERTICAL_FOV            = 70;              // The vertical FOV of the camera. Useful for future calculations.
    const int GANTRYCAM_FRAME_RETRIEVAL_THREADS = 5;               // The number of threads allocated to the threadpool for performing frame copies to other threads.
    const std::string GANTRYCAM_SERIAL          = "2022051301";    // The serial of the camera to open. You can get this by running: "lsusb -v | grep -i iSerial"
    const PIXEL_FORMATS GANTRYCAM_PIXELTYPE     = PIXEL_FORMATS::eBGR;    // The pixel layout of the camera.

    ///////////////////////////////////////////////////////////////////////////
}    // namespace constants
