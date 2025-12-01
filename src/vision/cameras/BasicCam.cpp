/******************************************************************************
 * @brief Implements the BasicCam class.
 *
 * @file BasicCam.cpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-08-19
 *
 * @copyright Copyright RoveSoSeniorDesign 2025 - All Rights Reserved
 ******************************************************************************/

#include "BasicCam.h"
#include "../../Constants.h"
#include "../../Logging.h"

#ifdef _WIN32
#include <dshow.h>
#include <windows.h>
#pragma comment(lib, "strmiids")
#pragma comment(lib, "ole32")
#pragma comment(lib, "oleaut32")
#else
#include <algorithm>
#include <dirent.h>
#include <fstream>
#endif

/******************************************************************************
 * @brief Construct a new Basic Cam:: Basic Cam object.
 *
 * @param szCameraSerial - The file path to the camera hardware.
 * @param nPropResolutionX - X res of camera.
 * @param nPropResolutionY - Y res of camera.
 * @param nPropFramesPerSecond - FPS camera is running at.
 * @param ePropPixelFormat - The pixel layout/format of the image.
 * @param dPropHorizontalFOV - The horizontal field of view.
 * @param dPropVerticalFOV - The vertical field of view.
 * @param bEnableRecordingFlag - Whether or not this camera should be recorded.
 * @param nNumFrameRetrievalThreads - The number of threads to use for frame queueing and copying.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-08-20
 ******************************************************************************/
BasicCam::BasicCam(const std::string szCameraSerial,
                   const int nPropResolutionX,
                   const int nPropResolutionY,
                   const int nPropFramesPerSecond,
                   const PIXEL_FORMATS ePropPixelFormat,
                   const double dPropHorizontalFOV,
                   const double dPropVerticalFOV,
                   const bool bEnableRecordingFlag,
                   const int nNumFrameRetrievalThreads) :
    BasicCamera(szCameraSerial,
                nPropResolutionX,
                nPropResolutionY,
                nPropFramesPerSecond,
                ePropPixelFormat,
                dPropHorizontalFOV,
                dPropVerticalFOV,
                bEnableRecordingFlag,
                nNumFrameRetrievalThreads)
{
    // Initialize the OpenCV mat to a black/empty image the size of the camera resolution.
    m_cvFrame = cv::Mat::zeros(nPropResolutionY, nPropResolutionX, CV_8UC4);

    // Set video cvCamera properties.
    m_cvCamera.set(cv::CAP_PROP_FRAME_WIDTH, nPropResolutionX);
    m_cvCamera.set(cv::CAP_PROP_FRAME_HEIGHT, nPropResolutionY);
    m_cvCamera.set(cv::CAP_PROP_FPS, nPropFramesPerSecond);

    // Attempt to open camera with OpenCV's VideoCapture and print if successfully opened or not.
    if (m_cvCamera.open(szCameraSerial))
    {
        // Submit logger message.
        LOG_INFO(logging::g_qSharedLogger, "Camera {} at {} has been successfully opened.", m_cvCamera.getBackendName(), szCameraSerial);
    }
    else
    {
        // Submit logger message.
        LOG_ERROR(logging::g_qSharedLogger, "Unable to open camera at {}", szCameraSerial);
    }

    // Check if recording is enabled.
    if (bEnableRecordingFlag)
    {
        // Create the path for the camera where we will store out video output file.
        std::filesystem::path szFilePath;
        std::filesystem::path szFilenameWithExtension;
        szFilePath = constants::LOGGING_OUTPUT_PATH_ABSOLUTE;                 // Main location for all recordings.
        szFilePath += logging::g_szProgramStartTimeString + "/recordings";    // Folder for each program run.
        szFilenameWithExtension = this->GetCameraLocation() + ".mkv";         // Folder for each camera index or name.

        // Check if directory exists.
        if (!std::filesystem::exists(szFilePath))
        {
            // Create directory.
            if (!std::filesystem::create_directories(szFilePath))
            {
                // Submit logger message.
                LOG_ERROR(logging::g_qSharedLogger,
                          "Unable to create the VideoWriter output directory: {} for camera {}",
                          szFilePath.string(),
                          this->GetCameraLocation());
            }
        }

        // Construct the full output path.
        std::filesystem::path szFullOutputPath = szFilePath / szFilenameWithExtension;
        // Initialize the VideoWriter object.
        m_cvVideoWriter.open(szFullOutputPath.string(), cv::VideoWriter::fourcc('H', '2', '6', '4'), nPropFramesPerSecond, this->GetPropResolution());
    }

    // Set max FPS of the ThreadedContinuousCode method.
    this->SetMainThreadIPSLimit(nPropFramesPerSecond);
}

/******************************************************************************
 * @brief Destroy the Basic Cam:: Basic Cam object.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-08-20
 ******************************************************************************/
BasicCam::~BasicCam()
{
    // Close the video writer if recording was enabled.
    if (m_bEnableRecordingFlag)
    {
        m_cvVideoWriter.release();
    }

    // Stop threaded code.
    this->RequestStop();
    this->Join();

    // Release camera capture object.
    m_cvCamera.release();

    // Submit logger message.
    LOG_INFO(logging::g_qSharedLogger, "Basic camera at path/URL {} has been successfully closed.", m_szCameraPath);
}

/******************************************************************************
 * @brief The code inside this private method runs in a separate thread, but still
 *      has access to this*. This method continuously get new frames from the OpenCV
 *      VideoCapture object and stores it in a member variable. Then a thread pool is
 *      started and joined once per iteration to mass copy the frames and/or measure
 *      to any other thread waiting in the queues.
 *
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-09-16
 ******************************************************************************/
void BasicCam::ThreadedContinuousCode()
{
    // Check if camera is NOT open.
    if (!m_cvCamera.isOpened())
    {
        // If this is the first iteration of the thread the camera probably isn't present so stop thread to save resources.
        if (this->GetThreadState() == ThreadState::eStarting)
        {
            // Shutdown threads for this BasicCam.
            this->RequestStop();

            // Submit logger message.
            LOG_CRITICAL(logging::g_qSharedLogger, "Camera start was attempted for BasicCam at {}/{}, but camera was never opened!", m_nCameraIndex, m_szCameraPath);
        }
        else
        {
            // Create instance variables.
            bool bCameraReopened                  = false;
            static bool bReopenAlreadyChecked     = false;
            std::chrono::time_point tmCurrentTime = std::chrono::system_clock::now();
            // Convert time point to seconds since epoch
            int nTimeSinceEpoch = std::chrono::duration_cast<std::chrono::seconds>(tmCurrentTime.time_since_epoch()).count();

            // Only try to reopen camera every 5 seconds.
            if (nTimeSinceEpoch % 5 == 0 && !bReopenAlreadyChecked)
            {
                // Check if camera was opened with an index or path.
                if (m_nCameraIndex == -1)
                {
                    // Attempt to reopen camera.
                    bCameraReopened = m_cvCamera.open(m_szCameraPath);
                }
                else
                {
                    // Attempt to reopen camera.
                    bCameraReopened = m_cvCamera.open(m_nCameraIndex);
                }

                // Check if camera was reopened.
                if (bCameraReopened)
                {
                    // Submit logger message.
                    LOG_INFO(logging::g_qSharedLogger, "Camera {}/{} has been reconnected and reopened!", m_nCameraIndex, m_szCameraPath);
                }
                else
                {
                    // Submit logger message.
                    LOG_WARNING(logging::g_qSharedLogger, "Attempt to reopen Camera {}/{} has failed! Trying again in 5 seconds...", m_nCameraIndex, m_szCameraPath);
                    // Sleep for five seconds.
                }

                // Set toggle.
                bReopenAlreadyChecked = true;
            }
            else if (nTimeSinceEpoch % 5 != 0)
            {
                // Reset toggle.
                bReopenAlreadyChecked = false;
            }
        }
    }
    else
    {
        // Check if new frame was computed successfully.
        if (m_cvCamera.read(m_cvFrame))
        {
            // Resize the frame.
            cv::resize(m_cvFrame, m_cvFrame, cv::Size(m_nPropResolutionX, m_nPropResolutionY), 0.0, 0.0, constants::BASICCAM_RESIZE_INTERPOLATION_METHOD);

            // Check if the recording flag is enabled.
            if (m_bEnableRecordingFlag)
            {
                // Write the current frame to the video writer.
                m_cvVideoWriter.write(m_cvFrame);
            }
        }
        else
        {
            // Submit logger message.
            LOG_ERROR(logging::g_qSharedLogger, "Unable to read new frame for camera {}, {}! Closing camera...", m_nCameraIndex, m_szCameraPath);
            // Release camera capture.
            m_cvCamera.release();

            // Fill camera frame member variable with zeros. This ensures a non-corrupt, black image.
            m_cvFrame = cv::Mat::zeros(m_nPropResolutionY, m_nPropResolutionX, CV_8UC3);
        }
    }

    // Acquire a shared_lock on the frame copy queue.
    std::shared_lock<std::shared_mutex> lkSchedulers(m_muPoolScheduleMutex);
    // Check if the frame copy queue is empty.
    if (!m_qFrameCopySchedule.empty())
    {
        // Start the thread pool to store multiple copies of the sl::Mat into the given cv::Mats.
        this->RunDetachedPool(m_qFrameCopySchedule.size(), m_nNumFrameRetrievalThreads);
        // Wait for thread pool to finish.
        this->JoinPool();
        // Release lock on frame copy queue.
        lkSchedulers.unlock();
    }
}

/******************************************************************************
 * @brief This method holds the code that is ran in the thread pool started by
 *      the ThreadedLinearCode() method. It copies the data from the different
 *      data objects to references of the same type stored in a vector queued up by the
 *      Grab methods.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-09-16
 ******************************************************************************/
void BasicCam::PooledLinearCode()
{
    // Acquire mutex for getting frames out of the queue.
    std::unique_lock<std::shared_mutex> lkFrameQueue(m_muFrameCopyMutex);
    // Check if the queue is empty.
    if (!m_qFrameCopySchedule.empty())
    {
        // Get frame container out of queue.
        containers::FrameFetchContainer<cv::Mat> stContainer = m_qFrameCopySchedule.front();
        // Pop out of queue.
        m_qFrameCopySchedule.pop();
        // Release lock.
        lkFrameQueue.unlock();

        // Copy frame to data container.
        *stContainer.pFrame = m_cvFrame.clone();
        // Signal future that the frame has been successfully retrieved.
        stContainer.pCopiedFrameStatus->set_value(true);
    }
    else
    {
        // Release lock.
        lkFrameQueue.unlock();
    }
}

/******************************************************************************
 * @brief Puts a frame pointer into a queue so a copy of a frame from the camera can be written to it.
 *      Remember, this code will be ran in whatever, class/thread calls it.
 *
 * @param cvFrame - A reference to the cv::Mat to store the frame in.
 * @return std::future<bool> - A future that should be waited on before the passed in frame is used.
 *                          Value will be true if frame was successfully retrieved.
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2025-09-09
 ******************************************************************************/
std::future<bool> BasicCam::RequestFrameCopy(cv::Mat& cvFrame)
{
    // Assemble the FrameFetchContainer.
    containers::FrameFetchContainer<cv::Mat> stContainer(cvFrame, m_ePropPixelFormat);

    // Acquire lock on frame copy queue.
    std::unique_lock<std::shared_mutex> lkScheduler(m_muPoolScheduleMutex);
    // Append frame fetch container to the schedule queue.
    m_qFrameCopySchedule.push(stContainer);
    // Release lock on the frame schedule queue.
    lkScheduler.unlock();

    // Return the future from the promise stored in the container.
    return stContainer.pCopiedFrameStatus->get_future();
}

/******************************************************************************
 * @brief Accessor for the camera open status.
 *
 * @return true - The camera has been successfully opened.
 * @return false - The camera has not been successfully opened.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-08-20
 ******************************************************************************/
bool BasicCam::GetCameraIsOpen()
{
    // Get camera status from OpenCV.
    return this->GetThreadState() == ThreadState::eRunning && m_cvCamera.isOpened();
}

/******************************************************************************
 * @brief Accessor for the cameras path or video index.
 *
 * @return std::string - The path or index of the camera.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-08-20
 ******************************************************************************/
std::string BasicCam::GetCameraLocation() const
{
    // If video path, return path string.
    // If the camera path is a file path, only return the last part of the path for easier identification. (file name)
    size_t lastSlashPos = m_szCameraPath.find_last_of("/\\");
    if (lastSlashPos != std::string::npos)
    {
        return m_szCameraPath.substr(lastSlashPos + 1);
    }
    else
    {
        // Otherwise return a string with a random number.
        return "video_index_" + std::to_string(rand() % 1000);
    }
}
