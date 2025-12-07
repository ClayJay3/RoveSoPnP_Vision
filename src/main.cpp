/******************************************************************************
 * @brief Main program file. Sets up classes and runs main program functions.
 *
 * @file main.cpp
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2025-06-20
 *
 * @copyright Copyright RoveSoSeniorDesign 2025 - All Rights Reserved
 ******************************************************************************/

#include "./Logging.h"
#include "vision/algorithms/CameraCalibration.hpp"
#include "vision/algorithms/ComponentDetector.hpp"
#include "vision/algorithms/FicucialDetector.hpp"
#include "vision/algorithms/PixelTo3D.hpp"
#include "vision/algorithms/PlacementVerifier.hpp"
#include "vision/algorithms/VisualHoming.hpp"
#include "vision/cameras/BasicCam.h"

/// \cond
#include <iostream>
#include <sys/ioctl.h>
#include <termios.h>
#include <vector>

/// \endcond

// Create a boolean used to handle a SIGINT and exit gracefully.
volatile sig_atomic_t bMainStop = false;
// Store original terminal settings.
struct termios g_stOriginalTermSettings;

/******************************************************************************
 * @brief Help function given to the C++ csignal standard library to run when
 * a CONTROL^C is given from the terminal.
 *
 * @param nSignal - Integer representing the interrupt value.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-01-08
 ******************************************************************************/
void SignalHandler(int nSignal)
{
    // Check signal type.
    if (nSignal == SIGINT || nSignal == SIGTERM)
    {
        // Submit logger message.
        LOG_INFO(logging::g_qSharedLogger, "Ctrl+C or SIGTERM received. Cleaning up...");

        // Update stop signal.
        bMainStop = true;
    }
    // The SIGQUIT signal can be sent to the terminal by pressing CNTL+\.
    else if (nSignal == SIGQUIT)
    {
        // Submit logger message.
        LOG_INFO(logging::g_qSharedLogger, "Quit signal key pressed. Cleaning up...");

        // Update stop signal.
        bMainStop = true;
    }
}

/******************************************************************************
 * @brief Reset terminal mode to original settings.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-04-04
 ******************************************************************************/
void ResetTerminalMode()
{
    tcsetattr(STDIN_FILENO, TCSANOW, &g_stOriginalTermSettings);
}

/******************************************************************************
 * @brief Mutator for the Non Canonical Terminal Mode private member.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-04-04
 ******************************************************************************/
void SetNonCanonicalTerminalMode()
{
    struct termios stNewTermSettings;

    tcgetattr(STDIN_FILENO, &g_stOriginalTermSettings);
    std::memcpy(&stNewTermSettings, &g_stOriginalTermSettings, sizeof(struct termios));

    stNewTermSettings.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &stNewTermSettings);

    atexit(ResetTerminalMode);
}

/******************************************************************************
 * @brief Check if a key has been pressed in the terminal.
 *
 * @return int - Number of bytes waiting in the terminal buffer.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-04-04
 ******************************************************************************/
int CheckKeyPress()
{
    int nBytesWaiting;
    ioctl(STDIN_FILENO, FIONREAD, &nBytesWaiting);
    return nBytesWaiting;
}

/******************************************************************************
 * @brief  main function.
 *
 * @return int - Exit status number.
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2025-06-20
 ******************************************************************************/
int main()
{
    // Initialize Loggers
    logging::InitializeLoggers(constants::LOGGING_OUTPUT_PATH_ABSOLUTE);

    /////////////////////////////////////////
    // Setup global objects.
    /////////////////////////////////////////
    // Setup signal interrupt handler.
    struct sigaction stSigBreak;
    stSigBreak.sa_handler = SignalHandler;
    stSigBreak.sa_flags   = 0;
    sigemptyset(&stSigBreak.sa_mask);
    sigaction(SIGINT, &stSigBreak, nullptr);
    sigaction(SIGQUIT, &stSigBreak, nullptr);
    // Set the terminal to non-canonical mode. This allows us to read a single character from the terminal without waiting for a newline.
    SetNonCanonicalTerminalMode();

    /////////////////////////////////////////
    // Declare local variables used in main loop.
    /////////////////////////////////////////
    // Get Camera and Tag detector pointers .
    IPS IterPerSecond = IPS();

    // Declare and initialize cameras.
    std::unique_ptr<BasicCam> pGantryCam = std::make_unique<BasicCam>(constants::GANTRYCAM_SERIAL,
                                                                      constants::GANTRYCAM_RESOLUTIONX,
                                                                      constants::GANTRYCAM_RESOLUTIONY,
                                                                      constants::GANTRYCAM_FPS,
                                                                      PIXEL_FORMATS::eBGRA,
                                                                      constants::GANTRYCAM_HORIZONTAL_FOV,
                                                                      constants::GANTRYCAM_VERTICAL_FOV,
                                                                      true,
                                                                      constants::GANTRYCAM_FRAME_RETRIEVAL_THREADS);

    // Create a vector of ints to store the FPS values for each thread.
    std::vector<uint32_t> vThreadFPSValues;
    // Vector to store frames for calibration.
    std::vector<cv::Mat> vCalibrationImages;

    // Start the camera.
    pGantryCam->Start();

    /*
        This while loop is the main periodic loop for the RoveSoPNP program.
        Loop until user sends sigkill or sigterm.
    */
    while (!bMainStop)
    {
        // Add each threads FPS value to the vector.
        vThreadFPSValues.clear();
        vThreadFPSValues.push_back(static_cast<uint32_t>(IterPerSecond.GetExactIPS()));
        vThreadFPSValues.push_back(static_cast<uint32_t>(pGantryCam->GetIPS().GetExactIPS()));

        // Create a string to append FPS values to.
        std::string szMainInfo = "";
        // Get FPS of all cameras and detectors and construct the info into a string.
        szMainInfo += "--------[ Threads FPS ]--------\n";
        szMainInfo += "Main Process FPS: " + std::to_string(IterPerSecond.GetExactIPS()) + "\n";
        szMainInfo += "Gantry Camera FPS: " + std::to_string(pGantryCam->GetIPS().GetExactIPS()) + "\n";

        // Submit logger message.
        LOG_DEBUG(logging::g_qSharedLogger, "{}", szMainInfo);

        // Print out the FPS stats to the console if the user presses 'f' or 'F'.
        if (CheckKeyPress() > 0)
        {
            char chTerminalInput = 0;
            ssize_t nBytesRead   = read(STDIN_FILENO, &chTerminalInput, 1);
            if (nBytesRead <= 0)
            {
                LOG_WARNING(logging::g_qSharedLogger, "Failed to read from terminal input.");
            }
            else
            {
                if (chTerminalInput == 'h' || chTerminalInput == 'H')
                {
                    // Print help message to console.
                    LOG_NOTICE(logging::g_qSharedLogger,
                               "\n--------[ RoveSoPNP Help ]--------\n"
                               "Press 'f' or 'F' to print FPS stats to the log file.\n"
                               "Press 'c' or 'C' to capture a frame for calibration.\n"
                               "Press 'x' or 'X' to execute calibration (interactive).\n"
                               "Press 'q' or 'Q' to quit the program.\n"
                               "-------------------------------------------\n");
                }
                else if (chTerminalInput == 'f' || chTerminalInput == 'F')
                {
                    LOG_NOTICE(logging::g_qSharedLogger, "{}", szMainInfo);
                }
                else if (chTerminalInput == 'q' || chTerminalInput == 'Q')
                {
                    LOG_INFO(logging::g_qSharedLogger, "'Q' key pressed. Initiating shutdown...");
                    bMainStop = true;
                }
                // Capture frame logic.
                else if (chTerminalInput == 'c' || chTerminalInput == 'C')
                {
                    cv::Mat cvFrame;
                    // Request a frame copy from the camera thread.
                    std::future<bool> fResult = pGantryCam->RequestFrameCopy(cvFrame);

                    // Wait for the frame to be copied.
                    if (fResult.get())
                    {
                        // Display frame.
                        cv::imshow("Calibration Frame", cvFrame);
                        cv::waitKey(1000);
                        // Store the frame.
                        vCalibrationImages.push_back(cvFrame.clone());
                        LOG_INFO(logging::g_qSharedLogger, "Captured calibration frame. Total images: {}", vCalibrationImages.size());
                    }
                    else
                    {
                        LOG_ERROR(logging::g_qSharedLogger, "Failed to capture frame from Gantry Camera.");
                    }
                }
                // Execute calibration logic.
                else if (chTerminalInput == 'x' || chTerminalInput == 'X')
                {
                    if (vCalibrationImages.empty())
                    {
                        LOG_WARNING(logging::g_qSharedLogger, "No images captured! Press 'c' to capture images first.");
                    }
                    else
                    {
                        // Temporarily reset terminal to canonical mode to allow standard user input.
                        ResetTerminalMode();

                        int nBoardWidth   = 9;
                        int nBoardHeight  = 6;
                        float fSquareSize = 25.0f;

                        // Ask for Board Dimensions.
                        std::cout << "\n--- Calibration Configuration ---\n";
                        std::cout << "Enter Board Width (number of inner corners, e.g., 9): ";
                        std::cin >> nBoardWidth;
                        std::cout << "Enter Board Height (number of inner corners, e.g., 6): ";
                        std::cin >> nBoardHeight;
                        std::cout << "Enter Square Size in mm (e.g., 25.0): ";
                        std::cin >> fSquareSize;

                        // Ask if user wants fisheye or normal calibration.
                        std::cout << "Select Calibration Model: (P)inhole or (F)isheye? ";
                        char chModeInput;
                        std::cin >> chModeInput;

                        DistortionModel eModel = PINHOLE;
                        if (chModeInput == 'f' || chModeInput == 'F')
                        {
                            eModel = FISHEYE;
                            LOG_INFO(logging::g_qSharedLogger, "Selected FISHEYE model.");
                        }
                        else
                        {
                            LOG_INFO(logging::g_qSharedLogger, "Selected PINHOLE model.");
                        }

                        LOG_INFO(logging::g_qSharedLogger, "Starting Calibration on {} images with board {}x{}...", vCalibrationImages.size(), nBoardWidth, nBoardHeight);

                        // Perform Calibration.
                        cv::Size cvBoardSize(nBoardWidth, nBoardHeight);
                        CameraConfig stResult = PixelTo3D::CalibrateCameraFromImages(vCalibrationImages, cvBoardSize, fSquareSize, eModel);

                        // Print out the result calibration.
                        std::cout << "\n========== Calibration Results ==========\n";
                        std::cout << "Camera Matrix (K):\n" << stResult.cvK << "\n\n";
                        std::cout << "Distortion Coefficients (D):\n" << stResult.cvD << "\n";
                        std::cout << "=========================================\n" << std::endl;

                        // Clear images after calibration.
                        vCalibrationImages.clear();
                        LOG_INFO(logging::g_qSharedLogger, "Calibration finished. Image buffer cleared.");

                        // Return terminal to non-canonical mode for main loop execution.
                        SetNonCanonicalTerminalMode();
                    }
                }
            }
        }

        // Update IPS tick.
        IterPerSecond.Tick();

        // No need to loop as fast as possible. Sleep...
        std::this_thread::sleep_for(std::chrono::microseconds(66666));
    }

    /////////////////////////////////////////
    // Cleanup.
    /////////////////////////////////////////

    // Submit logger message that program is done cleaning up and is now exiting.
    LOG_INFO(logging::g_qSharedLogger, "Clean up finished. Exiting...");

    // Successful exit.
    return 0;
}
