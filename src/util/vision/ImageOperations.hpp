/******************************************************************************
 * @brief Defines and implements functions related to GENERAL operations on images. All
 *      functions are defined within the imgops namespace.
 *
 * @file ImageOperations.hpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-08-31
 *
 * @copyright Copyright RoveSoSeniorDesign 2025 - All Rights Reserved
 ******************************************************************************/

#ifndef IMAGE_OPERATIONS_HPP
#define IMAGE_OPERATIONS_HPP

#include "../../Logging.h"

/// \cond
#include <opencv2/opencv.hpp>

/// \endcond

/******************************************************************************
 * @brief Namespace containing functions related to GENERAL operations on images or other
 *      large binary operations.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-08-31
 ******************************************************************************/
namespace imgops
{
    /******************************************************************************
     * @brief Given a cv::Mat containing X, Y, Z, and BGRA values for each pixel in
     *      the third dimension, repackage the colors values and store them
     *      in a new mat.
     *
     * @param cvInputPointCloud - The input point cloud containing X, Y, Z, and BGRA values for each pixel.
     * @param cvOutputColors - The output colored image made from the BGRA values in the point cloud. Third dimension length is 4 (B, G, R, A)
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-10-05
     ******************************************************************************/
    inline void SplitPointCloudColors(cv::Mat& cvInputPointCloud, cv::Mat& cvOutputColors)
    {
        // Initialize output color mat if necessary.
        if (cvOutputColors.empty())
        {
            // Fill mat with zeros, same size as input point cloud, but with 4 char values in third dimension.
            cvOutputColors = cv::Mat(cvInputPointCloud.rows, cvInputPointCloud.cols, CV_8UC4, cv::Scalar(0));
        }
        // Check if the given mat has the correct dimensions.
        else if (cvOutputColors.rows != cvInputPointCloud.rows || cvOutputColors.cols != cvInputPointCloud.cols || cvOutputColors.channels() != 4)
        {
            // Submit logger error.
            LOG_ERROR(logging::g_qSharedLogger,
                      "SplitPointCloudColors: The given point cloud cv::Mat and output colors cv::Mat do not have the same dimensions! Dimensions are: (height = {} and "
                      "{}) (width = {} and {}) (channels = {} and {})",
                      cvInputPointCloud.rows,
                      cvOutputColors.rows,
                      cvInputPointCloud.cols,
                      cvOutputColors.cols,
                      cvInputPointCloud.channels(),
                      cvOutputColors.channels());
        }
        // Attempt to split colors from point cloud normally.
        else
        {
            // Loop through the color values of the point cloud and convert them to four char values instead of a float32.
            for (int nY = 0; nY < cvInputPointCloud.rows; ++nY)
            {
                for (int nX = 0; nX < cvInputPointCloud.cols; ++nX)
                {
                    // Get the current color value for the current pixel. Reinterpret case doesn't convert number, just copies bits.
                    unsigned int unColor = *reinterpret_cast<unsigned int*>(&cvInputPointCloud.at<cv::Vec4f>(nY, nX)[3]);
                    // Separate float32 BGRA values into four char values. Uses bitshift right and bitwise AND mask.
                    unsigned char ucB = (unColor >> 0) & 0xFF;
                    unsigned char ucG = (unColor >> 8) & 0xFF;
                    unsigned char ucR = (unColor >> 16) & 0xFF;
                    unsigned char ucA = (unColor >> 24) & 0xFF;

                    // Store char color values in the output mat.
                    cvOutputColors.at<cv::Vec4b>(nY, nX) = cv::Vec4b(ucB, ucG, ucR, ucA);
                }
            }
        }
    }

    /******************************************************************************
     * @brief Custom implementation of bilateral filter (since openCV is slow)
     *
     * @param cvInputFrame - cv::Mat of the image to filter
     * @param usDiameter - Diameter of kernel for aggregation of pixel values
     * @param dSigmaColor - Std.Dev for color weights
     * @param dSigmaSpace - Std.Dev for spatial weights
     *
     * @author Kai Shafe (kasq5m@umsystem.edu)
     * @date 2025-10-17
     ******************************************************************************/
    inline void CustomBilateralFilter(cv::Mat& cvInputFrame, ushort usDiameter, double dSigmaColor, double dSigmaSpace)
    {
        int rows = cvInputFrame.rows;
        int cols = cvInputFrame.cols;

        for (int y = 0; y < rows; y++)
        {
            for (int x = 0; x < cols; x++)
            {
                double sumIntensity = 0.0;
                double sumWeight    = 0.0;

                uchar centerPixel   = cvInputFrame.at<uchar>(y, x);

                for (int j = -usDiameter; j <= usDiameter; j++)
                {
                    for (int i = -usDiameter; i <= usDiameter; i++)
                    {
                        int neighborX = x + i;
                        int neighborY = y + j;

                        // Bound check
                        if (neighborX >= 0 && neighborX < cols && neighborY >= 0 && neighborY < rows)
                        {
                            uchar neighborPixel = cvInputFrame.at<uchar>(neighborY, neighborX);

                            // Calculate the spatial weight (Gaussian)
                            double spatialWeight = std::exp(-(i * i + j * j) / (2 * dSigmaSpace * dSigmaSpace));

                            // Calculate the intensity weight (Gaussian)
                            uchar colorDistance    = std::abs(centerPixel - neighborPixel);
                            double intensityWeight = std::exp(-(colorDistance * colorDistance) / (2 * dSigmaColor * dSigmaColor));

                            // Calculate the bilateral weight
                            double bilateralWeight = spatialWeight * intensityWeight;

                            sumIntensity += bilateralWeight * neighborPixel;
                            sumWeight += bilateralWeight;
                        }
                    }
                }

                // Calculate the filtered pixel value
                uchar filteredPixel          = sumIntensity / sumWeight;
                cvInputFrame.at<uchar>(y, x) = filteredPixel;
            }
        }
    }

    /******************************************************************************
     * @brief Given an image and a divisor, divide the precision of the elements.
     *
     * @param cvFrame - The frame to alter.
     * @param nDiv - The amount that the precision should be decreased.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2025-10-18
     ******************************************************************************/
    inline void ColorReduce(cv::Mat& cvFrame, int nDiv = 64)
    {
        // Number of lines.
        int nLines = cvFrame.rows;
        // Number of elements per line.
        int nElements = cvFrame.cols * cvFrame.channels();

        // Loop through each row of the image.
        for (int nI = 0; nI < nLines; nI++)
        {
            // Get the address of row j.
            uchar* chData = cvFrame.ptr<uchar>(nI);

            // Loop through the elements and decrease precision.
            for (int nJ = 0; nJ < nElements; nJ++)
            {
                // Process each pixel.
                chData[nJ] = chData[nJ] / nDiv * nDiv + nDiv / 2;
            }
        }
    }
}    // namespace imgops
#endif
