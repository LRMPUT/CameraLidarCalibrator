/*
 * BSD 3-Clause License
 *
 * This file is part of the CameraLidarCalibrator distribution (https://github.com/LRMPUT/CameraLidarCalibrator).
 * Copyright (c) 2020, Michał Nowicki (michal.nowicki@put.poznan.pl)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef CAMERA_LIDAR_CALIBRATOR_CAMERALASERCALIBRATION_H
#define CAMERA_LIDAR_CALIBRATOR_CAMERALASERCALIBRATION_H

#include <Eigen/Eigen>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


#include <iostream>
#include <vector>
#include <algorithm>
#include <iterator>
#include <assert.h>
#include <cmath>
#include <numeric>
#include <string>
#include <functional>

#include "Optimization.h"

struct Contour {
    double timestamp;
    double px[4], py[4];
};

class CameraLaserCalibration {

public:
    CameraLaserCalibration();

    void setSensorType(std::string sensor_type);

    //
    static void onMouse(int event, int x, int y, int, void *laserImageClass);

    void onMouse(int event, int x, int y);

    //
    void calibrate( std::vector<std::pair<double, pcl::PointCloud<pcl::PointXYZI>::Ptr>> & cloudBuffer,
                    std::vector<std::pair<double, cv::Mat>> & imageBuffer,
                    bool readMarkedPlanes = false,
                    std::string readContoursPath = "",
                    bool readChessboard = false,
                    std::string readChessboardPath = "",
                    double artifical_time_shift = 0.0,
                    bool increasePointNumber = 0,
                    bool intelligentEdgeSelection = false,
                    unsigned int maxEdgeNumber = 10000) ;

    void planeFitting(std::vector  <Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &selectedPoints, double &rmseErr, Eigen::Vector4d &planeEq);

    std::vector<Contour> readContours(std::string readContoursPath);

    void saveCameraChessboardSpeed(Optimization &optimizationProblem);
    void saveLidarChessboardSpeed(Optimization &optimizationProblem);

    void verify( std::vector<std::pair<double, pcl::PointCloud<pcl::PointXYZI>::Ptr>> & cloudBuffer,
                    std::vector<std::pair<double, cv::Mat>> & imageBuffer);

    double estimateRelativeTime(pcl::PointXYZI &point, float &startOri, float &endOri, int &row, bool &halfPassed);

protected:

    // Camera parameters
    cv::Mat cameraMatrixInMat;
    Eigen::Matrix3d cameraMatrix;
    cv::Mat distortionCoefficients;

    // Camera matrix
    double fx, fy, cx, cy, d0, d1, d2, d3;

    // Parameters typical for MRS6xxx
    std::string sensor_type;
    int SCANNER_COLS, SCANNER_ROWS;
    double SCAN_PERIOD, UPPER_BOUND, LOWER_BOUND, VELO_FACTOR;


    // Optimization param
    bool timeShiftFixed;

    const int DEPTH_SCALING_VALUE = 30;



    bool recording;

    cv::Mat depthMarkImage;
    std::vector<cv::Point> contour;

    cv::Rect2d roi;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

#endif //CAMERA_LIDAR_CALIBRATOR_CAMERALASERCALIBRATION_H
