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
#ifndef CAMERA_LIDAR_CALIBRATOR_CAMERADETECTIONS_H
#define CAMERA_LIDAR_CALIBRATOR_CAMERADETECTIONS_H

#include <Eigen/Eigen>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

class ChessboardDetection {
public:
    double timestamp;
    double timestampGlobal;
    Eigen::Vector4d planeEq;
    Eigen::Vector3d planeEqMinRepr;

    std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d>> border;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class CameraDetection {



public:

    CameraDetection(double &tS, double &atS, cv::Mat &cameraMatrix, cv::Mat distortionCoef) : timestampStart(tS),
        artifical_time_shift(atS), cameraMatrixInMat(cameraMatrix), distortionCoefficients(distortionCoef) {

    }

    bool detectChessboards(double timestamp, cv::Mat img, cv::Mat camera_matrix, cv::Mat distortion,
            ChessboardDetection &chessboardDetection);


    void read(const std::string &readChessboardPath, std::vector <std::pair<double, cv::Mat>> &imageBuffer,
            std::vector<ChessboardDetection> &chessboardDetections);

    void process(std::vector <std::pair<double, cv::Mat>> &imageBuffer, std::vector<ChessboardDetection> &chessboardDetections);

    void saveResult(std::string path, std::vector<ChessboardDetection> &chessboards) ;

private:

    const int boardHeight = 5;
    const int boardWidth = 8;
    const double CHESSBOARD_CORNER_SIZE = 0.1;
    const bool showDetections = false;

    double timestampStart;
    double artifical_time_shift;
    cv::Mat cameraMatrixInMat;
    cv::Mat distortionCoefficients;


};


#endif //CAMERA_LIDAR_CALIBRATOR_CAMERADETECTIONS_H
