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
#ifndef CAMERA_LIDAR_CALIBRATOR_OPTIMIZATION_H
#define CAMERA_LIDAR_CALIBRATOR_OPTIMIZATION_H

#include<Eigen/StdVector>
#include <opencv2/opencv.hpp>

#include "../EXTERNAL/g2o/g2o/core/sparse_optimizer.h"

#include "detections/CameraDetection.h"

class PointStamped {
public:
    double timestamp;
    Eigen::Vector3d point;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class ConvexityResult {
public:
    double time;
    unsigned int goodEdgesCount;
    double avgChi2;
};

class Optimization {
public:
    Optimization() {
        ratioOfTotal = 1.00;
    }

    // Method to determine the final transformation
    void optimizeCameraLaser(double & estimatedTimeShift,
                             Eigen::Matrix4d & estimatedLaserInCamera,
                             double &rmse,
                             int &good, int &total,  std::vector<ConvexityResult> & convex,
                             bool timeShiftFixed);

    // All detected chessboards
    std::vector<ChessboardDetection> chessboards;

    // All selected lidar points belonging to the chessboard plane
    std::vector<PointStamped,Eigen::aligned_allocator<PointStamped>> lidarPoints;

    // Method used to determine the chessboard location based on interpolated detections
    bool findApproximatedChessboard(double pointTimestamp, Eigen::Vector3d &chessboardMinRepr, Eigen::Vector3d &jacobianN,
            double &jacobianD);

    double setRatioOfTotal(double _ratio) {
        ratioOfTotal = _ratio;
    }

private:


    void randomEdgeSelection(double ratio);

    g2o::SparseOptimizer optimizer;

    double ratioOfTotal;



public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};






#endif //CAMERA_LIDAR_CALIBRATOR_OPTIMIZATION_H
