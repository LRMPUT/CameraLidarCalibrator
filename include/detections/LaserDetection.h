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
#ifndef CAMERA_LIDAR_CALIBRATOR_LASERDETECTION_H
#define CAMERA_LIDAR_CALIBRATOR_LASERDETECTION_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <pcl/common/io.h>

class SingleCloudPoints {
public:

    std::vector <Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> selectedPoints;
    Eigen::Vector4d planeEq;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct Match {
    double angle;
    double pointSize;
    Eigen::Vector4d planeEq;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    pcl::PointIndices indices;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class LaserDetection {

public:

    LaserDetection () {
        min_pt.x = min_pt.y = min_pt.z = - 40;
        max_pt.x = max_pt.y = max_pt.z = 40;
        Eigen::Vector4d prevPlaneEq = Eigen::Vector4d::Zero();
        Eigen::Vector4f prevCentroid = Eigen::Vector4f::Zero();
        cloudIdx = 0;
    }

    void filterCustomVoxelGrid(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

    pcl::PointCloud<pcl::PointXYZI>::Ptr detect(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, Eigen::Vector4d &gtPlaneEq);



    pcl::PointCloud<pcl::PointXYZI>::Ptr process(std::vector <std::pair<double, pcl::PointCloud<pcl::PointXYZI>::Ptr>> &cloudBuffer);

    void verify(std::vector<SingleCloudPoints, Eigen::aligned_allocator<SingleCloudPoints>> &manualPlaneSelection);

    std::vector<SingleCloudPoints, Eigen::aligned_allocator<SingleCloudPoints>> gtDetections;

private:
    int cloudIdx = 0;
    pcl::PointXYZI min_pt, max_pt;
    Eigen::Vector4d prevPlaneEq;
    Eigen::Vector4f prevCentroid;
};

#endif //CAMERA_LIDAR_CALIBRATOR_LASERDETECTION_H
