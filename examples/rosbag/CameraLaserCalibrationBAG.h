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
#ifndef CAMERA_LIDAR_CALIBRATOR_CAMERALASERCALIBRATIONBAG_H
#define CAMERA_LIDAR_CALIBRATOR_CAMERALASERCALIBRATIONBAG_H


#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/eigen.hpp>
#include <sensor_msgs/image_encodings.h>

#include <iostream>
#include <vector>
#include <algorithm>
#include <iterator>
#include <assert.h>
#include <cmath>

#include <chrono>

#include <std_msgs/UInt8.h>

#include "CameraLaserCalibration.h"



class CameraLaserCalibrationBAG : public CameraLaserCalibration {
public:

    CameraLaserCalibrationBAG();

    // ROS
    ros::NodeHandle nh;

    // Subscribers
    ros::Subscriber subLaserCloud, subRvizAction;
    image_transport::Subscriber subImage;

    // Publishers
//    ros::Publisher pubLaserCloud;
//    image_transport::Publisher pubImage;

    // Image transport & tf buffer
    image_transport::ImageTransport imageTransport;

    // (timestamp, cloud)
    std::vector<std::pair<double, pcl::PointCloud<pcl::PointXYZI>::Ptr>> cloudBuffer;

    // (timestamp, image)
    std::vector<std::pair<double, cv::Mat>> imageBuffer;


    std::string TAG;

    template<class T>
    T readParameter(ros::NodeHandle &nh, std::string name, T defaultValue) {
        T value;
        if (nh.getParam(name.c_str(), value))
            ROS_INFO_STREAM(TAG << name << " : " << value);
        else {
            ROS_ERROR_STREAM(TAG << "No value for " << name << " set -- default equal to " << value);
            value = defaultValue;
        }
        return value;
    }


public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif //CAMERA_LIDAR_CALIBRATOR_CAMERALASERCALIBRATIONROS_H