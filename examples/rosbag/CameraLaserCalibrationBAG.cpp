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
#include "CameraLaserCalibrationBAG.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>

CameraLaserCalibrationBAG::CameraLaserCalibrationBAG() : nh("~"), imageTransport(nh), CameraLaserCalibration() {
    TAG = "CameraLaserCalibrationBAG";

    std::string path = ros::package::getPath("camera_lidar_calibrator");

    // Reading from launch file
    std::string sensor_type = readParameter<std::string> (nh, "sensor_type", "MRS6124");
    std::string rosbag_path = path + "/" + readParameter<std::string> (nh, "bag_path", "***.bag");
    bool read_countours_from_file = readParameter<bool> (nh, "read_countours_from_file", 0);
    std::string read_contours_path = path + "/" + readParameter<std::string> (nh, "read_countours_path", "");
    bool read_chessboards_from_file = readParameter<bool> (nh, "read_chessboards_from_file", 0);
    std::string read_chessboard_path = path + "/" + readParameter<std::string> (nh, "read_chessboards_path", "");

    std::string camera_img = readParameter<std::string> (nh, "camera_topic",  "/mv_FF001589/image_raw/compressed");

    fx = readParameter<double> (nh, "fx", 0.0);
    fy = readParameter<double> (nh, "fy", 0.0);
    cx = readParameter<double> (nh, "cx", 0.0);
    cy = readParameter<double> (nh, "cy", 0.0);

    d0 = readParameter<double> (nh, "d0", 0.0);
    d1 = readParameter<double> (nh, "d1", 0.0);
    d2 = readParameter<double> (nh, "d2", 0.0);
    d3 = readParameter<double> (nh, "d3", 0.0);


    unsigned int cloudEveryN = readParameter<int> (nh, "cloud_every_n", 1);
    unsigned int imageEveryN = readParameter<int> (nh, "image_every_n", 1);

    timeShiftFixed = readParameter<bool> (nh, "time_shift_fixed", 0);

    std::cout << "timeShiftFixed = " << timeShiftFixed << std::endl;

    double artificialTimeShift = readParameter<double> (nh, "artifical_time_shift", 0.0);

    bool inteligentEdgeSelection = readParameter<bool> (nh, "intelligent_edge_selection", false);
    double maxEdgeNumber = readParameter<int> (nh, "max_edge_number", 10000);

    int increasePointNumber = readParameter<int> (nh, "increase_point_number", 0);

    setSensorType(sensor_type);

    cameraMatrix = Eigen::Matrix3d::Identity();
    cameraMatrix(0, 0) = fx;
    cameraMatrix(0, 2) = cx;
    cameraMatrix(1, 1) = fy;
    cameraMatrix(1, 2) = cy;
    cv::eigen2cv(cameraMatrix, cameraMatrixInMat);

    // Distortion coefficients
    distortionCoefficients = cv::Mat(1, 5, CV_32F);
    distortionCoefficients.at<float>(0, 0) = d0;
    distortionCoefficients.at<float>(0, 1) = d1;
    distortionCoefficients.at<float>(0, 2) = d2;
    distortionCoefficients.at<float>(0, 3) = d3;
    distortionCoefficients.at<float>(0, 4) = 0.0;

    // Reading data

    rosbag::Bag bag;
    bag.open(rosbag_path, rosbag::bagmode::Read);

    std::string mrs_cloud = "/sick_mrs6xxx/cloud_single_echo";
    std::string velo_cloud = "/velodyne_points";

    // Which topics to read from rosbag
    std::vector<std::string> topics;
    topics.push_back(mrs_cloud);
    topics.push_back(velo_cloud);
    topics.push_back(camera_img);

    rosbag::View view(bag, rosbag::TopicQuery(topics));
    unsigned int cloudId = 0, imageId = 0;
    for (auto it = view.begin(); it != view.end(); ++it)
    {

        rosbag::MessageInstance const & m  = *it;

        if (m.getTopic() == mrs_cloud || m.getTopic() == velo_cloud)
        {
            sensor_msgs::PointCloud2::ConstPtr cloudMsg = m.instantiate<sensor_msgs::PointCloud2>();
            if (cloudMsg != NULL && cloudId % cloudEveryN == 0)
            {
                // Publishes
                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZI>);
                pcl::fromROSMsg(*cloudMsg, *cloud);
                cloudBuffer.push_back(std::make_pair(cloudMsg->header.stamp.toSec(), cloud));
            }
            cloudId++;
        }


        else if (m.getTopic() == camera_img)
        {

            sensor_msgs::CompressedImageConstPtr imageMsg = m.instantiate<sensor_msgs::CompressedImage>();
            if (imageMsg != NULL && imageId % imageEveryN == 0)
            {
                cv::Mat image = cv::imdecode(cv::Mat(imageMsg->data),1);
                imageBuffer.push_back(std::make_pair(imageMsg->header.stamp.toSec(), image));
            }
            imageId++;
        }

    }
    bag.close();

    // TODO: Using up to 340 clouds
    cloudBuffer.resize(340);

    std::cout << "Image buf size : " << imageBuffer.size() << std::endl;
    std::cout << "Image cloud size : " << cloudBuffer.size() << std::endl;


    calibrate(cloudBuffer, imageBuffer, read_countours_from_file, read_contours_path,
              read_chessboards_from_file, read_chessboard_path,
              artificialTimeShift,
            increasePointNumber, inteligentEdgeSelection, maxEdgeNumber);

//    verify(cloudBuffer, imageBuffer);

    cloudBuffer.clear();
    imageBuffer.clear();
}
