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
#include "CameraLaserCalibration.h"
#include <Eigen/StdVector>
#include <ros/package.h>
#include <fstream>

#include <opencv2/core/utility.hpp>
//#include <opencv2/tracking.hpp>

#include "utils/colormap.h"
#include "utils/BSpline.h"
#include "local_g2o/minimal_plane.h"

#include "detections/CameraDetection.h"
#include "detections/LaserDetection.h"

CameraLaserCalibration::CameraLaserCalibration() {

    std::cout << "READY!" << std::endl;
}

void CameraLaserCalibration::setSensorType(std::string _sensor_type) {
    sensor_type = _sensor_type;
    if(sensor_type == "VLP16") {
        SCANNER_COLS = 1824;
        SCANNER_ROWS = 16;
        SCAN_PERIOD = 0.1;
        UPPER_BOUND = 15;
        LOWER_BOUND = -15;
        VELO_FACTOR = (SCANNER_ROWS - 1) / (UPPER_BOUND - LOWER_BOUND);
    }
    else if (sensor_type == "MRS6124") {
        SCANNER_COLS = 924;
        SCANNER_ROWS = 24;
        SCAN_PERIOD = 0.1;
    }
    else {
        std::cout << "Unrecognized sensor type!" << std::endl;
        exit(0);
    }

}

void CameraLaserCalibration::onMouse(int event, int x, int y, int, void *laserImageClass) {
    if (event != cv::EVENT_LBUTTONDOWN)
        return;

    CameraLaserCalibration *laserImage = reinterpret_cast<CameraLaserCalibration *>(laserImageClass);
    laserImage->onMouse(event, x, y);

    return;
}


void CameraLaserCalibration::onMouse(int event, int x, int y) {
    circle(depthMarkImage, cv::Point(x, y), 4, cv::Scalar(255, 255, 255), CV_FILLED, 8, 0);
    contour.push_back(cv::Point(x, y));


    std::cout << "contour = " << contour.size() << std::endl;

    // If we have 4 points
    if (contour.size() == 4) {

        // Convex hull of these points
        std::vector <std::vector<cv::Point>> hull(1);
        cv::convexHull(contour, hull[0]);

        cv::drawContours(depthMarkImage, hull, 0, 255);

        cv::waitKey(1);
        cv::destroyWindow("Depth image");
    }


    imshow("Depth image", depthMarkImage);
}


void
CameraLaserCalibration::calibrate(std::vector <std::pair<double, pcl::PointCloud<pcl::PointXYZI>::Ptr>> &cloudBuffer,
                                  std::vector <std::pair<double, cv::Mat>> &imageBuffer,
                                  bool readMarkedPlanes,
                                  std::string readContoursPath,
                                  bool readChessboard,
                                  std::string readChessboardPath,
                                  double artifical_time_shift,
                                  bool increasePointNumber,
                                  bool intelligentEdgeSelection,
                                  unsigned int maxEdgeNumber) {

    std::cout << "-->  artifical_time_shift = " << artifical_time_shift << std::endl;
    std::string path = ros::package::getPath("camera_lidar_calibrator");

    // Just to visualize for article
    double visTimeOffset = 0.0;
    bool showProjection = false;

    // Testing camera - laser
    Optimization optimizationProblem;

    // Time relatie to the start
    double cloudBufferTimeStart = cloudBuffer[0].first;
    double chessboardBufferTimeStart = imageBuffer[0].first;

    double timestampStart = std::min(cloudBufferTimeStart, chessboardBufferTimeStart);

    // Reading or loading chessboard detections
    CameraDetection cameraDetection(timestampStart, artifical_time_shift, cameraMatrixInMat, distortionCoefficients);
    if (readChessboard)
        cameraDetection.read(readChessboardPath, imageBuffer, optimizationProblem.chessboards);
    else
        cameraDetection.process(imageBuffer, optimizationProblem.chessboards);
    cameraDetection.saveResult(path, optimizationProblem.chessboards);

    // Saving chessboard speed
    saveCameraChessboardSpeed(optimizationProblem);




    std::vector<Contour> contours;
    std::fstream contourStr;

    // Either read or save contours
    if(readMarkedPlanes)
        contours = readContours(readContoursPath);
    else
        contourStr.open(path + "/log/cloudContoursWrite.txt", std::ios::out);

    // For each cloud
    cv::Mat lastDepthIntImgScaled;
    cv::Mat prevConsideredPoints;

    // FOR COMPARISON
    std::vector<SingleCloudPoints, Eigen::aligned_allocator<SingleCloudPoints>> manualPlaneSelection;

    // Reading or loading laser on chessboard detections
    // https://bitbucket.org/unizg-fer-lamor/calirad/src/master/lidar_utils/src/polygon_tracker.cpp
    LaserDetection laserDetection;

    for (int cloudIdx=0;cloudIdx<cloudBuffer.size();cloudIdx++) {

        // Selected timestamp and cloud
        double cloudTimestamp = cloudBuffer[cloudIdx].first - timestampStart;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = cloudBuffer[cloudIdx].second;

        std::cout << "Processing cloud: " << cloudIdx + 1 << " / " << cloudBuffer.size() << ": " << cloudTimestamp
                  << std::endl;

        float startOri = -std::atan2(cloud->points[0].y, cloud->points[0].x);
        float endOri = -std::atan2(cloud->points[cloud->points.size() - 1].y,
                                   cloud->points[cloud->points.size() - 1].x) + 2 * float(M_PI);
        if (endOri - startOri > 3 * M_PI) {
            endOri -= 2 * M_PI;
        } else if (endOri - startOri < M_PI) {
            endOri += 2 * M_PI;
        }


        // Create a range image
        cv::Mat depthImg = cv::Mat(SCANNER_ROWS, SCANNER_COLS, CV_32FC1);
        depthImg = 0;
        cv::Mat timeShiftImg = cv::Mat(SCANNER_ROWS, SCANNER_COLS, CV_32FC1);
        timeShiftImg = 0;

        // For each point
        bool halfPassed = false;
        std::map<std::pair<int, int>, int> rowcol2cloud;

        for (int cloudPointIdx = 0; cloudPointIdx < cloud->points.size(); cloudPointIdx++) {

            // Is it a proper measurement
            if (!pcl_isfinite(cloud->points[cloudPointIdx].x) ||
                !pcl_isfinite(cloud->points[cloudPointIdx].y) ||
                !pcl_isfinite(cloud->points[cloudPointIdx].z)) {
                continue;
            }

            // Is it far enough from the scanner
            float val = sqrt(cloud->points[cloudPointIdx].x * cloud->points[cloudPointIdx].x +
                             cloud->points[cloudPointIdx].y * cloud->points[cloudPointIdx].y +
                             cloud->points[cloudPointIdx].z * cloud->points[cloudPointIdx].z);
            if (val < 0.5 || val > 50) {
                continue;
            }

            int row = 0, col = 0;

            if (sensor_type == "MRS6124") {
                row = cloudPointIdx / SCANNER_COLS;
                col = cloudPointIdx % SCANNER_COLS;
            } else if (sensor_type == "VLP16") {
                auto &point = cloud->points[cloudPointIdx];
                float angle = std::atan(point.z / std::sqrt(point.x * point.x + point.y * point.y));
                row = SCANNER_ROWS - int(((angle * 180 / M_PI) - LOWER_BOUND) * VELO_FACTOR + 0.5) - 1;
                //col = cloudPointIdx / SCANNER_ROWS;
                float angleHor = std::atan2(point.y, point.x); // from -PI to PI
                if (angleHor < -M_PI)
                    angleHor += 2 * M_PI;
                if (angleHor >= M_PI)
                    angleHor -= 2 * M_PI;
                col = (angleHor + M_PI) * SCANNER_COLS * 1.0 / (2 * M_PI);
            }


            // Save mapping for the future
            rowcol2cloud[std::make_pair(row, col)] = cloudPointIdx;

            // Filling depth image
            depthImg.at<float>(row, col) = val;

            // Estimate time:
            // - from driver (real time) stored in intensity channel
            // - estimate it based on sensor operation
            double relTime = estimateRelativeTime(cloud->points[cloudPointIdx], startOri, endOri, row, halfPassed);
//            double relTime = cloud->points[cloudPointIdx].intensity;

            // Save time shift for a point
            timeShiftImg.at<float>(row, col) = relTime;

            // Let's fill intensity with relative time
            cloud->points[cloudPointIdx].intensity = relTime;
        }

        // float image to SCALED integer image
        cv::Mat depthIntImg, depthIntImgScaled;
        depthImg.convertTo(depthIntImg, CV_8UC1, DEPTH_SCALING_VALUE);
        cv::resize(depthIntImg, depthIntImgScaled, cv::Size(), 1, 20);

        depthMarkImage = depthIntImgScaled.clone();

        contour.clear();

        // Read from file
        if (readMarkedPlanes) {

            bool found = false;
            for (int rcIdx = 0; rcIdx < contours.size(); rcIdx++) {
                if (fabs(contours[rcIdx].timestamp - cloudBuffer[cloudIdx].first) < 0.001) {
                    found = true;

                    for (int pIdx = 0; pIdx < 4; pIdx++)
                        contour.push_back(cv::Point(contours[rcIdx].px[pIdx], contours[rcIdx].py[pIdx] * 20));
                    break;
                }
            }

            if (!found)
                continue;
        }
        else
        {
            depthMarkImage = depthIntImgScaled;
            cv::imshow("Depth image", depthIntImgScaled);
            cv::setMouseCallback("Depth image", onMouse, this);
            cv::waitKey(0);
        }

//        else if (cloudIdx == 0 || true)
//        {
////            cv::Mat trackingImg =  imageBuffer[cloudIdx].second; // TODO: TO TEST TRACKING
////            depthMarkImage = trackingImg.clone(); // TODO: TO TEST TRACKING
//

//
//
//            // Top left (min x and min y)
//            int minX = contour[0].x, minY = contour[0].y;
//            int maxX = contour[0].x, maxY = contour[0].y;
//            for (int i=1;i<contour.size();i++) {
//                if (minX > contour[i].x)
//                    minX = contour[i].x;
//                if (minY > contour[i].y)
//                    minY = contour[i].y;
//                if (maxX < contour[i].x)
//                    maxX = contour[i].x;
//                if (maxY < contour[i].y)
//                    maxY = contour[i].y;
//            }
//            roi = cv::Rect2d(minX,minY, maxX - minX, maxY-minY);
//
////            cv::Mat trackingImg = depthIntImgScaled;
////            cv::cvtColor(depthIntImgScaled, trackingImg, cv::COLOR_GRAY2BGR);
//
////            tracker->init(trackingImg,roi);
//
////            cv::rectangle( depthIntImgScaled, roi, cv::Scalar( 255, 0, 0 ), 2, 1 );
////            imshow("Depth image",depthIntImgScaled);
////            cv::waitKey(0);
//        }
//        else if (true) {
//
//
//            // update the tracking result
////            cv::Rect2d roi;
//            cv::Mat trackingImg = imageBuffer[cloudIdx].second; // TODO: TO TEST TRACKING
////            cv::cvtColor(depthIntImgScaled, trackingImg, cv::COLOR_GRAY2BGR);
//
////            bool trackerValue = tracker->update(trackingImg,roi);
////            std::cout << "trackerValue : " << trackerValue << std::endl;
//
//            // draw the tracked object
//            cv::rectangle( trackingImg, roi, cv::Scalar( 255, 0, 0 ), 2, 1 );
//
//            // show image with the tracked object
//            imshow("tracker",trackingImg);
//            cv::waitKey(0);
//        }
////             Track last marking
//        else {
//
//            // Finding the center of previous polygon
//            double centerRow = 0, centerCol = 0, numberOfPoints = 0;
//            for (int i = 0; i < SCANNER_COLS; i++) {
//                for (int j = 0; j < SCANNER_ROWS; j++) {
//
//                    if (prevConsideredPoints.at<unsigned char>(j, i) > 0) {
//                        centerRow += j;
//                        centerCol += i;
//                        numberOfPoints++;
//                    }
//                }
//            }
//            centerRow = centerRow / numberOfPoints;
//            centerCol = centerCol / numberOfPoints;
//            std::cout << "Center: " << centerRow << " " << centerCol << " | " << numberOfPoints << std::endl;
//
//            // Finding distances for all previous points
//            std::vector<double> rowDistToCenter, colDistToCenter;
//            for (int i = 0; i < SCANNER_COLS; i++) {
//                for (int j = 0; j < SCANNER_ROWS; j++) {
//
//                    if (prevConsideredPoints.at<unsigned char>(j, i) > 0) {
//
//                        double imageDist = std::sqrt((j-centerRow) * (j-centerRow) );
//                        rowDistToCenter.push_back(imageDist);
//                        imageDist = std::sqrt((i-centerCol)*(i-centerCol));
//                        colDistToCenter.push_back(imageDist);
//                    }
//                }
//            }
//            std::sort(rowDistToCenter.begin(), rowDistToCenter.end());
//            std::sort(colDistToCenter.begin(), colDistToCenter.end());
//
//            // Setting threshold for now
//            double threshold = 0.6;
//            double rowDist = rowDistToCenter[threshold * numberOfPoints];
//            double colDist = colDistToCenter[threshold * numberOfPoints];
//
//
//            contour.push_back(cv::Point(centerCol - colDist, 20*(centerRow - rowDist)));
//            contour.push_back(cv::Point(centerCol + colDist, 20*(centerRow - rowDist)));
//            contour.push_back(cv::Point(centerCol - colDist, 20*(centerRow + rowDist)));
//            contour.push_back(cv::Point(centerCol + colDist, 20*(centerRow + rowDist)));
//
////            for (int i = 0; i < SCANNER_COLS; i++) {
////                for (int j = 0; j < SCANNER_ROWS; j++) {
////                    if (prevConsideredPoints.at<unsigned char>(j, i) > 0) {
////
////                        double imageDist = std::sqrt((j-centerRow) * (j-centerRow) + (i-centerCol)*(i-centerCol));
//////                        if (imageDist < 20) {
////                            contour.push_back(cv::Point(i,j * 20));
//////                        }
////                    }
////                }
////            }
//
//            std::vector <std::vector<cv::Point>> drawhull(1);
//            cv::convexHull(contour, drawhull[0]);
//
//            for (int x = 0 ; x < drawhull[0].size() ; x++ )
//            {
//                std::cout << "drawhull: " << drawhull[0][x].x << " " << drawhull[0][x].y << std::endl;
//            }
//
//
//            cv::drawContours(depthIntImgScaled, drawhull, 0, 255);
//
//            cv::imshow("Depth image", depthIntImgScaled);
//            cv::waitKey(0);
//
//
//
//        }

        lastDepthIntImgScaled = depthIntImgScaled.clone();
//        cv::waitKey(0);

        std::cout << "contour.size() = " << contour.size() << std::endl;

        if (contour.size() < 4)
            continue;

        cv::Mat depthImgIntScaledRGB;
        cv::cvtColor(depthIntImgScaled, depthImgIntScaledRGB, cv::COLOR_GRAY2BGR);

        std::vector<std::vector<cv::Point>> drawhull(1);
        cv::convexHull(contour, drawhull[0]);
        cv::drawContours(depthIntImgScaled, drawhull, 0, 255);

        if (sensor_type == "MRS6124")
            cv::drawContours(depthImgIntScaledRGB, drawhull, 0, CV_RGB(0, 255, 0));
        else if (sensor_type == "VLP16")
            cv::drawContours(depthImgIntScaledRGB, drawhull, 0, CV_RGB(255, 0, 0));

        std::vector<cv::Point2f> contourFloatPoints;
        for (auto &p : contour) {
            contourFloatPoints.push_back(cv::Point2f(p.x, p.y / 20.0));
        }


        // Saving contour points
        if (!readMarkedPlanes) {
            contourStr << std::fixed << std::setprecision(6) << cloudBuffer[cloudIdx].first << " ";
            for (auto &p : contourFloatPoints) {
                contourStr << p.x << " " << p.y << " ";
            }
            contourStr << std::endl;
        }


        std::vector<std::vector<cv::Point2f>> hull(1);
        cv::convexHull(contourFloatPoints, hull[0]);

        // Selected points with corresponding time shifts
        std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> selectedPoints;
        std::vector<double> selectedPointsTimeIncrement;

        // Going throught the 2D range image and finding the points inside polygon
        cv::Mat consideredPoints = cv::Mat(SCANNER_ROWS, SCANNER_COLS, CV_8UC1);
        consideredPoints = 0;

        // Going through all rows and cols
        for (int i = 0; i < SCANNER_COLS; i++) {
            for (int j = 0; j < SCANNER_ROWS; j++) {

                // Distance to convex hull: val > 0 -> inside
                double val = cv::pointPolygonTest(hull[0], cv::Point2f(i, j), false);

                // Inside and proper depth measurement
                if (val > 0 && depthImg.at<float>(j, i) > 0) {

                    // Getting the point index from row & col
                    int index = rowcol2cloud[std::make_pair(j, i)];

                    // Selected 3D point
                    Eigen::Vector4d p = Eigen::Vector4d(cloud->points[index].x,
                                                        cloud->points[index].y,
                                                        cloud->points[index].z, 1.0);
                    selectedPoints.push_back(p);

                    // Time increment
                    selectedPointsTimeIncrement.push_back(timeShiftImg.at<float>(j, i));

                    // Mark point as selected
                    consideredPoints.at<unsigned char>(j, i) = 255;

                    // Visualization
                    circle(depthIntImgScaled, cv::Point(i, j * 20), 1, 255);

                    if (sensor_type == "MRS6124")
                        circle(depthImgIntScaledRGB, cv::Point(i, j * 20), 1, CV_RGB(0, 255, 0));
                    else if (sensor_type == "VLP16")
                        circle(depthImgIntScaledRGB, cv::Point(i, j * 20), 1, CV_RGB(255, 0, 0));

//                    // Visualization on original image
//                    {
//                        Eigen::Vector4d pCam = visLidarInCamera * p;
//                        std::vector <cv::Point3d> framePoints;
//                        std::vector <cv::Point2d> imageFramePoints;
//                        framePoints.push_back(cv::Point3d(pCam(0), pCam(1), pCam(2)));
//                        cv::Mat rvec = cv::Mat(cv::Size(3, 1), CV_64F), tvec = cv::Mat(cv::Size(3, 1), CV_64F);
//                        tvec = 0, rvec = 0;
//                        cv::projectPoints(framePoints, rvec, tvec, cameraMatrixInMat, distortionCoefficients,
//                                          imageFramePoints);
//
//                        double pGlobalTime = cloudBuffer[cloudIdx].first + timeShiftImg.at<float>(j, i) + visTimeOffset;
//
//                        for (int imgIdx = 0; imgIdx < imageBuffer.size(); imgIdx++) {
//                            if (fabs(imageBuffer[imgIdx].first - pGlobalTime) < 0.08)
//                                if (showProjection)
//                                    cv::circle(imageBuffer[imgIdx].second, (cv::Point) imageFramePoints[0], 2, CV_RGB(255, 0, 0));
//                        }
//
//                    }
                }
                // Just visualize // TODO: Stupid code
//                if ( depthImg.at<float>(j, i) > 0 ) {
//                    int index = rowcol2cloud[std::make_pair(j,i)];
//                    Eigen::Vector4d p = Eigen::Vector4d(cloud->points[index].x,
//                                                        cloud->points[index].y,
//                                                        cloud->points[index].z, 1.0);
//                    Eigen::Vector4d pCam = visLidarInCamera * p;
//                    std::vector <cv::Point3d> framePoints;
//                    std::vector <cv::Point2d> imageFramePoints;
//                    framePoints.push_back(cv::Point3d(pCam(0), pCam(1), pCam(2)));
//                    cv::Mat rvec = cv::Mat(cv::Size(3, 1), CV_64F), tvec = cv::Mat(cv::Size(3, 1), CV_64F);
//                    tvec = 0, rvec = 0;
//                    cv::projectPoints(framePoints, rvec, tvec, cameraMatrixInMat, distortionCoefficients,
//                                      imageFramePoints);
//
//                    double pGlobalTime = cloudBuffer[cloudIdx].first + timeShiftImg.at<float>(j, i) + visTimeOffset;
//
//                    for (int imgIdx = 0; imgIdx < imageBuffer.size(); imgIdx++) {
//                        if (fabs(imageBuffer[imgIdx].first - pGlobalTime) < 0.08)
//                            if (showProjection)
//                                cv::circle(imageBuffer[imgIdx].second, (cv::Point) imageFramePoints[0], 2, CV_RGB(0, 255, 0));
//                    }
//
//                }
            }
        }
        prevConsideredPoints = consideredPoints;

        std::cout << "Number of selectedPoints: " << selectedPoints.size() << std::endl;
        if (selectedPoints.size() == 0)
            continue;

        // Plane fitting to marked points from 3D lidar
        double rmseErr;
        Eigen::Vector4d planeEq;
        planeFitting(selectedPoints, rmseErr, planeEq);

        /// Saving to compare to laserDetection
        SingleCloudPoints scp;
        scp.selectedPoints = selectedPoints;
        scp.planeEq = planeEq;
        manualPlaneSelection.push_back(scp);

        /// Automatic Laser Detection
//        pcl::PointCloud<pcl::PointXYZI>::Ptr automaticSelectedPoints = laserDetection.detect(cloud, planeEq);

        // Uncomment when marking manually
//        if (!readMarkedPlanes) {
//            cv::imshow("Depth image", depthIntImgScaled);
//            cv::waitKey(0);
//        }

//        // Saving for video an image of chessboard and range image
//        {
//            int bestIdx = 0;
//            double cloudTime = cloudBuffer[cloudIdx].first + visTimeOffset;
//            for (int imgIdx = 0; imgIdx < imageBuffer.size(); imgIdx++) {
//                if (fabs(imageBuffer[imgIdx].first - cloudTime) < fabs(imageBuffer[bestIdx].first - cloudTime))
//                    bestIdx = imgIdx;
//            }
//
//
//            cv::Mat imgRGB = imageBuffer[bestIdx].second, imgRGBScaled;
//
//            // FIT TO DEPTH
//            cv::resize(imgRGB, imgRGBScaled, cv::Size(depthIntImgScaled.cols,depthIntImgScaled.rows));
//            cv::Mat out = cv::Mat(depthIntImgScaled.cols, depthIntImgScaled.rows * 2, CV_8UC3);
//            cv::vconcat(imgRGBScaled, depthImgIntScaledRGB, out);
//
//            // FIT TO RGB
////            cv::Mat depthImgIntScaledRGB2;
////            cv::resize(depthImgIntScaledRGB, depthImgIntScaledRGB2, cv::Size(imgRGB.cols,imgRGB.rows));
////            cv::Mat out = cv::Mat(imgRGB.cols, imgRGB.rows * 2, CV_8UC3);
////            cv::vconcat(imgRGB, depthImgIntScaledRGB2, out);
//
////            std::cout << "vconcat done!" << std::endl;
//
//            std::string indexAsString = std::to_string(cloudIdx);
//            cv::imwrite(path + "/visReal/img" + std::string(5 - indexAsString.length(), '0') + indexAsString + ".png", out);
//        }

//        std::cout << "\t Manual vs Automation : " << selectedPoints.size()  << " vs " << automaticSelectedPoints->size() << std::endl;
//        for (int i=0;i<200;i++) {
//            std::cout << selectedPointsTimeIncrement[i] << " " << automaticSelectedPoints->points[i].intensity << std::endl;
//        }

        // Adding all selected points (from markings)
        for (int pointIdx = 0; pointIdx < selectedPoints.size(); pointIdx++) {

            PointStamped ps;
            ps.timestamp = cloudTimestamp + selectedPointsTimeIncrement[pointIdx];
            ps.point = Eigen::Vector3d(selectedPoints[pointIdx].x(), selectedPoints[pointIdx].y(),
                                         selectedPoints[pointIdx].z());

            optimizationProblem.lidarPoints.push_back(ps);
        }

        // Adding all selected points (from automatic detection)
//        for (int pointIdx = 0; pointIdx < automaticSelectedPoints->points.size(); pointIdx++) {
//
//            PointStamped ps;
//            ps.timestamp = cloudTimestamp + automaticSelectedPoints->points[pointIdx].intensity;
//            ps.point = automaticSelectedPoints->points[pointIdx].getArray3fMap().cast<double>();
//
//            optimizationProblem.lidarPoints.push_back(ps);
//        }

        // SAVE time shift visualization
//        {
//            std::string indexAsString = std::to_string(cloudIdx);
//
//            cv::Mat out, outScaled;
//            cv::normalize(timeShiftImg, out, 0, 255, cv::NORM_MINMAX, CV_8UC1);
//            cv::resize(out, outScaled, cv::Size(400, 400));
//            cv::applyColorMap(outScaled, out, cv::COLORMAP_JET);
//            cv::imwrite(
//                    path + "/time/imgTimeShift" + std::string(5 - indexAsString.length(), '0') + indexAsString + ".png",
//                    out);
//
//            cv::Mat outDepth, outDepth2;
////        cv::resize(depthIntImg, outDepth, cv::Size(400,400));
//            cv::resize(depthIntImg, outDepth, cv::Size(), 1, 20);
////        cv::applyColorMap(outDepth, outDepth2, cv::COLORMAP_BONE);
//            cv::imwrite(path + "/time/depth" + std::string(5 - indexAsString.length(), '0') + indexAsString + ".png",
//                        outDepth);
//        }
    }
    contourStr.close();

    // Plane rotation speed from laser
    saveLidarChessboardSpeed(optimizationProblem);


    /// Compare lidar detections with manual marks
//    laserDetection.verify(manualPlaneSelection);
//    pcl::PointCloud<pcl::PointXYZI>::Ptr laserAutomaticSelectedPoints = laserDetection.process(cloudBuffer);
//
//    std::cout << " ---------------- " << std::endl;
////    std::cout << "laserAutomaticSelectedPoints.size() = " << laserAutomaticSelectedPoints->size() << std::endl;
//    std::cout << "optimizationProblem.lidarPoints.size() = " << optimizationProblem.lidarPoints.size() << std::endl;
//    for (int testIdx = 0; testIdx < 100; testIdx ++) {
//        std::cout << "testIdx = " << testIdx << " | relTime = " << laserAutomaticSelectedPoints->points[testIdx].intensity << std::endl;
//    }


//    exit(0);

    std::cout << "optimizeCameraLaser()" << std::endl;

    // Calibration!
    double estimatedTimeShift = 0.0;
    double rmse;
    int good, total;
    std::vector<ConvexityResult> convex;

    // Pose -> laser pose in camera -> TODO Initial guess!
    Eigen::Matrix4d estimatedLaserInCamera = Eigen::Matrix4d::Identity();

    // MRS setup
    if(sensor_type == "VLP16") {
        estimatedLaserInCamera.block<3, 1>(0, 0) = Eigen::Vector3d::UnitZ();
        estimatedLaserInCamera.block<3, 1>(0, 1) = -Eigen::Vector3d::UnitX();
        estimatedLaserInCamera.block<3, 1>(0, 2) = -Eigen::Vector3d::UnitY();
        estimatedLaserInCamera.block<3, 1>(0, 3) = Eigen::Vector3d(0.0, 0.0, 0.0);
    }
    else if (sensor_type == "MRS6124") {
        // TODO: Let's check if the problem is not with Lie Algebra
        estimatedLaserInCamera.block<3, 1>(0, 0) = Eigen::Vector3d::UnitZ();
        estimatedLaserInCamera.block<3, 1>(0, 1) = Eigen::Vector3d::UnitX();
        estimatedLaserInCamera.block<3, 1>(0, 2) = Eigen::Vector3d::UnitY();
        estimatedLaserInCamera.block<3, 1>(0, 3) = Eigen::Vector3d(0.17, 0.0, 0.0); // LEFT
//        estimatedLaserInCamera.block<3, 1>(0, 3) = Eigen::Vector3d(-0.17, 0.0, 0.0); // RIGHT
        estimatedTimeShift = -0.2; // INITIAL TIME = 0.0
    }


    /// Let's do it 100 times for not and cmp results
//    optimizationProblem.setRatioOfTotal(0.35);
    optimizationProblem.setRatioOfTotal(1.0);
    for (int optIdx = 0; optIdx < 1; optIdx ++) {
        // Optimization!
        optimizationProblem.optimizeCameraLaser(estimatedTimeShift, estimatedLaserInCamera, rmse, good, total,
                                                convex, timeShiftFixed);


        // Saving result
        std::ofstream resultStr;
        resultStr.open(path + "/log/result" + std::to_string(optIdx) + ".txt");
        if (resultStr.is_open()) {
            resultStr << estimatedTimeShift << std::endl << std::endl;
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    resultStr << estimatedLaserInCamera(i, j) << " ";
                }
                resultStr << std::endl;
            }

            resultStr << std::endl << rmse << " " << good << " " << total << std::endl;
            resultStr.close();
        }
    }

    // Is it convex?
    std::ofstream convexStr;
    convexStr.open(path + "/log/convex.txt");
    if (convexStr.is_open())
    {
        for (auto &cr : convex)
            convexStr << cr.time << " " << cr.goodEdgesCount << " " << cr.avgChi2 << std::endl;
        convexStr.close();
    }


    // Show some projections
    std::cout << "Save img" << std::endl;
    for (int i = 0; i < imageBuffer.size(); i++) {
        std::string indexAsString = std::to_string(i);
        cv::imwrite(path + "log/img" + std::string(5 - indexAsString.length(), '0') + indexAsString + ".png", imageBuffer[i].second);
    }


}

void CameraLaserCalibration::planeFitting(std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &selectedPoints, double &rmseErr, Eigen::Vector4d &planeEq) {

    // Fitting a plane to point in camera coordinate system
    Eigen::MatrixXd points = Eigen::MatrixXd::Zero(4, selectedPoints.size());

    for (int i = 0; i < selectedPoints.size(); i++) {
        points.col(i) = selectedPoints[i];
    }
    planeEq = Eigen::Vector4d::Zero();

    // Computing mean
    Eigen::Vector4d mean = Eigen::Vector4d::Zero();
    for (int j = 0; j < points.cols(); j++)
        mean += points.col(j);
    mean /= points.cols();

    // Points without mean
    Eigen::MatrixXd demeanPts = points;
    for (int j = 0; j < demeanPts.cols(); ++j)
        demeanPts.col(j) -= mean;

    // Covariance
    Eigen::Matrix3d covariance = demeanPts.topRows<3>() * demeanPts.topRows<3>().transpose();

    // EigenValue
    Eigen::SelfAdjointEigenSolver <Eigen::Matrix3d> evd(covariance);

    // Plane equation
    planeEq.head<3>() = evd.eigenvectors().col(0);
    planeEq(3) = -planeEq.head<3>().dot(mean.head<3>());

    // Let's check how well these points fit
    rmseErr = 0;
    double minErr = 99999, maxErr = 0;
    for (int i = 0; i < selectedPoints.size(); i++) {
        double err = planeEq.transpose() * selectedPoints[i];
        rmseErr += err * err;
        if (err > maxErr)
            maxErr = err;
        if (err < minErr)
            minErr = err;
    }
    rmseErr = rmseErr / selectedPoints.size();
    rmseErr = sqrt(rmseErr);
    std::cout << "RMSE error : " << rmseErr << " m" << std::endl;
    std::cout << "Max error : " << maxErr << " m" << std::endl;
    std::cout << "Min error : " << minErr << " m" << std::endl;

    // Making sure it is good for interpolation
    if (planeEq(3) < 0 )
        planeEq = -planeEq;
}




std::vector<Contour> CameraLaserCalibration::readContours(std::string readContoursPath) {

    std::vector<Contour> out;

    std::fstream contourStr;
    contourStr.open(readContoursPath, std::ios::in);

    while(!contourStr.eof()) {
        Contour rc;
        contourStr >> rc.timestamp;

        for (int pIdx = 0; pIdx < 4; pIdx ++) {
            contourStr >> rc.px[pIdx] >> rc.py[pIdx];
        }
        out.push_back(rc);
    }
    contourStr.close();
    return out;
}

void CameraLaserCalibration::saveCameraChessboardSpeed(Optimization &optimizationProblem) {
    std::string path = ros::package::getPath("camera_lidar_calibrator");

    // Let's compute rotation speed for autocorrelation to determine initial time shift
    std::ofstream chessboardSpeedStream(path + "/log/chessboardSpeed.txt");
    double tStart = optimizationProblem.chessboards[0].timestamp;
    double tEnd = optimizationProblem.chessboards.back().timestamp;

    Eigen::Vector4d lastInterPlaneEq = Eigen::Vector4d::Zero();
    bool lastFound = false;
    for (double tPoint = tStart; tPoint <= tEnd; tPoint = tPoint + 0.002) {

        // Find planeEq for tPoint
        int indexPoses = 0;
        for (; indexPoses < optimizationProblem.chessboards.size(); indexPoses++) {
            if (optimizationProblem.chessboards[indexPoses].timestamp > tPoint)
                break;
        }

        // Is it possible to interpolate
        if (indexPoses >= 1 && indexPoses < optimizationProblem.chessboards.size() - 2) {
            const double &timeB = optimizationProblem.chessboards[indexPoses].timestamp;
            const double &timeC = optimizationProblem.chessboards[indexPoses + 1].timestamp;

            double u = (tPoint - timeB) / (timeC - timeB);

            // Finding interpolated plane
            Eigen::Matrix<double, 3, 4> test;
            test.col(0) = optimizationProblem.chessboards[indexPoses - 1].planeEqMinRepr;
            test.col(1) = optimizationProblem.chessboards[indexPoses].planeEqMinRepr;
            test.col(2) = optimizationProblem.chessboards[indexPoses + 1].planeEqMinRepr;
            test.col(3) = optimizationProblem.chessboards[indexPoses + 2].planeEqMinRepr;

            Eigen::Vector3d ignoreJN;
            double ignoreJD;
            Eigen::Vector3d chessboardMinRepr = BSpline::interMinPlaneCum(test, u, ignoreJN, ignoreJD);
            Eigen::Vector4d interPlaneEq = plane::minRepr2planeEq(chessboardMinRepr);

            if (lastFound) {
                double angle = acos(interPlaneEq.head<3>().dot(lastInterPlaneEq.head<3>())) * 180.0 / M_PI ;
                double dangle = angle / 0.002;

                chessboardSpeedStream << std::fixed << std::setprecision(6) << tPoint << " " << dangle << std::endl;
            }

            lastInterPlaneEq = interPlaneEq;
            lastFound = true;
        } else
            lastFound = false;
    }
    chessboardSpeedStream.close();
}

void CameraLaserCalibration::saveLidarChessboardSpeed(Optimization &optimizationProblem) {
    std::string path = ros::package::getPath("camera_lidar_calibrator");

    std::vector<PointStamped,Eigen::aligned_allocator<PointStamped>> lPoints = optimizationProblem.lidarPoints;
    std::cout << "lPoints = " <<lPoints.size() << std::endl;
    std::sort( lPoints.begin( ), lPoints.end( ), [ ]( const PointStamped& lhs, const PointStamped& rhs )
    {
        return lhs.timestamp < rhs.timestamp;
    });

    // Group of 30?
    double lastTimestamp = -10.0;
    Eigen::Vector4d lastPlaneEq = Eigen::Vector4d::Zero();

    std::ofstream chessboardStream(path + "/log/chessboardLidarSpeed.txt");

    const double timeLimit = 0.02;
    for (int i=0;i<lPoints.size();) {
        double timeA = lPoints[i].timestamp;

        int idx2 = 0;
        for (;idx2+i<lPoints.size();idx2++) {
            if (lPoints[i+idx2].timestamp - timeA >= timeLimit)
                break;
        }

        // Plane fitting
        Eigen::MatrixXd points = Eigen::MatrixXd::Zero(3, idx2);

        for (int j = 0; j < idx2; j++) {
            points.col(j) = lPoints[i+j].point;
        }

        // Computing mean
        Eigen::Vector3d mean = Eigen::Vector3d::Zero();
        for (int j = 0; j < points.cols(); j++)
            mean += points.col(j);
        mean /= points.cols();

        // Points without mean
        Eigen::MatrixXd demeanPts = points;
        for (int j = 0; j < demeanPts.cols(); ++j)
            demeanPts.col(j) -= mean;

        // Covariance
        Eigen::Matrix3d covariance = demeanPts * demeanPts.transpose();

//            std::cout << "cov " << covariance << std::endl;

        // EigenValue
        Eigen::SelfAdjointEigenSolver <Eigen::Matrix3d> evd(covariance);

        // Plane equation
        Eigen::Vector4d planeEq = Eigen::Vector4d::Zero();
        planeEq.head<3>() = evd.eigenvectors().col(0);
        planeEq(3) = -planeEq.head<3>().dot(mean.head<3>());

//            std::cout << "? " << planeEq.transpose() << std::endl;

        double timestamp = (lPoints[i].timestamp + lPoints[i+idx2-1].timestamp)/2.0;

        if (timestamp - lastTimestamp < 5*timeLimit) {
            double angle = acos(planeEq.head<3>().dot(lastPlaneEq.head<3>())) * M_PI / 180.0;
            double dt = timestamp - lastTimestamp;
            double dangle = angle / dt;

            chessboardStream << std::fixed << std::setprecision(6) << timestamp << " " << dangle << std::endl;


        }
        lastPlaneEq = planeEq;
        lastTimestamp = timestamp;
        i = i + idx2 + 1;
    }

    chessboardStream.close();
}

void CameraLaserCalibration::verify( std::vector<std::pair<double, pcl::PointCloud<pcl::PointXYZI>::Ptr>> & cloudBuffer,
                    std::vector<std::pair<double, cv::Mat>> & imageBuffer) {
    std::string path = ros::package::getPath("camera_lidar_calibrator");

    // Just to visualize for article
    Eigen::Matrix4d visLidarInCamera = Eigen::Matrix4d::Identity();
    double visTimeOffset = 0.0;
    bool showProjection = true;

    // MRS - LEFT - with time
//    visLidarInCamera << -0.00272021, 0.997201, -0.0747234, 0.146934,
//                        -0.0993313, 0.0740847, 0.992293, 0.082831,
//                        0.995051, 0.0101216, 0.0988517, 0.110501,
//                        0, 0, 0, 1;
//    double visTimeOffset = -0.200988;

    visLidarInCamera << -6.73571e-05, 0.99694, -0.0781678, 0.150733,
                        -0.094452, 0.077812, 0.992484, 0.0693782,
                        0.995529, 0.00744996, 0.0941578, 0.109564,
                        0, 0, 0, 1;
    visTimeOffset = -0.202727;

    /// TODO: Different shift group
//    visLidarInCamera << 0.00170944, 0.996879, -0.0789256, 0.148399,
//                        -0.0948317, 0.0787316, 0.992375, 0.06965,
//                        0.995492, 0.00578824, 0.0946703, 0.109736,
//                        0, 0, 0, 1 ;
//    visTimeOffset =  -0.215952;

//    /// MRS LEFT - new shift group, reverse direction
//    visLidarInCamera << -0.00217429, 0.997145, -0.0754846, 0.152421,
//                        -0.0920154, 0.074965, 0.992932, 0.0635762,
//                        0.995755, 0.00910466, 0.0915897, 0.108853,
//                        0, 0, 0, 1 ;
//    visTimeOffset = -0.016217; //-0.216217

    // MRS - RIGHT
//    visLidarInCamera << -0.0148395, 0.999092, -0.039925, -0.184701,
//                        -0.140019, 0.0374596, 0.98944, 0.0563761,
//                        0.990038, 0.020273,  0.139336, 0.11055,
//                        0, 0, 0, 1;
//    visTimeOffset = -0.217112;


    // MRS - RIGHT - with time
//    visLidarInCamera << -0.0179561, 0.999121, -0.0378711, -0.184959,
//               -0.144893, 0.0348771, 0.988832, 0.0695616,
//                0.989284, 0.0232428, 0.14414, 0.111721,
//                0, 0, 0, 1;
//    visTimeOffset = -0.202409;

////    // MRS - LEFT - no time // TODO
//    visLidarInCamera << -0.00967675, 0.996099, -0.0877111, 0.167363,
//                        -0.10352, 0.086246, 0.990881, 0.141901,
//                        0.99458, 0.0186684, 0.102282, 0.109857,
//                        0, 0, 0, 1;
//    visTimeOffset = 0.0;

    // MRS - RIGHT - no time
//    visLidarInCamera << 0.0079124, 0.996702, -0.0807639, -0.238829,
//                        -0.147315, 0.0810471, 0.985763, 0.109252,
//                        0.989058, 0.004098, 0.147471, 0.124274,
//                        0, 0, 0, 1;
//    visTimeOffset = 0.0;

    // Copied image buffer
    std::vector<std::pair<double, cv::Mat>> origImageBuffer;
    for (int i=0;i<imageBuffer.size();i++) {
        cv::Mat img = imageBuffer[i].second.clone();
        std::pair<double, cv::Mat> p = std::make_pair(imageBuffer[i].first, img);
        origImageBuffer.push_back(p);
    }


    // For each cloud
    int lastImgIdx = 0;
    for (int cloudIdx=0;cloudIdx<cloudBuffer.size();cloudIdx++) {

        // Selected timestamp and cloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = cloudBuffer[cloudIdx].second;
        std::cout << "Processing cloud: " << cloudIdx + 1 << " / " << cloudBuffer.size() << ": " << std::endl;

        float startOri = -std::atan2(cloud->points[0].y, cloud->points[0].x);
        float endOri = -std::atan2(cloud->points[cloud->points.size() - 1].y,
                                   cloud->points[cloud->points.size() - 1].x) + 2 * float(M_PI);
        if (endOri - startOri > 3 * M_PI) {
            endOri -= 2 * M_PI;
        } else if (endOri - startOri < M_PI) {
            endOri += 2 * M_PI;
        }

        // Create a range image
        cv::Mat depthImg = cv::Mat(SCANNER_ROWS, SCANNER_COLS, CV_32FC1);
        depthImg = 0;
        cv::Mat timeShiftImg = cv::Mat(SCANNER_ROWS, SCANNER_COLS, CV_32FC1);
        timeShiftImg = 0;

        // For each point
        bool halfPassed = false;
        std::map < std::pair<int, int>, int > rowcol2cloud;

        for (int cloudPointIdx = 0; cloudPointIdx < cloud->points.size(); cloudPointIdx++) {

            // Is it a proper measurement
            if (!pcl_isfinite(cloud->points[cloudPointIdx].x) ||
                !pcl_isfinite(cloud->points[cloudPointIdx].y) ||
                !pcl_isfinite(cloud->points[cloudPointIdx].z)) {
                continue;
            }


            // Is it far enough from the scanner
            float val = sqrt(cloud->points[cloudPointIdx].x * cloud->points[cloudPointIdx].x +
                             cloud->points[cloudPointIdx].y * cloud->points[cloudPointIdx].y +
                             cloud->points[cloudPointIdx].z * cloud->points[cloudPointIdx].z);
            if (val < 0.5 || val > 50) {
                continue;
            }

            int row = 0, col = 0;

            if (sensor_type == "MRS6124") {
                row = cloudPointIdx / SCANNER_COLS;
                col = cloudPointIdx % SCANNER_COLS;
            }
            else if(sensor_type == "VLP16") {
                auto &point = cloud->points[cloudPointIdx];
                float angle = std::atan(point.z / std::sqrt(point.x * point.x + point.y * point.y));
                row = SCANNER_ROWS - int(((angle * 180 / M_PI) - LOWER_BOUND) * VELO_FACTOR + 0.5) - 1;
                float angleHor = std::atan2(point.y, point.x); // from -PI to PI
                if (angleHor < -M_PI)
                    angleHor += 2 * M_PI;
                if (angleHor >= M_PI)
                    angleHor -= 2* M_PI;
                col = (angleHor+M_PI) * SCANNER_COLS * 1.0 / (2*M_PI);
            }


            // Save mapping for the future
            rowcol2cloud[std::make_pair(row,col)] = cloudPointIdx;

            // Filling depth image
            depthImg.at<float>(row, col) = val;

            // MRS
            double relTime = 0.0;
            if (sensor_type == "MRS6124") {
                // MRS 6300 (CCW) rotates in the other direction than Velodyne (CW) :)
                const double startOri = 1.0472; // -60 deg
                const double endOri = -1.0472; // 60 deg

                // Time for one group * (group ID + shift in the group)
                // MRS6300 captures 4 groups from the top, scanIDs (0 to 23) are numbered from the bottom.
                // Taking a scan takes scanPeriod, one group takes scanPeriod / 4
                // shiftGroup -> 0 for top group, 1/4*scanPeriod for 2nd group, 2/4*scanPeriod for 3rd group, 3/4*scan Period for 4th group
                // shiftInGroup -> depends on the horizontal angle, MRS6300 scans CCW, startOri=60deg, endOri=-60deg, relativeAngleChange * 1/4*scanPeriod

                // Assign time shift for each point
                float ori = -std::atan2(cloud->points[cloudPointIdx].y, cloud->points[cloudPointIdx].x);

                ori = -ori;

                int shiftGroup = ceil((23 - row) / 6);
                float shiftInGroup = (startOri - ori) / (2 * M_PI);
                relTime = SCAN_PERIOD / 4.0 * shiftGroup + SCAN_PERIOD * shiftInGroup; // TODO: Testing https://www.sick.com/media/docs/0/40/540/Operating_instructions_MRS6000_en_IM0076540.PDF
                if (relTime > 1)
                    relTime = relTime - 1;
//                std::cout << "relTime = " << relTime << " " << row << " " << SCAN_PERIOD * shiftInGroup <<  std::endl;
            }
            // Velodyne VLP
            else if (sensor_type == "VLP16") {
                // calculate horizontal point angle
                auto &point = cloud->points[cloudPointIdx];
                float ori = -std::atan2(point.y, point.x);
                if (!halfPassed) {
                    if (ori < startOri - M_PI / 2) {
                        ori += 2 * M_PI;
                    } else if (ori > startOri + M_PI * 3 / 2) {
                        ori -= 2 * M_PI;
                    }

                    if (ori - startOri > M_PI) {
                        halfPassed = true;
                    }
                } else {
                    ori += 2 * M_PI;

                    if (ori < endOri - M_PI * 3 / 2) {
                        ori += 2 * M_PI;
                    } else if (ori > endOri + M_PI / 2) {
                        ori -= 2 * M_PI;
                    }
                }

                // calculate relative scan time based on point orientation
                // Velodyne driver set timestamp of the cloud to its end so - SCAN_PERIOD
                relTime = SCAN_PERIOD * (ori - startOri) / (endOri - startOri) - SCAN_PERIOD;
            }

            // Save time shift for a point
            timeShiftImg.at<float>(row, col) = relTime;
        }

        // Going through all rows and cols
        cv::Mat projectTimeFill = cv::Mat(imageBuffer[0].second.rows, imageBuffer[0].second.cols, CV_32FC1);
        projectTimeFill = 1.0;
        for (int i = 0; i < SCANNER_COLS; i++) {
            for (int j = 0; j < SCANNER_ROWS; j++) {

                // Just visualize // TODO: Stupid code
                if ( depthImg.at<float>(j, i) > 0 ) {
                    int index = rowcol2cloud[std::make_pair(j,i)];
                    Eigen::Vector4d p = Eigen::Vector4d(cloud->points[index].x,
                                                        cloud->points[index].y,
                                                        cloud->points[index].z, 1.0);
                    double range = p.head<3>().norm();

                    Eigen::Vector4d pCam = visLidarInCamera * p;
                    std::vector <cv::Point3d> framePoints;
                    std::vector <cv::Point2d> imageFramePoints;
                    framePoints.push_back(cv::Point3d(pCam(0), pCam(1), pCam(2)));
                    cv::Mat rvec = cv::Mat(cv::Size(3, 1), CV_64F), tvec = cv::Mat(cv::Size(3, 1), CV_64F);
                    tvec = 0, rvec = 0;
                    cv::projectPoints(framePoints, rvec, tvec, cameraMatrixInMat, distortionCoefficients,
                                      imageFramePoints);

                    double pGlobalTime = cloudBuffer[cloudIdx].first + timeShiftImg.at<float>(j, i) + visTimeOffset;

                    // Consider only images in the neighbourhood
                    int newLastImgIdx = lastImgIdx;
                    for (int imgIdx = std::max(0, lastImgIdx-10); imgIdx < std::min((int)imageBuffer.size(), lastImgIdx + 10); imgIdx++) {

                        // If the time threshold is satisfied then project this point
                        double timeDiff = fabs(imageBuffer[imgIdx].first - pGlobalTime);
                        if (timeDiff < 0.08) // TODO: Important value 80 ms
                        {
                            cv::Point projectP = (cv::Point) imageFramePoints[0];
                            if (showProjection && projectTimeFill.at<float>(projectP.y, projectP.x) > timeDiff) {

//                            std::cout << "range : " << range << " " << jet_color::range2interval(range) << std::endl;
                                double b = jet_color::blue(range);
                                double g = jet_color::green(range);
                                double r = jet_color::red(range);
//                                std::cout << "color : " << b << " " << g << " " << r << std::endl;
                                cv::Scalar color = cv::Scalar(b*255, g*255, r*255, 0);
//                                if (timeDiff < 0.02)
//                                    color = CV_RGB(255, 0, 0);
//                                else if (timeDiff < 0.04)
//                                    color = CV_RGB(0, 255, 0);
//                                else if (timeDiff < 0.06)
//                                    color = CV_RGB(0, 0, 255);
//                                else
//                                    color = CV_RGB(255, 255, 255);

                                /// TODO: Blending with addWeighted: https://stackoverflow.com/questions/24480751/how-to-create-a-semi-transparent-shape

                                cv::circle(imageBuffer[imgIdx].second, (cv::Point) imageFramePoints[0], 1,
                                           color, 2);
                                projectTimeFill.at<float>(projectP.y, projectP.x) = timeDiff;
                            }
                            newLastImgIdx = imgIdx;
                        }
                    }
                    lastImgIdx = newLastImgIdx;

                }
            }
        }

        if(cloudIdx == 3) {
            cv::Mat out, outScaled;
            cv::normalize(timeShiftImg, out, 0, 255, cv::NORM_MINMAX, CV_8UC1);
            cv::resize(out, outScaled, cv::Size(400, 400));
            cv::applyColorMap(outScaled, out, cv::COLORMAP_JET);
            cv::imwrite(path + "/vis/imgTimeShift.png", out);
        }
    }






    // Show some projections
    for (int i = 0; i < imageBuffer.size(); i++) {

        // Blending
        double alpha = 0.8;
        cv::Mat outImg;
        cv::addWeighted(imageBuffer[i].second, alpha, origImageBuffer[i].second, 1 - alpha, 0, outImg);


//        std::cout << "Save img" << std::endl;
        std::string indexAsString = std::to_string(i);
        cv::imwrite(path + "/vis/img" + std::string(5 - indexAsString.length(), '0') + indexAsString + ".png", outImg);//imageBuffer[i].second);
    }

    exit(0);

}

double CameraLaserCalibration::estimateRelativeTime(pcl::PointXYZI &point, float &startOri, float &endOri,
        int &row, bool &halfPassed) {

    if (sensor_type == "MRS6124") {
        // MRS 6300 (CCW) rotates in the other direction than Velodyne (CW) :)
        const double startOri = 1.0472; // -60 deg
//                const double endOri = -1.0472; // 60 deg

        // Time for one group * (group ID + shift in the group)
        // MRS6300 captures 4 groups from the top, scanIDs (0 to 23) are numbered from the bottom.
        // Taking a scan takes scanPeriod, one group takes scanPeriod / 4
        // shiftGroup -> 0 for top group, 1/4*scanPeriod for 2nd group, 2/4*scanPeriod for 3rd group, 3/4*scan Period for 4th group
        // shiftInGroup -> depends on the horizontal angle, MRS6300 scans CCW, startOri=60deg, endOri=-60deg, relativeAngleChange * 1/4*scanPeriod

        // Assign time shift for each point
        float ori = -std::atan2(point.y, point.x);
        int shiftGroup = ceil((23 - row) / 6);

        float shiftInGroup = (startOri - ori) / (2 * M_PI);

        double relTime = SCAN_PERIOD / 4.0 * shiftGroup + SCAN_PERIOD * shiftInGroup; // TODO: Testing https://www.sick.com/media/docs/0/40/540/Operating_instructions_MRS6000_en_IM0076540.PDF
        if (relTime > 1)
            relTime = relTime - 1;

        return relTime;
    }
        // Velodyne VLP
    else if (sensor_type == "VLP16") {
        // calculate horizontal point angle
        float ori = -std::atan2(point.y, point.x);
        if (!halfPassed) {
            if (ori < startOri - M_PI / 2) {
                ori += 2 * M_PI;
            } else if (ori > startOri + M_PI * 3 / 2) {
                ori -= 2 * M_PI;
            }

            if (ori - startOri > M_PI) {
                halfPassed = true;
            }
        } else {
            ori += 2 * M_PI;

            if (ori < endOri - M_PI * 3 / 2) {
                ori += 2 * M_PI;
            } else if (ori > endOri + M_PI / 2) {
                ori -= 2 * M_PI;
            }
        }

        // calculate relative scan time based on point orientation
        // Velodyne driver set timestamp of the cloud to its end so - SCAN_PERIOD
        return SCAN_PERIOD * (ori - startOri) / (endOri - startOri) - SCAN_PERIOD;

    }
    return 0.0;
}