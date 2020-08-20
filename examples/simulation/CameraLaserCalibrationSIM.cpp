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
#include "CameraLaserCalibrationSIM.h"

#include <ros/package.h>
#include <random>
#include <chrono>
#include "utils/BSpline.h"
#include "local_g2o/minimal_plane.h"

CameraLaserCalibrationSIM::CameraLaserCalibrationSIM() : nh("~"), CameraLaserCalibration() {
    TAG = "CameraLaserCalibrationSIM";

    std::string path = ros::package::getPath("camera_lidar_calibrator");

    // Parameters to read from file
    double artificialTimeShift = readParameter<double> (nh, "artifical_time_shift", 0.0);
    double lidar_gauss_sigma = readParameter<double> (nh, "lidar_gauss_sigma", 0.0);
    bool intelligentEdgeSelection = readParameter<bool> (nh, "intelligent_edge_selection", false);
    double maxEdgeNumber = readParameter<int> (nh, "max_edge_number", 10000);
    int random_seed = readParameter<int> (nh, "random_seed", 0);
    bool timeShiftFixed = readParameter<bool> (nh, "time_shift_fixed", false);
    double chessboardFramerate = readParameter<double> (nh, "camera_chessboard_framerate", 10.0);


    // Random seed based on provided value to achieve repeatbility for different settings
    srand((unsigned int) random_seed);
    std::mt19937_64 rng;
    rng.seed(random_seed);

    // initialize a uniform distribution between 0 and 1
    std::uniform_real_distribution<double> unifFromZeroToHalfPi(0, M_PI/2.0);
    std::uniform_real_distribution<double> unifFromZeroToQuarterPi(0, M_PI/4.0);
    std::uniform_real_distribution<double> unifFromZeroToEightPi(0, M_PI/8.0);
    std::normal_distribution<double> gauss(0, lidar_gauss_sigma);

    // Initial real transformation between laser scanner and camera
    // LIDAR (x forward, y left, z up) in Camera (x right, y down, z forward)
    Eigen::Matrix4d TlaserInCam = Eigen::Matrix4d::Identity();
    TlaserInCam.block<3, 1>(0, 0) = Eigen::Vector3d::UnitZ();
    TlaserInCam.block<3, 1>(0, 1) = -Eigen::Vector3d::UnitX();
    TlaserInCam.block<3, 1>(0, 2) = -Eigen::Vector3d::UnitY();

    // Random modification to transformation between laser scanner and camera
    // x from -1 to 1
    // y from -0.5 to 0.5
    // z from -0.25 to 0.25
    // angle from 0 to 45 deg
    Eigen::Matrix4d modification = Eigen::Matrix4d::Identity();
    modification.block<3,1>(0,3) = Eigen::Vector3d::Random();
    modification(0,3) = modification(0,3) * 1; // From -1 to 1
    modification(1,3) = modification(1,3) * 0.5; // From -0.5 to 0.5
    modification(2,3) = modification(2,3) * 0.25; // From -0.25 to 0.25

    Eigen::Vector3d randomAxis = Eigen::Vector3d::Random();
    Eigen::AngleAxisd modifAA(unifFromZeroToQuarterPi(rng),randomAxis.normalized());
    modification.block<3,3>(0,0) = modifAA.toRotationMatrix();

    // Assumed real transformation between laser scanner and camera
    TlaserInCam = TlaserInCam * modification;
    std::cout << "Real TlaserInCam" << std::endl << TlaserInCam << std::endl;

    // Initial guess for laser pose in camera is in ``vicinity`` of the real value
    // x from -0.1 to 0.1
    // y from -0.1 to 0.1
    // z from -0.1 to 0.1
    // angle from 0 to 22.5 deg
    Eigen::Matrix4d initialGuessError = Eigen::Matrix4d::Identity();
    initialGuessError.block<3,1>(0,3) = Eigen::Vector3d::Random();
    initialGuessError(0,3) = initialGuessError(0,3) * 0.1; // From -0.1 to 0.1
    initialGuessError(1,3) = initialGuessError(1,3) * 0.1; // From -0.1 to 0.1
    initialGuessError(2,3) = initialGuessError(2,3) * 0.1; // From -0.1 to 0.1

    Eigen::Vector3d randomAxis2 = Eigen::Vector3d::Random();
    Eigen::AngleAxisd igAA(unifFromZeroToEightPi(rng), randomAxis2.normalized());
    initialGuessError.block<3,3>(0,0) = igAA.toRotationMatrix();

    // Initial guess for optimization
    Eigen::Matrix4d estimatedLaserInCamera = TlaserInCam * initialGuessError;
//    Eigen::Matrix4d estimatedLaserInCamera = TlaserInCam; // TODO Uncomment for perfect initial guess

    // Size of the chessboard pattern, 9 x 6, 10 cm each square
    double xInc = 9 * 0.1, yInc = 6 * 0.1;

    // At start, the chessboard is placed directly in front of camera image, 3 meters in front
    Eigen::Matrix4d chessboardOrigin;
    chessboardOrigin.block<4,1>(0,0) << -xInc / 2, - yInc /2, 3, 1; // left top
    chessboardOrigin.block<4,1>(0,1) << xInc / 2, -yInc /2, 3, 1; // right top
    chessboardOrigin.block<4,1>(0,2) << xInc / 2,  yInc /2, 3, 1; // right bottom
    chessboardOrigin.block<4,1>(0,3) << -xInc / 2, yInc /2, 3, 1; // left bottom
    Eigen::Vector4d originPlaneEq;
    originPlaneEq << 0, 0, -1, 3;

    // Generate main detections every 5 second
    unsigned int nPoses = 13;
    const double mainDetectionFramerate = 1/5.0;
    std::vector<ChessboardDetection> chessboards;
    std::vector<PoseStamped> poses;
    for (int i=0;i<nPoses;i++) {

        // Let's generate random change to initial pose of chessboard pattern (0, 0, 3)
        // x from -4 to 4
        // y from -1 to 1
        // z from -2 to 2
        // rotation by no more than 90 deg

        Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
        pose.block<3,1>(0,3) = Eigen::Vector3d::Random();
        pose(0,3) = pose(0,3) * 4;
        pose(1,3) = pose(1,3);
        pose(2,3) = pose(2,3) * 2;

        Eigen::Vector3d randomAxis = Eigen::Vector3d::Random();
        Eigen::AngleAxisd aa(unifFromZeroToHalfPi(rng), randomAxis.normalized());
        pose.block<3,3>(0,0) = aa.toRotationMatrix();

        PoseStamped ps;
        ps.timestamp = 5*(i-1);
        ps.pose = pose;
        ps.r = aa.axis() * aa.angle();
        ps.t = pose.block<3,1>(0,3);
        poses.push_back(ps);

        // Inverse transpose to move planeEq (R 0 \\ -t^T R 1)
        Eigen::Matrix4d inverseTranspose = LieAlgebra::invT(pose);

        // New plane equation
        Eigen::Vector4d estPlaneEq = inverseTranspose * originPlaneEq;

        ChessboardDetection cd;
        cd.timestamp = 5*(i-1);
        cd.timestampGlobal = cd.timestamp;
        cd.planeEq = estPlaneEq;
        cd.planeEqMinRepr = plane::planeEq2minRepr(estPlaneEq);
        chessboards.push_back(cd);
    }

    saveChessboardEqs(path, "chessboardEqsInit.txt", chessboards);

    // Densify with B-Spline interpolation between key planes (we have to reject 1st and last due to inability to interpolate with bSpline)
//    const double wantedChessboardFramerate = 10;
    const double wantedChessboardFramerate = chessboardFramerate;
    std::vector<ChessboardDetection> denseChessboards;//, denseChessboards2;
    std::vector<PoseStamped> densePoses;
    for (int index=1;index<poses.size()-2;index++)
    {
        // Computing increment that will result in wanted framerate
        double increment = mainDetectionFramerate / wantedChessboardFramerate;
        double timeBetweenConsecutives = poses[index + 1].timestamp - poses[index].timestamp;

        // Generating missing chessboards with b-Splines
        for (double u = 0; u < 1; u+= increment) {

            // Interpolating SE3 (Really t and SO(3))
            Eigen::Matrix4d interT = BSpline::interTSO3Cum(poses[index - 1].pose, poses[index].pose,
                                                           poses[index + 1].pose, poses[index + 2].pose, u);

            // Getting new point location
            PoseStamped ps;
            ps.timestamp = poses[index].timestamp + timeBetweenConsecutives * u;
            ps.pose = interT;
            Eigen::Matrix3d tmp = interT.block<3,3>(0,0);
            ps.r = LieAlgebra::log(tmp);
            ps.t = interT.block<3,1>(0,3);
            densePoses.push_back(ps);

            // Inverse transpose to move planeEq (R 0 \\ -t^T R 1)
            Eigen::Matrix4d inverseTranspose = LieAlgebra::invT(interT);

            // Interpolated planeEq
            Eigen::Vector4d interPlaneEq = inverseTranspose * originPlaneEq;

            ChessboardDetection cd;
            cd.timestamp = poses[index].timestamp + timeBetweenConsecutives * u;
            cd.timestampGlobal = cd.timestamp;
            cd.planeEq = interPlaneEq;
            cd.planeEqMinRepr = plane::planeEq2minRepr(interPlaneEq);
            denseChessboards.push_back(cd);
        }
    }
//    std::cout << "Number of generated chessboards: " << denseChessboards.size() << std::endl;

    // Saving
    saveChessboards(path, densePoses, chessboardOrigin);
    saveChessboardEqs(path, "chessboardEqs.txt",denseChessboards);

    // Generate lidarPoints with pseudo VLP-16 scanner
    setSensorType("VLP16");
    double tStart = densePoses[0].timestamp;
    double tEnd = densePoses.back().timestamp;

    int scanCount = (tEnd - tStart) / SCAN_PERIOD;
    std::cout << "Generating " << scanCount << " scans" << std::endl;
    std::vector<PointStamped,Eigen::aligned_allocator<PointStamped>> lidarPoints;


    for (int index = 0; index < scanCount; index++) {
        double tCloudStart = tStart + SCAN_PERIOD * index;

        // Marking used points
        cv::Mat consideredPoints = cv::Mat(SCANNER_ROWS, SCANNER_COLS, CV_8UC1);
        consideredPoints = 0;

        // Generating cloud points
        double horAngleIncrement = 360.0 / (SCANNER_COLS - 1);
        double vertAngleIncrement = (UPPER_BOUND - LOWER_BOUND) / (SCANNER_ROWS - 1);

        for (int idxCol = 0; idxCol < SCANNER_COLS; idxCol ++ ) {
            double tPoint = tCloudStart + double(idxCol) / SCANNER_COLS  * SCAN_PERIOD;

            // Find planeEq for tPoint
            int indexPoses = 0;
            for (;indexPoses<densePoses.size();indexPoses++){
                if (densePoses[indexPoses].timestamp > tPoint)
                    break;
            }

            // Is it possible to interpolate using B-spline
            if (indexPoses >= 1 && indexPoses < densePoses.size() - 2) {
                const double &timeB = densePoses[indexPoses].timestamp;
                const double &timeC = densePoses[indexPoses + 1].timestamp;

                double u = (tPoint - timeB) / (timeC - timeB);

                // TODO This is previous code with further SE3 interpolation
                // Interpolating transformation
                Eigen::Matrix4d interT = BSpline::interTSO3Cum(densePoses[indexPoses - 1].pose,
                                                               densePoses[indexPoses].pose,
                                                               densePoses[indexPoses + 1].pose,
                                                               densePoses[indexPoses + 2].pose, u);

                // Inverse transpose to move planeEq (R 0 \\ -t^T R 1)
                Eigen::Matrix4d inverseTranspose = LieAlgebra::invT(interT);

                // Interpolated planeEq
                Eigen::Vector4d interPlaneEq = inverseTranspose * originPlaneEq;

                Eigen::Matrix4d transformedCornersOut = interT * chessboardOrigin;

                // TODO: This is code with plane interpolation
//                Eigen::Matrix<double, 3, 4> test;
//                test.col(0) = denseChessboards[indexPoses-1].planeEqMinRepr;
//                test.col(1) = denseChessboards[indexPoses].planeEqMinRepr;
//                test.col(2) = denseChessboards[indexPoses+1].planeEqMinRepr;
//                test.col(3) = denseChessboards[indexPoses+2].planeEqMinRepr;
//
//                Eigen::Vector3d jacobianN;
//                double jacobianD;
//                Eigen::Vector3d chessboardMinRepr = BSpline::interMinPlaneCum(test, u, jacobianN, jacobianD);
//                Eigen::Vector4d interPlaneEq = plane::minRepr2planeEq(chessboardMinRepr);
//
//                // B-Spline for corner interpolation
//                Eigen::Matrix4d transformedCornersA = densePoses[indexPoses-1].pose * chessboardOrigin;
//                Eigen::Matrix4d transformedCornersB = densePoses[indexPoses].pose * chessboardOrigin;
//                Eigen::Matrix4d transformedCornersC = densePoses[indexPoses+1].pose * chessboardOrigin;
//                Eigen::Matrix4d transformedCornersD = densePoses[indexPoses+2].pose * chessboardOrigin;
//
//                Eigen::Matrix4d transformedCornersOut = Eigen::Matrix4d::Zero();
//                for (int row = 0; row < 3; row++) {
//                    for (int col = 0; col < 4; col++) {
//                        Eigen::Vector4d cps;
//                        cps << transformedCornersA(row,col), transformedCornersB(row,col), transformedCornersC(row,col), transformedCornersD(row,col);
//                        transformedCornersOut(row,col) = BSpline::interValueAbs(cps, u);
//                    }
//                }

                Eigen::Vector3d leftTopCorner = transformedCornersOut.block<3, 1>(0, 0);
                Eigen::Vector3d rightTopCorner = transformedCornersOut.block<3,1>(0,1);
                Eigen::Vector3d rightBottomCorner = transformedCornersOut.block<3,1>(0,2);
                Eigen::Vector3d leftBottomCorner = transformedCornersOut.block<3,1>(0,3);

                // Vectors used to check if point is inside the polygon
                Eigen::Vector3d polyRtLt = rightTopCorner - leftTopCorner;
                Eigen::Vector3d polyRbRt = rightBottomCorner - rightTopCorner;
                Eigen::Vector3d polyLbRb = leftBottomCorner - rightBottomCorner;
                Eigen::Vector3d polyLtLb = leftTopCorner - leftBottomCorner;

                for (int idxRow = 0; idxRow < SCANNER_ROWS; idxRow++) {

                    // Looking for intersection between point and plane
                    double horAngleInRad = idxCol * horAngleIncrement * M_PI / 180.0;
                    double vertAngleInRad = (LOWER_BOUND + idxRow * vertAngleIncrement) * M_PI / 180.0;

                    // Direction based on angles
                    Eigen::Vector3d vec;
                    vec(0) = cos(horAngleInRad) * cos(vertAngleInRad);
                    vec(1) = sin(horAngleInRad) * cos(vertAngleInRad);
                    vec(2) = sin(vertAngleInRad);

                    // Finding the 'r' that intersects with the created planeEq
                    double nom = (-interPlaneEq(3) - interPlaneEq.head<3>().transpose() * TlaserInCam.block<3,1>(0,3));
                    double denom = interPlaneEq.head<3>().transpose() * TlaserInCam.block<3,3>(0,0) * vec;
                    double r = nom / denom;

                    // Point lying on a plane in front of scanner
                    if ( r > 0 ) {

                        // Check if it inside chessboard polygon
                        Eigen::Vector3d point = r * vec;
                        Eigen::Vector3d pointInCamera = TlaserInCam.block<3,3>(0,0) * point + TlaserInCam.block<3,1>(0,3);

                        Eigen::Vector3d pRt = rightTopCorner - pointInCamera;
                        Eigen::Vector3d pLt = leftTopCorner - pointInCamera;
                        Eigen::Vector3d pLb = leftBottomCorner - pointInCamera;
                        Eigen::Vector3d pRb = rightBottomCorner - pointInCamera;

                        // Checking 4 angles between point to vertex and each side. The same signs mean that point is inside
                        int check = 0;

                        if (pRt.dot(polyRtLt) < 0)
                            check--;
                        else
                            check++;

                        if (pLt.dot(polyLtLb) < 0)
                            check--;
                        else
                            check++;

                        if (pLb.dot(polyLbRb) < 0)
                            check--;
                        else
                            check++;

                        if (pRb.dot(polyRbRt) < 0)
                            check--;
                        else
                            check++;

                        // Point is inside the corners of the plane
                        if (check == 4 || check == -4) {

                            // Creating a laser point (adding artificial time and random range if needed)
                            PointStamped ps;
                            ps.timestamp = tPoint + artificialTimeShift;
                            ps.point = (r + gauss(rng)) * vec;
                            lidarPoints.push_back(ps);

                            consideredPoints.at<unsigned char>(idxRow, idxCol) = 255;
                        }
                    }
                }
            }
        }

        // Saving scan with points used for debugging
        cv::Mat consideredPointsScaled;
        cv::resize(consideredPoints, consideredPointsScaled, cv::Size(400,400));
        std::string indexAsString = std::to_string(index);
        cv::imwrite(path + "/sim/" +  std::string(5 - indexAsString.length(), '0') + indexAsString + ".png", consideredPointsScaled);
    }

    std::cout << "Total points: " << lidarPoints.size() << std::endl;

    // Saving points
    std::ofstream pointsStream;
    pointsStream.open(path + "/sim/laserPoints.txt");
    for (int i=0;i<lidarPoints.size();i++) {
        Eigen::Vector3d pointInCamera = TlaserInCam.block<3,3>(0,0) * lidarPoints[i].point + TlaserInCam.block<3,1>(0,3);

        pointsStream << std::fixed << std::setprecision(6) << lidarPoints[i].timestamp << " ";
        pointsStream << pointInCamera.transpose() << std::endl;
    }
    pointsStream.close();

    /// Test optimization
    Optimization optimizationProblem;

    // Filling prepared optimization data
    optimizationProblem.chessboards.swap(denseChessboards);
    optimizationProblem.lidarPoints.swap(lidarPoints);

    // Speed for initial time
    saveLidarChessboardSpeed(optimizationProblem);
    saveCameraChessboardSpeed(optimizationProblem);

    std::cout << "optimizationProblem.chessboards.size() = " << optimizationProblem.chessboards.size() << std::endl;
    std::cout << "optimizationProblem.lidarPoints.size() = " << optimizationProblem.lidarPoints.size() << std::endl;

    // Calibration!
    double estimatedTimeShift;
    double rmse;
    int good, total;
    std::vector<ConvexityResult> convex;

    // Optimization!
    optimizationProblem.optimizeCameraLaser(estimatedTimeShift, estimatedLaserInCamera, rmse, good, total,
                                            convex, timeShiftFixed);

    // Computing error metrics
    Eigen::Matrix4d error = estimatedLaserInCamera.inverse() * TlaserInCam;
    double errT = error.block<3,1>(0,3).norm();
    Eigen::Matrix3d errR = error.block<3,3>(0,0);
    Eigen::Vector3d errLogR = LieAlgebra::log(errR);
    double errTime = artificialTimeShift + estimatedTimeShift;

    std::cout << " --- " << std::endl;
    std::cout << "Metric error: " << errT*100 << " cm" << std::endl;
    std::cout << "Orient error: " << errLogR.norm()*180/M_PI << " deg" << std::endl;
    std::cout << "Time error: " << errTime * 1000 << " ms" << std::endl;
    std::cout << " --- " << std::endl;

    std::ofstream errorStr;
    errorStr.open(path + "/error.txt");
    if (errorStr.is_open())
    {
        errorStr << errT*100 << " " << errLogR.norm()*180/M_PI << " " << errTime * 1000 << std::endl;
        errorStr << estimatedLaserInCamera << std::endl;
        errorStr << TlaserInCam << std::endl;
        errorStr << estimatedTimeShift << std::endl;
        errorStr << artificialTimeShift << std::endl;
        errorStr.close();
    }


    // Checking metric/rot speed vs our measure vs time error
    double tspeed = 0, rspeed = 0;
    for (int i=1;i<densePoses.size();i++) {
        Eigen::Vector3d t = densePoses[i].t - densePoses[i-1].t;
        tspeed +=t.norm();
        Eigen::Vector3d r = LieAlgebra::log(densePoses[i].pose.block<3,3>(0,0).transpose() * densePoses[i-1].pose.block<3,3>(0,0));
        rspeed += (r.norm() * 180.0 / M_PI);
    }
    tspeed = tspeed / (densePoses.back().timestamp - densePoses[0].timestamp);
    rspeed = rspeed / (densePoses.back().timestamp - densePoses[0].timestamp);
    std::cout << "Average translational speed: " << tspeed << " m/s" << std::endl;
    std::cout << "Average rotational speed: " << rspeed << " deg/s" << std::endl;

    std::ofstream timeStr;
    timeStr.open(path + "/checkTime.txt");
    if (timeStr.is_open())
    {
        timeStr << "Tspeed[m/s] Rspeed[deg/s] ErrTime[ms] MeasuredContribTime" << std::endl;
//        timeStr << tspeed << " " << rspeed << " " << errTime * 1000 << " " << optimizationProblem.sumContrib(6) <<std::endl;
        timeStr << tspeed << " " << rspeed << " " << errTime * 1000 << " " << optimizationProblem.lidarPoints.size() <<std::endl;

        timeStr.close();
    }
}

void CameraLaserCalibrationSIM::saveChessboards(std::string path, std::vector<PoseStamped> & densePoses, Eigen::Matrix4d & chessboardOrigin) {
    std::ofstream chessboardStream;
    chessboardStream.open(path + "/sim/chessboards.txt");
    for (int i=0;i<densePoses.size();i++) {

        Eigen::Matrix4d transformedCorners = densePoses[i].pose * chessboardOrigin;

        Eigen::Vector3d leftTopCorner = transformedCorners.block<3,1>(0,0);
        Eigen::Vector3d rightTopCorner = transformedCorners.block<3,1>(0,1);
        Eigen::Vector3d rightBottomCorner = transformedCorners.block<3,1>(0,2);
        Eigen::Vector3d leftBottomCorner = transformedCorners.block<3,1>(0,3);

        chessboardStream << std::fixed << std::setprecision(6) << densePoses[i].timestamp << " ";
        chessboardStream << leftTopCorner.transpose() << " ";
        chessboardStream << rightTopCorner.transpose() << " ";
        chessboardStream << rightBottomCorner.transpose() << " ";
        chessboardStream << leftBottomCorner.transpose() << std::endl;
    }
    chessboardStream.close();
}

void CameraLaserCalibrationSIM::saveChessboardEqs(std::string path, std::string filename, std::vector<ChessboardDetection> & denseChessboards){
    std::ofstream chessboardEqStream;
    chessboardEqStream.open(path + "/sim/" + filename);
    for (int i=0;i<denseChessboards.size();i++) {

        chessboardEqStream << std::fixed << std::setprecision(6) << denseChessboards[i].timestampGlobal << " ";
        chessboardEqStream << denseChessboards[i].planeEq.transpose() << " ";
        chessboardEqStream << denseChessboards[i].planeEqMinRepr.transpose() << std::endl;
    }
    chessboardEqStream.close();
}