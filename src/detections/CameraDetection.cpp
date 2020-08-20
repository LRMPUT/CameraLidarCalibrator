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
#include "detections/CameraDetection.h"
#include "local_g2o/minimal_plane.h"

bool CameraDetection::detectChessboards(double timestamp, cv::Mat img, cv::Mat camera_matrix, cv::Mat distortion,
                                        ChessboardDetection &chessboardDetection) {


    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    cv::Size cbSize = cv::Size(boardHeight, boardWidth);

    std::vector<cv::Point2f> imagePoints;

    cv::Mat grayDst;
    cv::equalizeHist(gray, grayDst);

    bool found = findChessboardCorners(grayDst, cbSize, imagePoints);

    if (found) {
        printf("%.3f - FOUND!\n", timestamp);

        // Subpix corrections
        cv::TermCriteria termcrit(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, 0.001);
        cv::cornerSubPix(gray, imagePoints, cv::Size(5, 5), cv::Size(-1, -1), termcrit);

        // Drawing corners
        for (int i = 0; i < imagePoints.size(); i++)
            cv::circle(img, (cv::Point) imagePoints[i], 4, CV_RGB(0, 255, 0), cv::FILLED);

        // 3D Corners
        std::vector<cv::Point3d> boardPoints;
        for (int i = 0; i < boardWidth; i++) {
            for (int j = 0; j < boardHeight; j++) {
                boardPoints.push_back(
                        cv::Point3d(double(i) * CHESSBOARD_CORNER_SIZE, double(j) * CHESSBOARD_CORNER_SIZE, 0.0));
            }
        }

        // Let's determine pose w.r.t. chessboard
        cv::Mat rvec = cv::Mat(cv::Size(3, 1), CV_64F);
        cv::Mat tvec = cv::Mat(cv::Size(3, 1), CV_64F);
        cv::solvePnP(cv::Mat(boardPoints), cv::Mat(imagePoints), camera_matrix, distortion, rvec, tvec);

        // Projecting the axes of coordinate system
        std::vector<cv::Point3d> framePoints;
        std::vector<cv::Point2d> imageFramePoints;
        framePoints.push_back(cv::Point3d(0.0, 0.0, 0.0));
        framePoints.push_back(cv::Point3d(0.2, 0.0, 0.0));
        framePoints.push_back(cv::Point3d(0.0, 0.2, 0.0));
        framePoints.push_back(cv::Point3d(0.0, 0.0, 0.2));
        projectPoints(framePoints, rvec, tvec, camera_matrix, distortion, imageFramePoints);

        cv::line(img, imageFramePoints[0], imageFramePoints[1], CV_RGB(255, 0, 0), 8);
        cv::line(img, imageFramePoints[0], imageFramePoints[2], CV_RGB(0, 255, 0), 8);
        cv::line(img, imageFramePoints[0], imageFramePoints[3], CV_RGB(0, 0, 255), 8);

        // Project the 3D points onto the image
        std::vector<cv::Point2d> imageBoardPoints;
        projectPoints(boardPoints, rvec, tvec, camera_matrix, distortion, imageBoardPoints);
//        for (int i = 0; i < imageBoardPoints.size(); i++)
//            cv::circle(img, (cv::Point) imageBoardPoints[i], 4, CV_RGB(255, 0, 0));


        if (showDetections) {
            cv::namedWindow("Chessboard check", 0);
            cv::imshow("Chessboard check", img);
            cv::waitKey(0);
        }

        // Converting from lie algebra to 4x4 matrix
        double angle = cv::norm(rvec);
        Eigen::Vector3d axis(rvec.at<double>(0, 0), rvec.at<double>(1, 0), rvec.at<double>(2, 0));
        Eigen::AngleAxisd aa(angle, axis / angle);

        Eigen::Matrix4d chessboardInCamera = Eigen::Matrix4d::Identity();
        chessboardInCamera.block<3, 3>(0, 0) = aa.toRotationMatrix();
        chessboardInCamera.block<3, 1>(0, 3) = Eigen::Vector3d(tvec.at<double>(0, 0), tvec.at<double>(1, 0),
                                                               tvec.at<double>(2, 0));

        // Fitting a plane to point in camera coordinate system
        Eigen::MatrixXd points = Eigen::MatrixXd::Zero(4, boardPoints.size());

        // We moved points to camera CS but one can just move the planeEq to cameraCS
        for (int i = 0; i < boardPoints.size(); i++) {
            Eigen::Vector4d boardP = Eigen::Vector4d(boardPoints[i].x, boardPoints[i].y, boardPoints[i].z, 1);
            points.col(i) = chessboardInCamera * boardP;
        }
        Eigen::Vector4d planeEq = Eigen::Vector4d::Zero();

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
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> evd(covariance);

        // Plane equation
        planeEq.head<3>() = evd.eigenvectors().col(0);
        planeEq(3) = -planeEq.head<3>().dot(mean.head<3>());

        // Making sure it is good for interpolation
        if (planeEq(3) < 0)
            planeEq = -planeEq;

        // Creating planeEq constraint for optimization
        chessboardDetection.timestamp = timestamp;
        chessboardDetection.planeEq = planeEq;
        chessboardDetection.planeEqMinRepr = plane::planeEq2minRepr(planeEq);

        // Adding plane borders
        Eigen::Vector4d topLeft = Eigen::Vector4d(-0.5 * CHESSBOARD_CORNER_SIZE, -0.5 * CHESSBOARD_CORNER_SIZE, 0, 1.0);
        Eigen::Vector4d topRight = Eigen::Vector4d((boardWidth + 0.5) * CHESSBOARD_CORNER_SIZE,
                                                   -0.5 * CHESSBOARD_CORNER_SIZE, 0, 1.0);
        Eigen::Vector4d bottomRight = Eigen::Vector4d((boardWidth + 0.5) * CHESSBOARD_CORNER_SIZE,
                                                      (boardHeight + 0.5) * CHESSBOARD_CORNER_SIZE, 0, 1.0);
        Eigen::Vector4d bottomLeft = Eigen::Vector4d(-0.5 * CHESSBOARD_CORNER_SIZE,
                                                     (boardHeight + 0.5) * CHESSBOARD_CORNER_SIZE, 0, 1.0);

        chessboardDetection.border.resize(4);
        chessboardDetection.border[0] = chessboardInCamera * topLeft;
        chessboardDetection.border[1] = chessboardInCamera * topRight;
        chessboardDetection.border[2] = chessboardInCamera * bottomRight;
        chessboardDetection.border[3] = chessboardInCamera * bottomLeft;

        return true;
    }

    std::cout << timestamp << " - NOT FOUND!" << std::endl;
    return false;
}


void CameraDetection::read(const std::string &readChessboardPath, std::vector<std::pair<double, cv::Mat>> &imageBuffer,
                           std::vector<ChessboardDetection> &chessboardDetections) {

    std::vector<ChessboardDetection> readChessboardDetections;
    std::ifstream chessboardStream(readChessboardPath);
    std::cout << "readChessboardPath = " << readChessboardPath << std::endl;

    while (!chessboardStream.eof()) {
        ChessboardDetection chessboardDetection;
        chessboardStream >> chessboardDetection.timestampGlobal;
        for (int i = 0; i < 3; i++)
            chessboardStream >> chessboardDetection.planeEqMinRepr(i);
        for (int i = 0; i < 4; i++)
            chessboardStream >> chessboardDetection.planeEq(i);
        readChessboardDetections.push_back(chessboardDetection);
    }
    chessboardStream.close();

    std::cout << "readChessboardDetections.size() " << readChessboardDetections.size() << std::endl;

    for (int i = 0; i < imageBuffer.size(); i++) {
        bool found = false;
        for (int cIdx = 0; cIdx < readChessboardDetections.size(); cIdx++) {
            if (fabs(readChessboardDetections[cIdx].timestampGlobal - imageBuffer[i].first) < 0.001) {
                found = true;

                readChessboardDetections[cIdx].timestamp = imageBuffer[i].first - timestampStart + artifical_time_shift;
                chessboardDetections.push_back(readChessboardDetections[cIdx]);
                std::cout << " - FOUND" << std::endl;
                break;
            }
        }
        if (!found) {
            std::cout << " - NOT FOUND" << std::endl;
            continue;
        }
    }

}

void CameraDetection::process(std::vector<std::pair<double, cv::Mat>> &imageBuffer,
                              std::vector<ChessboardDetection> &chessboardDetections) {
    // Processing all images
    for (int i = 0; i < imageBuffer.size(); i++) {

        std::cout << "Processing image: ";
        printf("%04d", i + 1);
        std::cout << " / " << imageBuffer.size() << ": ";


        // Doing detection

        // Detect chessboard and saving it in optimization problem
        ChessboardDetection chessboardDetection;
        chessboardDetection.timestampGlobal = imageBuffer[i].first;

        bool success = detectChessboards(imageBuffer[i].first - timestampStart + artifical_time_shift,
                                         imageBuffer[i].second, cameraMatrixInMat, distortionCoefficients,
                                         chessboardDetection);
        if (success) {
            chessboardDetections.push_back(chessboardDetection);
        }

    }
}

void CameraDetection::saveResult(std::string path, std::vector<ChessboardDetection> &chessboards) {
    // Saving chessboard recognitions
    std::ofstream chessboardStream(path + "/log/chessboards.txt");
    for (int i = 0; i < chessboards.size(); i++) {
        chessboardStream << std::fixed << std::setprecision(6) << chessboards[i].timestampGlobal
                         << " ";
        chessboardStream << chessboards[i].planeEqMinRepr.transpose() << " ";
        chessboardStream << chessboards[i].planeEq.transpose() << std::endl;
    }
    chessboardStream.close();
}

