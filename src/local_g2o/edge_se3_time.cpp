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
#include "local_g2o/edge_se3_time.h"
#include "local_g2o/minimal_plane.h"

namespace g2o {

    EdgeSE3Time::EdgeSE3Time() : BaseBinaryEdge<1, number_t, VertexSE3Expmap, VertexOne>() {

    }

    bool EdgeSE3Time::read(std::istream &is) {
        return true;
    }

    bool EdgeSE3Time::write(std::ostream &os) const {
        return os.good();
    }

    void EdgeSE3Time::computeError() {

        // Pose estimate
        const VertexSE3Expmap *v1 = static_cast<const VertexSE3Expmap *>(_vertices[0]);

        // Pose moved from lidar CS to camera CS
        Vector3 pointInCamera = v1->estimate().map(pointInLidar);

        // Time estimate
        const VertexOne *vTime = static_cast<const VertexOne *>(_vertices[1]);

        // New time guess
        double modifiedPointTimestamp = pointTimestamp + vTime->estimate()[0];

        // Finding the chessboard at that time
        Eigen::Vector3d chessboardMinRepr, jacobianN;
        double jacobianD;
        bool found = calibrationData->findApproximatedChessboard(modifiedPointTimestamp, chessboardMinRepr, jacobianN,
                jacobianD);

        if (found) {
            // Plane as (n,d)
            Eigen::Vector4d planeEq = plane::minRepr2planeEq(chessboardMinRepr);

            // Error distance to plane
            _error[0] = planeEq.head<3>().dot(pointInCamera) + planeEq(3);
        }
        // Outlier
        else {
            _error[0] = 0; // I think it makes more sense as constraint without proper time would just be ignored
        }
    }

    void EdgeSE3Time::linearizeOplus() {

        // Pose estimate
        const VertexSE3Expmap *v1 = static_cast<const VertexSE3Expmap *>(_vertices[0]);

        // Pose
        SE3Quat T = v1->estimate();

        // Pose moved from lidar CS to camera CS
        Vector3 pointInCamera = v1->estimate().map(pointInLidar);

        // Time estimate
        const VertexOne *vTime = static_cast<const VertexOne *>(_vertices[1]);

        // New time guess
        double modifiedPointTimestamp = pointTimestamp + vTime->estimate()[0];

        // Finding the chessboard at that time
        Eigen::Vector3d chessboardMinRepr, jacobianN;
        double jacobianD;
        bool found = calibrationData->findApproximatedChessboard(modifiedPointTimestamp, chessboardMinRepr, jacobianN,
                jacobianD);

        if (found) {
            // Plane as (n,d)
            Eigen::Vector4d planeEq = plane::minRepr2planeEq(chessboardMinRepr);

            /// w.r.t pose
            Eigen::Matrix<double, 3, 6> mat;
            mat.block<3,3>(0,0) = -LieAlgebra::skew(pointInLidar);
            mat.block<3,3>(0,3) = Eigen::Matrix<double, 3, 3>::Identity();
            _jacobianOplusXi = planeEq.head<3>().transpose() * T.rotation().toRotationMatrix() * mat;

            /// w.r.t time
            Eigen::Matrix<double,1,1> jTime;
            jTime(0) = pointInCamera.transpose() * jacobianN + jacobianD;
            _jacobianOplusXj = jTime;
        }
        // Outlier
        else {
            _jacobianOplusXi = Eigen::Matrix<double,6,1>::Zero();
            _jacobianOplusXj = Eigen::Matrix<double,1,1>::Zero();
        }
    }
}