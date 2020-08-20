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
#ifndef LIEALEGEBRA_H
#define LIEALEGEBRA_H

#include <Eigen/Eigen>

namespace Eigen {
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
}

class LieAlgebra {
public:
    static Eigen::Matrix3d exp(const Eigen::Vector3d &omega) {
        Eigen::AngleAxisd aa(omega.norm(), omega.normalized());
        return aa.toRotationMatrix();
    }

    static Eigen::Vector3d log(const Eigen::Matrix3d &R) {

        Eigen::AngleAxisd aa (R);
        return aa.axis() * aa.angle();

    }

    static Eigen::Matrix4d inv(const Eigen::Matrix4d &mat) {
        Eigen::Matrix4d inv = Eigen::Matrix4d::Identity();
        inv.block<3, 3>(0, 0) = mat.block<3, 3>(0, 0).transpose();
        inv.block<3, 1>(0, 3) = -inv.block<3, 3>(0, 0) * mat.block<3, 1>(0, 3);

        return inv;
    }

    static Eigen::Matrix4d invT(const Eigen::Matrix4d &mat) {
        Eigen::Matrix4d invT = Eigen::Matrix4d::Identity();
        invT.block<3,3>(0,0) = mat.block<3,3>(0,0);
        invT.block<1,3>(3,0) = -mat.block<3,1>(0,3).transpose() * invT.block<3,3>(0,0);

        return invT;
    }

    static Eigen::Matrix3d skew(const Eigen::Vector3d &v) {
        Eigen::Matrix3d ret;
        ret <<	0,		-v(2),	v(1),
                v(2),	0,		-v(0),
                -v(1),	v(0),	0;
        return ret;
    }

    static Eigen::Vector3d deskew(const Eigen::Matrix3d &R) {
        Eigen::Vector3d ret;
        // TODO: Check if it is symmetric
        ret <<	R(2,1), R(0,2), R(1,0);
        return ret;
    }

};

#endif // LIEALEGEBRA_H
