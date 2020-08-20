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
#include "local_g2o/minimal_plane.h"
#include <iostream>

    Eigen::Vector3d plane::planeEq2minRepr(Eigen::Vector4d planeEq) {

        // Making sure it is good for interpolation
        if (planeEq(3) < 0)
            planeEq = -planeEq;


        double theta = acos(planeEq(2));

        // There is an issue at theta = pi so let's just avoid it ;)
        if (fabs(theta - M_PI) < 1e-9) {
            planeEq = -planeEq;
            theta = acos(planeEq(2));
        }

        double omega_x = 0, omega_y = 0;
        if (theta < 0.00001) {
            double theta2 = theta * theta;
            double theta4 = theta2 * theta2;
            omega_x = -planeEq(1) * (1 + theta2 / 6.0 + 7.0 / 360.0 * theta4);
            omega_y = planeEq(0) * (1 + theta2 / 6.0 + 7.0 / 360.0 * theta4);
        } else {
            omega_x = -planeEq(1) * theta / sin(theta);
            omega_y = planeEq(0) * theta / sin(theta);
        }

        Eigen::Vector3d minimalPlane;
        minimalPlane(0) = omega_x;
        minimalPlane(1) = omega_y;
        minimalPlane(2) = planeEq(3);

        return minimalPlane;
    }

    /*
     * Computes the usual representation (n,d) of the plane provided in minimal representation (3 elements)
     */
    Eigen::Vector4d plane::minRepr2planeEq(Eigen::Vector3d minimalPlane) {

        Eigen::Vector3d omega = minimalPlane;
        omega(2) = 0;

        Eigen::Vector3d axisZ = Eigen::Vector3d::Zero();
        axisZ(2) = 1.0;

        Eigen::Vector4d planeEq;
        planeEq.head<3>() = LieAlgebra::exp(omega) * axisZ;
        planeEq(3) = minimalPlane(2);

        // Making sure it is good for interpolation
        if (planeEq(3) < 0)
            planeEq = -planeEq;

        return planeEq;
    }


    /*
     * Computes the minimal representation (3 elements) of the plane provided as (n,d)
    */
    Eigen::Vector3d plane::spherical_planeEq2minRepr(Eigen::Vector4d planeEq) {

        // Making sure it is good for interpolation
        if (planeEq(3) < 0 )
            planeEq = -planeEq;


        double theta = acos(planeEq(2));

        // There is an issue at theta = pi so let's just avoid it ;)
        if (fabs(theta - M_PI) < 1e-9) {
            planeEq = -planeEq;
            theta = acos(planeEq(2));
        }

        double omega_x = 0, omega_y = 0;
        if (theta<0.00001) {
            double theta2 = theta * theta;
            double theta4 = theta2 * theta2;
            omega_x = planeEq(0) * (1 + theta2 / 6.0 + 7.0/360.0 * theta4);
            omega_y = planeEq(1) * (1 + theta2 / 6.0 + 7.0/360.0 * theta4);
        }
        else {
            omega_x = planeEq(0) * theta / sin(theta);
            omega_y = planeEq(1) * theta / sin(theta);
        }

        Eigen::Vector3d minimalPlane;
        minimalPlane(0) = omega_x;
        minimalPlane(1) = omega_y;
        minimalPlane(2) = planeEq(3);

        return minimalPlane;
    }

    /*
     * Computes the usual representation (n,d) of the plane provided in minimal representation (3 elements)
     */
    Eigen::Vector4d plane::spherical_minRepr2planeEq(Eigen::Vector3d minimalPlane) {

        Eigen::Vector3d omega = minimalPlane;
        omega(2) = 0;

        double theta = omega.norm();

        if (theta<0.00001) {
            std::cout << "Taylor needed :/" << std::endl;
        }

        Eigen::Vector4d planeEq;
        planeEq(0) = omega(0) * sin(theta) / theta;
        planeEq(1) = omega(1) * sin(theta) / theta;
        planeEq(2) = cos(theta);
        planeEq(3) = minimalPlane(2);

        // Making sure it is good for interpolation
        if (planeEq(3) < 0 )
            planeEq = -planeEq;

        return planeEq;
    }