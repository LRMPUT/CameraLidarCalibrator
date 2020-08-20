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
#ifndef CAMERA_LIDAR_CALIBRATOR_MINIMAL_PLANE_H
#define CAMERA_LIDAR_CALIBRATOR_MINIMAL_PLANE_H
#include <Eigen/Core>
#include "LieAlgebra.h"

class plane {
public:
    static Eigen::Vector3d planeEq2minRepr(Eigen::Vector4d planeEq) ;
    /*
     * Computes the usual representation (n,d) of the plane provided in minimal representation (3 elements)
     */
    static Eigen::Vector4d minRepr2planeEq(Eigen::Vector3d minimalPlane) ;

    /*
     * Computes the minimal representation (3 elements) of the plane provided as (n,d)
    */
    static Eigen::Vector3d spherical_planeEq2minRepr(Eigen::Vector4d planeEq);

    /*
     * Computes the usual representation (n,d) of the plane provided in minimal representation (3 elements)
     */
    static Eigen::Vector4d spherical_minRepr2planeEq(Eigen::Vector3d minimalPlane);
};

#endif //CAMERA_LIDAR_CALIBRATOR_MINIMAL_PLANE_H
