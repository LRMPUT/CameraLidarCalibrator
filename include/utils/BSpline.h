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
#ifndef CAMERA_LIDAR_CALIBRATOR_BSPLINE_H
#define CAMERA_LIDAR_CALIBRATOR_BSPLINE_H

#include <Eigen/Eigen>

class BSpline {
public:

    /**
     * Cubic bspline for 1-dimensional B-spline interpolation (internally with absolute values)
     * @param values 4 1-dim CP used in the interpolation
     * @param u value from 0 to 1 representing the place between 2nd and 3rd node
     * @return interpolated value
     */
    static double interValueAbs(Eigen::Vector4d values, double u);

    /**
     * Cubic bspline for 1-dimensional B-spline interpolation (internally with relative values)
     * @param values 4 times 1-dim CP used in the interpolation
     * @param u value from 0 to 1 representing the place between 2nd and 3rd node
     * @return interpolated value
     */
    static double interValueCum(Eigen::Vector4d values, double u);

    /**
     * Cubic bspline for SO3 B-spline interpolation (internally with relative values)
     * @param values 4 times SO3 stored as columns in Lie Algebra
     * @param u value from 0 to 1 representing the place between 2nd and 3rd node
     * @return interpolated SO3 in Lie Algebra
     */
    static Eigen::Vector3d interSO3Cum(Eigen::Matrix<double, 3, 4> &values, double u);

    /**
     * Cubic bspline for SE3 (Really T3 and SO3) interpolation (internally with relative values)
     * @param Ta first pose
     * @param Tb second pose
     * @param Tc third pose
     * @param Td fourth pose
     * @param u value from 0 to 1 representing the place between 2nd and 3rd node
     * @return interpolated pose
     */
    static Eigen::Matrix4d interTSO3Cum(Eigen::Matrix4d &Ta, Eigen::Matrix4d &Tb, Eigen::Matrix4d &Tc, Eigen::Matrix4d &Td, double u);

    /**
     * Cubic bspline for min plane B-spline interpolation using Lie Algebra (internally with relative values)
     * @param values 4 times planes in minimal representation as columns
     * @param u value from 0 to 1 representing the place between 2nd and 3rd node
     * @param jacobianN Jacobian of Lie Alebgra part of minimal part representation
     * @param jacobianD Jacobian of the distance part of minimal part representation
     * @param debug whether should we print some information
     * @return interpolated plane in minimal representation
     */
    static Eigen::Vector3d interMinPlaneCum(Eigen::Matrix<double, 3, 4> &values, double u, Eigen::Vector3d &jacobianN,
            double &jacobianD, bool debug = false);

    /**
     * Cubic bspline for min plane interpolation using S3 (internally with relative values)
     * @param values  4 times planes in minimal representation as columns
     * @param u value from 0 to 1 representing the place between 2nd and 3rd node
     * @return interpolated plane in minimal representation
     */
    static Eigen::Vector3d interMinPlaneSphere(Eigen::Matrix<double, 3, 4> &values, double u);



};
#endif //CAMERA_LIDAR_CALIBRATOR_BSPLINE_H
