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
#include "utils/BSpline.h"
#include "local_g2o/LieAlgebra.h"
#include <iostream>

double BSpline::interValueAbs(Eigen::Vector4d values, double u) {

    Eigen::Matrix<double, 4, 4> bSplineMatrix;
    bSplineMatrix << 1, -3, 3, -1,  4, 0, -6, 3,   1, 3, 3, -3,    0, 0, 0, 1;
    double bSplineCoeff = 1.0 / 6;

    Eigen::Matrix<double, 4, 1> vecU;
    vecU << 1, u, u*u, u*u*u;

    return values.transpose() * bSplineCoeff * bSplineMatrix * vecU;
}

double BSpline::interValueCum(Eigen::Vector4d values, double u) {

    Eigen::Matrix<double, 4, 4> bSplineMatrix;
    bSplineMatrix <<  6, 0, 0, 0,    5, 3, -3, 1,    1, 3, 3, -2,    0, 0, 0, 1;
    double bSplineCoeff = 1.0 / 6;

    Eigen::Matrix<double, 4, 1> vecU;
    vecU << 1, u, u*u, u*u*u;

    values(3) = values(3) - values(2);
    values(2) = values(2) - values(1);
    values(1) = values(1) - values(0);

    return values.transpose() * bSplineCoeff * bSplineMatrix * vecU;
}

Eigen::Vector3d BSpline::interSO3Cum(Eigen::Matrix<double, 3, 4> & values, double u) {
    // Preparing bSpline (could be done onve if time is an issue)
    Eigen::Matrix<double, 4, 4> bSplineMatrix;
    bSplineMatrix <<  6, 0, 0, 0,    5, 3, -3, 1,    1, 3, 3, -2,    0, 0, 0, 1;
    double bSplineCoeff = 1.0 / 6;

    Eigen::Matrix<double, 4, 1> vecU;
    vecU << 1, u, u*u, u*u*u;

    Eigen::Matrix<double, 4, 1> b = bSplineCoeff * bSplineMatrix * vecU;

    // Computing needed diffs in Lie Algebra
    Eigen::Vector3d val0 = values.col(0);
    Eigen::Vector3d val1 = values.col(1);
    Eigen::Vector3d val2 = values.col(2);
    Eigen::Vector3d val3 = values.col(3);
    Eigen::Vector3d tmp;
    Eigen::Matrix3d tmpMat, sum = Eigen::Matrix3d::Identity();

    // 0
    sum = sum * LieAlgebra::exp(val0); // * b(0) == 1

    // 1
    tmpMat = LieAlgebra::exp(val0).transpose() * LieAlgebra::exp(val1);
    tmp = b(1) * LieAlgebra::log(tmpMat);
    sum = sum * LieAlgebra::exp(tmp);

    // 2
    tmpMat = LieAlgebra::exp(val1).transpose() * LieAlgebra::exp(val2);
    tmp = b(2) * LieAlgebra::log(tmpMat);
    sum = sum * LieAlgebra::exp(tmp);

    // 3
    tmpMat = LieAlgebra::exp(val2).transpose() * LieAlgebra::exp(val3);
    tmp = b(3) * LieAlgebra::log(tmpMat);
    sum = sum * LieAlgebra::exp(tmp);

    // Final estimated rotation
    return LieAlgebra::log(sum);
}

Eigen::Matrix4d BSpline::interTSO3Cum(Eigen::Matrix4d & Ta, Eigen::Matrix4d & Tb, Eigen::Matrix4d & Tc, Eigen::Matrix4d & Td, double u) {

    Eigen::Matrix4d interT = Eigen::Matrix4d::Identity();
    for (int idx = 0; idx < 3; idx ++) {
        Eigen::Vector4d vals;
        vals << Ta(idx, 3), Tb(idx, 3), Tc(idx, 3), Td(idx, 3);
        interT(idx, 3) = BSpline::interValueCum(vals, u);
    }

    Eigen::Matrix<double, 3, 4> test;
    test.col(0) = LieAlgebra::log(Ta.block<3,3>(0,0));
    test.col(1) = LieAlgebra::log(Tb.block<3,3>(0,0));
    test.col(2) = LieAlgebra::log(Tc.block<3,3>(0,0));
    test.col(3) = LieAlgebra::log(Td.block<3,3>(0,0));

    Eigen::Vector3d interR = BSpline::interSO3Cum(test, u);
    interT.block<3,3>(0,0) = LieAlgebra::exp(interR);
    return interT;
}

Eigen::Vector3d BSpline::interMinPlaneCum(Eigen::Matrix<double, 3, 4> & values, double u, Eigen::Vector3d &jacobianN, double &jacobianD, bool debug) {

    // Preparing bSpline (could be done onve if time is an issue)
    Eigen::Matrix<double, 4, 4> bSplineMatrix;
    bSplineMatrix <<  6, 0, 0, 0,    5, 3, -3, 1,    1, 3, 3, -2,    0, 0, 0, 1;
    double bSplineCoeff = 1.0 / 6;

    // Coefficient of bspline: coeff * C * vecU
    Eigen::Matrix<double, 4, 1> vecU;
    vecU << 1, u, u*u, u*u*u;
    Eigen::Matrix<double, 4, 1> b = bSplineCoeff * bSplineMatrix * vecU;

    // Coefficient of bspline derivative: coeff * C * vecU'
    vecU << 0, 1, 2*u, 3*u*u;
    Eigen::Matrix<double, 4, 1> bDeriv = bSplineCoeff * bSplineMatrix * vecU;

    // Min plane representations in LieAlgebra
    Eigen::Vector3d plane0 = Eigen::Vector3d (values(0,0), values(1,0), 0);
    Eigen::Vector3d plane1 = Eigen::Vector3d (values(0,1), values(1,1), 0);
    Eigen::Vector3d plane2 = Eigen::Vector3d (values(0,2), values(1,2), 0);
    Eigen::Vector3d plane3 = Eigen::Vector3d (values(0,3), values(1,3), 0);

    // SO(3) values
    Eigen::Matrix3d VAL0 = LieAlgebra::exp(plane0);
    Eigen::Matrix3d VAL1 = LieAlgebra::exp(plane1);
    Eigen::Matrix3d VAL2 = LieAlgebra::exp(plane2);
    Eigen::Matrix3d VAL3 = LieAlgebra::exp(plane3);

    // A1 - difference time coefficient
    Eigen::Vector3d diff01 = LieAlgebra::log(VAL0.transpose() * VAL1);
    Eigen::Matrix3d A1 = LieAlgebra::exp(b(1) * diff01);

    // A2 - difference time coefficient
    Eigen::Vector3d diff12 = LieAlgebra::log(VAL1.transpose() * VAL2);
    Eigen::Matrix3d A2 = LieAlgebra::exp( b(2) * diff12);

    // A3 - difference time coefficient
    Eigen::Vector3d diff23 = LieAlgebra::log(VAL2.transpose() * VAL3);
    Eigen::Matrix3d A3 = LieAlgebra::exp(b(3) * diff23);

    // Final estimated plane (ignoring z component)
    Eigen::Matrix3d sum = VAL0 * A1 * A2 * A3;
    Eigen::Vector3d final = LieAlgebra::log(sum);

    if (debug) {
        std::cout << "\t\t sum = " << sum << std::endl;
        std::cout << "\t\t" << LieAlgebra::log(sum).transpose() << std::endl;
    }

//    if ( fabs(final[2]) > 1e-4 )
//        std::cout << "FINAL: " << final.transpose() << std::endl;
//    std::cout << "Test diff01: " << diff01.transpose() << std::endl;
//    std::cout << "Test diff12: " << diff12.transpose() << std::endl;
//    std::cout << "Test diff23: " << diff23.transpose() << std::endl;

    // Derivative for normal
//    Eigen::Matrix3d A1d = bDeriv(1) * A1 * LieAlgebra::skew(diff01); // TODO: What about 1/dt?
//    Eigen::Matrix3d A2d = bDeriv(2) * A2 * LieAlgebra::skew(diff12);
//    Eigen::Matrix3d A3d = bDeriv(3) * A3 * LieAlgebra::skew(diff23);

    // It is a little bit shorter as it is exp(log(...)) * [0 0 1]'
//    jacobianN = VAL0 * (A1d * A2 * A3 + A1 * A2d * A3 + A1 * A2 * A3d) * Eigen::Vector3d::UnitZ();

    /// Checking equation from https://arxiv.org/pdf/1911.08860.pdf
    Eigen::Vector3d w2 = bDeriv(1) * diff01;
    Eigen::Vector3d w3 = LieAlgebra::deskew(A2.transpose() * LieAlgebra::skew(w2) * A2) + bDeriv(2) * diff12;
    Eigen::Vector3d w4 = LieAlgebra::deskew(A3.transpose() * LieAlgebra::skew(w3) * A3) + bDeriv(3) * diff23;

    // It is a little bit shorter as it is exp(log(...)) * [0 0 1]'
    jacobianN = sum * LieAlgebra::skew(w4) * Eigen::Vector3d::UnitZ();
    /// END

    // Performing bspline for d
    Eigen::Vector4d valDiff = Eigen::Vector4d::Zero();
    valDiff(0) = values(2,0);
    valDiff(1) = values(2,1) - values(2,0);
    valDiff(2) = values(2,2) - values(2,1);
    valDiff(3) = values(2,3) - values(2,2);

    final(2) = b.transpose() * valDiff;

    // Derivative for d
    Eigen::Matrix<double, 1, 1> jD = bDeriv.transpose() * valDiff;
    jacobianD = jD(0);

    return final;
}

Eigen::Vector3d BSpline::interMinPlaneSphere(Eigen::Matrix<double, 3, 4> & values, double u) {

    // Preparing bSpline (could be done once if time is an issue)
//    Eigen::Matrix<double, 4, 4> bSplineMatrix;
//    bSplineMatrix <<  6, 0, 0, 0,    5, 3, -3, 1,    1, 3, 3, -2,    0, 0, 0, 1;
//    double bSplineCoeff = 1.0 / 6;
    Eigen::Matrix<double, 4, 4> bSplineMatrix;
    bSplineMatrix << 1, -3, 3, -1,  4, 0, -6, 3,   1, 3, 3, -3,    0, 0, 0, 1;
    double bSplineCoeff = 1.0 / 6;

    // Coefficient of bspline: coeff * C * vecU
    Eigen::Matrix<double, 4, 1> vecU;
    vecU << 1, u, u*u, u*u*u;
    Eigen::Matrix<double, 4, 1> b = bSplineCoeff * bSplineMatrix * vecU;

    // Initial guess
    Eigen::Vector2d planeA = values.col(0).head<2>();
    Eigen::Vector2d planeB = values.col(1).head<2>();
    Eigen::Vector2d planeC = values.col(2).head<2>();
    Eigen::Vector2d planeD = values.col(3).head<2>();


    // TODO: In A1 it is defined on the original non-minimal plane, what about the denominator?
    Eigen::Vector2d init = b(0) * planeA + b(1) * planeB + b(2) * planeC + b(3) * planeD;

    // Iterative search for best interpolation
    while(true) {
        Eigen::Vector2d err = b(0) * (planeA - init) + b(1) * (planeB - init) + b(2) * (planeC - init)
                              + b(3) * (planeD - init);

        init = init + err;

        if (err.norm() < 1e-7)
            break;

        std::cout << "Next iteration?" << std::endl;
    }


    Eigen::Vector3d final;
    final.head<2>() = init;

    // Performing bspline for d
    Eigen::Matrix<double, 4, 1> vd = values.row(2);
    final(2) =  vd.transpose() * b;

    return final;
}



