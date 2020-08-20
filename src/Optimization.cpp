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
#include "Optimization.h"
#include "utils/BSpline.h"
#include <cstdio>


#include "../EXTERNAL/g2o/g2o/core/block_solver.h"
#include "../EXTERNAL/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "../EXTERNAL/g2o/g2o/core/optimization_algorithm_gauss_newton.h"
#include "../EXTERNAL/g2o/g2o/solvers/pcg/linear_solver_pcg.h"
#include "../EXTERNAL/g2o/g2o/solvers/dense/linear_solver_dense.h"
#include "../EXTERNAL/g2o/g2o/solvers/eigen/linear_solver_eigen.h"
#include "../EXTERNAL/g2o/g2o/core/sparse_optimizer_terminate_action.h"
#include "../EXTERNAL/g2o/g2o/core/robust_kernel_impl.h"

#include "local_g2o/vertex_one.h"
#include "local_g2o/edge_one_prior.h"
#include "local_g2o/edge_se3_time.h"



class EdgeContribution {
public:
    int index;
    int indexForSorting;
    Eigen::Matrix<double, 7, 1> contribution;
    g2o::EdgeSE3Time* edge;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

bool Optimization::findApproximatedChessboard(double pointTimestamp, Eigen::Vector3d &chessboardMinRepr, Eigen::Vector3d &jacobianN,
        double &jacobianD) {

    // Finding proper chessboards based on time
    int index = 0;
    for (;index<chessboards.size();index++){
        if (chessboards[index].timestamp > pointTimestamp)
            break;
    }

    // Let's skip it if the point is prior to first two detection or after the last two detections
    if (index <= 1  || index +1 >= chessboards.size()) {
            return false;
    }

    // Check cubic spline
    const double &timeA = chessboards[index - 2].timestamp;
    const double &timeB = chessboards[index - 1].timestamp;
    const double &timeC = chessboards[index].timestamp;
    const double &timeD = chessboards[index + 1].timestamp;

    const double &timeAB = timeB - timeA;
    const double &timeBC = timeC - timeB;
    const double &timeCD = timeD - timeC;

    // Sufficiently equidistant distribution
    // TODO: It could be made better with some relative thresholds
    // TODO: Irregular grid would be just the best!
    if (fabs(timeAB - timeBC) < 0.004 && fabs(timeBC - timeCD) < 0.004 && fabs(timeAB - timeCD) < 0.004) {

        // Time from 0 to 1 (0 meaning pointB, 1 meaning point C)
        double u = (pointTimestamp - timeB) / timeBC;

        // B-spline
        Eigen::Matrix<double, 3, 4> test;
        test.col(0) = chessboards[index - 2].planeEqMinRepr;
        test.col(1) = chessboards[index - 1].planeEqMinRepr;
        test.col(2) = chessboards[index].planeEqMinRepr;
        test.col(3) = chessboards[index + 1].planeEqMinRepr;

        chessboardMinRepr = BSpline::interMinPlaneCum(test, u, jacobianN, jacobianD);

        // Jacobians have \Delta t in the denominator
        jacobianN = jacobianN / timeBC;
        jacobianD = jacobianD / timeBC;

        return true;
    }

    return false;
}

void Optimization::optimizeCameraLaser(double & estimatedTimeShift,
                                       Eigen::Matrix4d & estimatedLaserInCamera,
                                       double &rmse,
                                       int &good, int &total,
                                       std::vector<ConvexityResult> & convex,
                                       bool timeShiftFixed) {

    std::cout << std::endl;
    std::cout << " ------------------------- " << std::endl;
    std::cout << " -- optimizeCameraLaser -- " << std::endl;
    std::cout << std::endl;
    optimizer.clear();

    const double timePrior = estimatedTimeShift;
    const double timePriorWeight = 0.0;
    bool verbose = true;

    // When error to plane exceeds 0.1 then we have an outlier
    const double planeHuber = 0.1;
    const double planeHuberSq = planeHuber * planeHuber;

    // Initial pose from existing guess
//    std::cout << "estimatedLaserInCamera = " << std::endl << estimatedLaserInCamera << std::endl;
    g2o::SE3Quat poseSE3Quat(estimatedLaserInCamera.block<3, 3>(0, 0), estimatedLaserInCamera.block<3, 1>(0, 3));
//    std::cout << "poseSE3Quat = " << std::endl << poseSE3Quat.to_homogeneous_matrix() << std::endl;

    // Optimizer
    optimizer.setVerbose(verbose);

    // Creating linear solver
    std::unique_ptr<g2o::BlockSolverX::LinearSolverType> linearSolver;

    linearSolver = g2o::make_unique<g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType >>();
    auto blockSolver = g2o::make_unique<g2o::BlockSolverX>(std::move(linearSolver));

    g2o::SparseOptimizerTerminateAction *terminateAction = new g2o::SparseOptimizerTerminateAction;
//    terminateAction->setGainThreshold(0.000000000000000000001); // 0.0000001
    terminateAction->setMaxIterations(250);
    optimizer.addPostIterationAction(terminateAction);

    g2o::OptimizationAlgorithmLevenberg *optimizationAlgorithm = new g2o::OptimizationAlgorithmLevenberg(
            std::move(blockSolver));

    // Setting Levenberg as optimization algorithm
    optimizer.setAlgorithm(optimizationAlgorithm);

    // New vertex - relative transformation
    const unsigned int poseId = 0;
    g2o::VertexSE3Expmap *vertexPose = new g2o::VertexSE3Expmap();
    vertexPose->setEstimate(poseSE3Quat);
    vertexPose->setId(poseId);
    vertexPose->setFixed(false);
    optimizer.addVertex(vertexPose);

    // New vertex - time shift
    const unsigned int timeId = 1;
    g2o::VertexOne *vertexOne = new g2o::VertexOne();
    Eigen::Matrix<double, 1, 1> estOne;
    estOne << timePrior;
    vertexOne->setEstimate(estOne);
    vertexOne->setId(timeId);
    vertexOne->setFixed(timeShiftFixed);
    optimizer.addVertex(vertexOne);

    // Adding prior to time
    g2o::EdgeOnePrior *ePrior = new g2o::EdgeOnePrior();
    ePrior->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(timeId)));
    Eigen::Matrix<double, 1, 1> obs;
    obs << timePrior;
    ePrior->setMeasurement(obs);
    ePrior->setInformation(g2o::Vector1::Identity() * timePriorWeight);
    optimizer.addEdge(ePrior);

    std::cout << "Number of lidar points: " << lidarPoints.size() << std::endl;
    std::cout << "Number of different chessboards: " << chessboards.size() << std::endl;

    // Each point contributes independence constraint
    unsigned int initialGoodEdges = 0;
    for (int i = 0; i < lidarPoints.size(); i++) {

        // Adding measurement
        g2o::EdgeSE3Time *e = new g2o::EdgeSE3Time();
        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(poseId)));
        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(timeId)));

        // information matrix
        Eigen::Matrix<double, 1, 1> infMat = Eigen::Matrix<double, 1, 1>::Identity();
        e->setInformation(infMat);

        // point in lidar
        const PointStamped &pointStamped = lidarPoints[i];
        e->pointTimestamp = pointStamped.timestamp;
        e->pointInLidar = pointStamped.point;

        // Access to data
        e->calibrationData = this;

        // Robust
        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
        e->setRobustKernel(rk);
        rk->setDelta(planeHuber);

        // Let's check if point does have a nice match on the continuous plane space
        Eigen::Vector3d ignoredPlaneEq, ignoredJacobianN;
        double ignoreJacobianD;

        bool matchFound = findApproximatedChessboard(pointStamped.timestamp, ignoredPlaneEq, ignoredJacobianN,
                ignoreJacobianD);
        if (!matchFound)
            e->setLevel(1);
        else
            initialGoodEdges++;

        // Adding edge
        optimizer.addEdge(e);

        e->computeError();
//        std::cout << "e->pointTimestamp = " << e->pointTimestamp << ", e->pointInLidar = " << e->pointInLidar.transpose() << std::endl;
    }


    // Initial guesses
//    g2o::VertexSE3Expmap *vPoseIn = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(poseId));
//    g2o::SE3Quat inEstimate = vPoseIn->estimate();
//    std::cout << "Initial laser in camera : " << std::endl << inEstimate.to_homogeneous_matrix() << std::endl;

    // We have to select some of the constraints
    if (ratioOfTotal < 1.00) {
            randomEdgeSelection(ratioOfTotal);
    }

    /// Optimization !!!
    std::cout << "Starting optimization with initialGoodEdges = " << initialGoodEdges << std::endl;

    // Optimize 2 times:
    // - iter 0 - removing outliers (with huber)
    // - iter 1 - removing outliers (without huber)
    for (int i = 0; i < 3; i++) {

        // Initialize system and optimize
        optimizer.initializeOptimization();
        optimizer.optimize(200);

        // Consider all edges
        int countBadEdges = 0, countGoodEdges = 0, countRejectedEdges = 0;
        double chi2inliers = 0;
        for (g2o::HyperGraph::EdgeSet::const_iterator it = optimizer.edges().begin();
             it != optimizer.edges().end(); ++it) {

            // For all pose-plane constraints
            g2o::EdgeSE3Time *ePointToPlane = dynamic_cast<g2o::EdgeSE3Time *>(*it);
            if (ePointToPlane) {

                // Computing current error
                ePointToPlane->computeError();
                const float chi2 = ePointToPlane->chi2();

                // Mark it as outlier
                if(i != 2) {
                    if (ePointToPlane->level() == 0 && chi2 > planeHuberSq) {
                        ePointToPlane->setLevel(1);
                        countBadEdges++;
                    } else if (ePointToPlane->level() == 1 && chi2 < planeHuberSq) {
                        ePointToPlane->setLevel(0);
                        chi2inliers += chi2;
                        countGoodEdges++;
                    }
                }

                // Count chi2
                if (ePointToPlane->level() == 0) {
                    chi2inliers += chi2;
                    countGoodEdges++;
                }
                else if (ePointToPlane->level() == 1)
                    countBadEdges++;
                else if (ePointToPlane->level() == 2)
                    countRejectedEdges++;

                // Turn off robust cost function
                if (i == 0)
                    ePointToPlane->setRobustKernel(0);
            }
        }

        if (verbose) {
            unsigned int totalEdges = countGoodEdges + countBadEdges + countRejectedEdges;
            std::cout << "-----> Opt. step: " << i << std::endl;
            std::cout << "       countGoodEdges: " << countGoodEdges << " / " << totalEdges << std::endl;
            std::cout << "       countBadEdges: " << countBadEdges << " / " << totalEdges << std::endl;
            std::cout << "       countRejectedEdges: " << countRejectedEdges << " / " << totalEdges << std::endl;
            std::cout << "       total         : " << optimizer.edges().size() << std::endl;
        }
        std::cout << "Opt. " << i << " error : " << chi2inliers << " avg: "
                  << std::sqrt(chi2inliers / countGoodEdges) * 100 << " cm" << std::endl;

        good = countGoodEdges;
        total = optimizer.edges().size();
        rmse = std::sqrt(chi2inliers / countGoodEdges);
    }

    // Laser in Camera transformation
    g2o::VertexSE3Expmap *vPoseOut = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(poseId));
    g2o::SE3Quat outEstimate = vPoseOut->estimate();

    // Time calibration
    g2o::VertexOne *vTimeOut = static_cast<g2o::VertexOne *>(optimizer.vertex(timeId));
    double timeCalibration = vTimeOut->estimate()[0];

    std::cout << std::endl << "--- Calibration result ---" << std::endl;
    std::cout << "Optimized time shift : " << timeCalibration << std::endl;
    std::cout << "Optimized laser in camera : " << std::endl << outEstimate.to_homogeneous_matrix() << std::endl;

    // Save values to return
    estimatedTimeShift = timeCalibration;
    estimatedLaserInCamera = outEstimate.to_homogeneous_matrix();

    // Check convexity
    if(false) {
        std::vector<double> times;
        for (double t = -0.02; t <= 0.02; t += 0.0001)
            times.push_back(t);

        // Tests for different times
        for (int j = 0; j < times.size(); j++) {

            double totalChi2 = 0;
            int countChi2 = 0;

            // For all edges
            for (g2o::HyperGraph::EdgeSet::const_iterator it = optimizer.edges().begin();
                 it != optimizer.edges().end(); ++it) {

                g2o::EdgeSE3Time *ePointToPlane = dynamic_cast<g2o::EdgeSE3Time *>(*it);
                if (ePointToPlane && ePointToPlane->level() != 2) {

                    // Set t as original time + artificial shift
                    double origTime = ePointToPlane->pointTimestamp;
                    ePointToPlane->pointTimestamp = origTime + times[j];

                    // Computing current error
                    ePointToPlane->computeError();
                    const float chi2 = ePointToPlane->chi2();

                    // Is it an inlier?
//                if (chi2 <= planeHuberSq) {
                    if (ePointToPlane->level() == 0) {
                        totalChi2 += chi2;
                        countChi2++;
                    }

                    // Go back to original time
                    ePointToPlane->pointTimestamp = origTime;
                }
            }

            // Save time increment and achieved result
            ConvexityResult cr;
            cr.time = times[j];
            cr.goodEdgesCount = countChi2;
            cr.avgChi2 = totalChi2;
            convex.push_back(cr);
        }
    }
}

void Optimization::randomEdgeSelection(double ratio) {

    // Copy all inliers and select the rest as rejected
    std::vector<g2o::EdgeSE3Time *> edges;
    for (g2o::HyperGraph::EdgeSet::const_iterator it = optimizer.edges().begin();
         it != optimizer.edges().end(); ++it) {

        // For all pose-plane constraints
        g2o::EdgeSE3Time *ePointToPlane = dynamic_cast<g2o::EdgeSE3Time *>(*it);
        if (ePointToPlane)
        {
            if (ePointToPlane->level() == 0) {
               edges.push_back(ePointToPlane);
            }

            // Mark edge as no selected
            ePointToPlane->setLevel(2);
        }
    }

    // Shuffle inliers
    std::random_shuffle ( edges.begin(), edges.end() );

//    std::cout << "random verificaiton " << edges[0]->pointInLidar.transpose() << std::endl;

    // Use wanted number of inliers
    const unsigned int numberToSelect = ratio * edges.size();
    for (int i=0;i < numberToSelect;i++) {
        edges[i]->setLevel(0);
    }
}