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
#ifndef CAMERA_LIDAR_CALIBRATOR_EDGE_SE3_TIME_H
#define CAMERA_LIDAR_CALIBRATOR_EDGE_SE3_TIME_H

#include "g2o/core/base_vertex.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o/types/slam3d/se3_ops.h"
#include "g2o/types/slam3d/se3quat.h"
#include "g2o/types/sba/types_sba.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/types/slam3d/g2o_types_slam3d_api.h"

#include "g2o/config.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/hyper_graph_action.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "vertex_one.h"
#include "LieAlgebra.h"
#include "Optimization.h"

namespace g2o {


    class G2O_TYPES_SLAM3D_API EdgeSE3Time : public BaseBinaryEdge<1, number_t, VertexSE3Expmap, VertexOne> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgeSE3Time();

        bool read(std::istream &is);

        bool write(std::ostream &os) const;

        void computeError();

        virtual number_t initialEstimatePossible(const OptimizableGraph::VertexSet &, OptimizableGraph::Vertex *) { return -1; }

        virtual void linearizeOplus();

        // Measurement information
        double pointTimestamp;
        Vector3 pointInLidar;
        Optimization * calibrationData;

    };
}

#endif //CAMERA_LIDAR_CALIBRATOR_EDGE_SE3_TIME_H
