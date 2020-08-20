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
#ifndef G2O_VERTEX_ONE_H
#define G2O_VERTEX_ONE_H

#include "g2o/config.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/hyper_graph_action.h"

#include <Eigen/Core>

namespace g2o {


    typedef Eigen::Matrix<number_t, 1, 1, Eigen::ColMajor> Vector1;

    class VertexOne : public BaseVertex<1, Vector1> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        VertexOne();

        virtual void setToOriginImpl() {
            _estimate.setZero();
        }

        virtual bool setEstimateDataImpl(const number_t *est) {
            _estimate[0] = est[0];
            return true;
        }

        virtual bool getEstimateData(number_t *est) const {
            est[0] = _estimate[0];
            return true;
        }

        virtual int estimateDimension() const {
            return 1;
        }

        virtual bool setMinimalEstimateDataImpl(const number_t *est) {
            return setEstimateData(est);
        }

        virtual bool getMinimalEstimateData(number_t *est) const {
            return getEstimateData(est);
        }

        virtual int minimalEstimateDimension() const {
            return 1;
        }

        virtual void oplusImpl(const number_t *update) {
            _estimate[0] += update[0];
        }

        virtual bool read(std::istream &is);

        virtual bool write(std::ostream &os) const;

    };


}

#endif