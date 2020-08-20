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
#ifndef GRAPHLOCALIZATION_EDGE_ONE_PRIOR_H
#define GRAPHLOCALIZATION_EDGE_ONE_PRIOR_H

#include "vertex_one.h"
#include "g2o/core/base_unary_edge.h"

namespace g2o {

    /**
     * \brief Prior for a single parameters
     */
    class EdgeOnePrior : public BaseUnaryEdge<1, Vector1, VertexOne> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        EdgeOnePrior();

        virtual void setMeasurement(const Vector1 &m) {
            _measurement = m;
        }

        virtual bool setMeasurementData(const number_t *d) {
            _measurement[0] = d[0];
            return true;
        }

        virtual bool getMeasurementData(number_t *d) const {
            d[0] = _measurement[0];
            return true;
        }

        virtual int measurementDimension() const { return 1; }

        virtual void linearizeOplus();

        virtual bool read(std::istream &is);

        virtual bool write(std::ostream &os) const;

        virtual void computeError() {
            const VertexOne *v = static_cast<const VertexOne *>(_vertices[0]);
//            if ( v->estimate() > 0)
                _error = v->estimate() - _measurement;
//            else
//                _error[0] = 99999999999.0;
        }

        virtual number_t initialEstimatePossible(const OptimizableGraph::VertexSet &, OptimizableGraph::Vertex *) { return 0; }
    };

} // end namespace

#endif //GRAPHLOCALIZATION_EDGE_ONE_PRIOR_H
