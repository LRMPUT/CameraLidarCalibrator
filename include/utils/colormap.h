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
#ifndef CAMERA_LIDAR_CALIBRATOR_COLORMAP_H
#define CAMERA_LIDAR_CALIBRATOR_COLORMAP_H

// Code from: https://stackoverflow.com/questions/7706339/grayscale-to-red-green-blue-matlab-jet-color-scale
namespace jet_color {

    double linear_inter(double val, double y0, double x0, double y1, double x1) {
        return (val - x0) * (y1 - y0) / (x1 - x0) + y0;
    }

    double base(double val) {
        if (val <= -0.75) return 0;
        else if (val <= -0.25) return linear_inter(val, 0.0, -0.75, 1.0, -0.25);
        else if (val <= 0.25) return 1.0;
        else if (val <= 0.75) return linear_inter(val, 1.0, 0.25, 0.0, 0.75);
        else return 0.0;
    }

    double range2interval(double range) {
        return (range * 0.05) - 1.0;
    }

    double red(double range) {
        range = range2interval(range);
        return base(range - 0.5);
    }

    double green(double range) {
        range = range2interval(range);
        return base(range);
    }

    double blue(double range) {
        range = range2interval(range);
        return base(range + 0.5);
    }

};

#endif //CAMERA_LIDAR_CALIBRATOR_COLORMAP_H
