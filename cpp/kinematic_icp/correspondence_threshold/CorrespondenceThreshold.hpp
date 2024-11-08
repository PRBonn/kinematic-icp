// MIT License
//
// Copyright (c) 2024 Tiziano Guadagnino, Benedikt Mersch, Ignacio Vizzo, Cyrill
// Stachniss.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#pragma once

#include <cmath>
#include <sophus/se3.hpp>

namespace kinematic_icp {

struct CorrespondenceThreshold {
    explicit CorrespondenceThreshold(const double map_discretization_error,
                                     const double max_range,
                                     const bool use_adaptive_threshold,
                                     const double fixed_threshold);

    void UpdateModelError(const Sophus::SE3d &current_deviation);

    double ComputeThreshold() const;

    inline void Reset() {
        model_sse_ = 0.0;
        num_samples_ = 1e-8;
    }

    // configurable parameters
    double map_discretization_error_;  // <-- Error introduced by the fact that we have a discrete
                                       // set of points of the surface we are measuring
    double max_range_;
    bool use_adaptive_threshold_;
    double fixed_threshold_;  // <-- Ignored if use_adaptive_threshold_ = true

    // Local cache for ccomputation
    double model_sse_;
    double num_samples_;
};
}  // namespace kinematic_icp
