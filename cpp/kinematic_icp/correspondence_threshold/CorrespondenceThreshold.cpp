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
#include "CorrespondenceThreshold.hpp"

#include <cmath>
#include <sophus/se3.hpp>

namespace {
double ModelError(const Sophus::SE3d &pose, const double max_range) {
    const double &theta = pose.so3().logAndTheta().theta;
    const double &delta_rot = 2.0 * max_range * std::sin(theta / 2.0);
    const double &delta_trans = pose.translation().norm();
    return delta_trans + delta_rot;
};
}  // namespace

namespace kinematic_icp {
CorrespondenceThreshold::CorrespondenceThreshold(const double map_discretization_error,
                                                 const double max_range,
                                                 const bool use_adaptive_threshold,
                                                 const double fixed_threshold)
    : map_discretization_error_(map_discretization_error),
      max_range_(max_range),
      use_adaptive_threshold_(use_adaptive_threshold),
      fixed_threshold_(fixed_threshold),
      model_sse_(0.0),
      num_samples_(1e-8) {}

double CorrespondenceThreshold::ComputeThreshold() const {
    const double moder_error_variance = std::sqrt(model_sse_ / num_samples_);
    const double adaptive_threshold = 3.0 * (map_discretization_error_ + moder_error_variance);
    return use_adaptive_threshold_ ? adaptive_threshold : fixed_threshold_;
}

void CorrespondenceThreshold::UpdateModelError(const Sophus::SE3d &current_deviation) {
    if (!use_adaptive_threshold_) return;

    const double &model_error = ModelError(current_deviation, max_range_);
    model_sse_ += model_error * model_error;
    num_samples_ += 1.0;
}

}  // namespace kinematic_icp
