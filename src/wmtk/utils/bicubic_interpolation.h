#pragma once

#include <Eigen/Dense>

namespace wmtk {

using BicubicVector = Eigen::Matrix<float, 16, 1>;
using BicubicMatrix = Eigen::Matrix<float, 16, 16>;

BicubicVector extract_samples(
    const size_t width,
    const size_t height,
    const std::vector<float>& buffer,
    const float xx,
    const float yy);

BicubicMatrix make_samples_to_bicubic_coeffs_operator();

float eval_bicubic_coeffs(const BicubicVector& coeffs, const float xx, const float yy);

} // namespace wmtk