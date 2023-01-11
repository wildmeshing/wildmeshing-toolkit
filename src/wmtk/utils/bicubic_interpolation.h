#pragma once

#include <Eigen/Dense>

enum WrappingMode { REPEAT, MIRROR_REPEAT, CLAMP_TO_EDGE };
namespace wmtk {

using BicubicVector = Eigen::Matrix<float, 16, 1>;
using BicubicMatrix = Eigen::Matrix<float, 16, 16>;

BicubicVector extract_samples(
    const size_t width,
    const size_t height,
    const float * buffer,
    const float xx,
    const float yy,
    const WrappingMode mode_x,
    const WrappingMode mode_y);

BicubicMatrix make_samples_to_bicubic_coeffs_operator();

float eval_bicubic_coeffs(const BicubicVector& coeffs, const float xx, const float yy);

} // namespace wmtk
