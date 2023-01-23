#pragma once

#include <Eigen/Dense>
#include <type_traits>
#include "autodiff.h"
enum class WrappingMode { REPEAT, MIRROR_REPEAT, CLAMP_TO_EDGE };
namespace wmtk {

inline int get_floor_value(float x)
{
    return static_cast<int>(floor(x));
}
inline int get_floor_value(double x)
{
    return static_cast<int>(floor(x));
}
inline int get_floor_value(DScalar2<double, Eigen::Vector2d, Eigen::Matrix2d> x)
{
    return static_cast<int>(floor(x.getValue()));
}

template <class T>
using BicubicVector = Eigen::Matrix<T, 16, 1>;
using BicubicMatrix = Eigen::Matrix<float, 16, 16>;

BicubicVector<float> extract_samples(
    const size_t width,
    const size_t height,
    const float* buffer,
    const int xx,
    const int yy,
    const WrappingMode mode_x,
    const WrappingMode mode_y);

BicubicMatrix make_samples_to_bicubic_coeffs_operator();

const BicubicMatrix& get_bicubic_matrix();

template <class T>
std::decay_t<T>
eval_bicubic_coeffs(const wmtk::BicubicVector<float>& coeffs, const T& sx, const T& sy)
{
    using ImageScalar = std::decay_t<T>;
    const auto xx = sx - get_floor_value(sx);
    const auto yy = sy - get_floor_value(sy);
    assert(0 <= get_floor_value(xx) && get_floor_value(xx) < 1);
    assert(0 <= get_floor_value(yy) && get_floor_value(yy) < 1);

    wmtk::BicubicVector<ImageScalar> vv;

    vv(0) = 1;
    vv(1) = xx;
    vv(2) = xx * xx;
    vv(3) = xx * xx * xx;

    vv(4) = yy;
    vv(5) = xx * yy;
    vv(6) = xx * xx * yy;
    vv(7) = xx * xx * xx * yy;

    vv(8) = yy * yy;
    vv(9) = xx * yy * yy;
    vv(10) = xx * xx * yy * yy;
    vv(11) = xx * xx * xx * yy * yy;

    vv(12) = yy * yy * yy;
    vv(13) = xx * yy * yy * yy;
    vv(14) = xx * xx * yy * yy * yy;
    vv(15) = xx * xx * xx * yy * yy * yy;

    return coeffs.cast<ImageScalar>().dot(vv);
}

} // namespace wmtk
