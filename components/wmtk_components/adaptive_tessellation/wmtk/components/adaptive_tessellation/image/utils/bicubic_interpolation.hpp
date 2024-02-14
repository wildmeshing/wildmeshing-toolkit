#pragma once

#include <wmtk/function/utils/autodiff.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <type_traits>
#include "SamplingParameters.hpp"
namespace wmtk::components ::image {
inline double get_value(float x)
{
    return static_cast<double>(x);
}
inline double get_value(double x)
{
    return x;
}
inline double get_value(
    DScalar2<double, Eigen::Matrix<double, 2, 1>, Eigen::Matrix<double, 2, 2>> x)
{
    return x.getValue();
}

inline double get_value(
    DScalar2<double, Eigen::Matrix<double, -1, 1>, Eigen::Matrix<double, -1, -1>> x)
{
    return x.getValue();
}

inline double get_value(
    DScalar2<double, Eigen::Matrix<double, -1, 1, 0, 3, 1>, Eigen::Matrix<double, -1, -1, 0, 3, 3>>
        x)
{
    return x.getValue();
}

template <class T>
using BicubicVector = Eigen::Matrix<T, 16, 1>;
using BicubicMatrix = Eigen::Matrix<float, 16, 16>;
BicubicVector<float> extract_samples(
    const size_t width,
    const size_t height,
    const float* buffer,
    const double xx,
    const double yy,
    const IMAGE_WRAPPING_MODE mode_x,
    const IMAGE_WRAPPING_MODE mode_y);

BicubicMatrix make_samples_to_bicubic_coeffs_operator();

const BicubicMatrix& get_bicubic_matrix();

template <class T>
std::decay_t<T> eval_bicubic_coeffs(const BicubicVector<float>& coeffs, const T& sx, const T& sy)
{
    using ImageScalar = std::decay_t<T>;

    const auto xx = sx - (floor(get_value(sx) - 0.5f) + 0.5f);
    const auto yy = sy - (floor(get_value(sy) - 0.5f) + 0.5f);
    assert(0 <= get_value(xx) && get_value(xx) < 1);
    assert(0 <= get_value(yy) && get_value(yy) < 1);

    BicubicVector<ImageScalar> vv;

    vv(0) = 1;
    vv(1) = xx;
    vv(2) = pow(xx, 2);
    vv(3) = pow(xx, 3);

    vv(4) = yy;
    vv(5) = xx * yy;
    vv(6) = pow(xx, 2) * yy;
    vv(7) = pow(xx, 3) * yy;

    vv(8) = pow(yy, 2);
    vv(9) = xx * pow(yy, 2);
    vv(10) = pow(xx, 2) * pow(yy, 2);
    vv(11) = pow(xx, 3) * pow(yy, 2);

    vv(12) = pow(yy, 3);
    vv(13) = xx * pow(yy, 3);
    vv(14) = pow(xx, 2) * pow(yy, 3);
    vv(15) = pow(xx, 3) * pow(yy, 3);

    return coeffs.cast<ImageScalar>().dot(vv);
}

} // namespace wmtk::components::image
