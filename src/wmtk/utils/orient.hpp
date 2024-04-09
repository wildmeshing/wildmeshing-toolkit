#pragma once

#include <wmtk/Types.hpp>

namespace wmtk::utils {

template <typename T>
int wmtk_orient3d(
    const Eigen::Vector3<T>& p0,
    const Eigen::Vector3<T>& p1,
    const Eigen::Vector3<T>& p2,
    const Eigen::Vector3<T>& p3)
{
    throw std::runtime_error("unsupported type");
}

template <>
int wmtk_orient3d(
    const Eigen::Vector3<Rational>& p0,
    const Eigen::Vector3<Rational>& p1,
    const Eigen::Vector3<Rational>& p2,
    const Eigen::Vector3<Rational>& p3);

template <>
int wmtk_orient3d(
    const Eigen::Vector3<double>& p0,
    const Eigen::Vector3<double>& p1,
    const Eigen::Vector3<double>& p2,
    const Eigen::Vector3<double>& p3);


template <typename T>
int wmtk_orient2d(
    const Eigen::Vector2<T>& p0,
    const Eigen::Vector2<T>& p1,
    const Eigen::Vector2<T>& p2)
{
    throw std::runtime_error("unsupported type");
}

template <>
int wmtk_orient2d(
    const Eigen::Vector2<Rational>& p0,
    const Eigen::Vector2<Rational>& p1,
    const Eigen::Vector2<Rational>& p2);

template <>
int wmtk_orient2d(
    const Eigen::Vector2<double>& p0,
    const Eigen::Vector2<double>& p1,
    const Eigen::Vector2<double>& p2);

int wmtk_orient1d(const Rational& p0, const Rational& p1);

int wmtk_orient1d(double p0, double p1);

} // namespace wmtk::utils