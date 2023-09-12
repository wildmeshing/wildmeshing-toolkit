#pragma once
#include "autodiff.h"

namespace wmtk {
namespace function {
using DScalar = DScalar2<double, Eigen::Matrix<double, -1, 1>, Eigen::Matrix<double, -1, -1>>;
using Scalar = typename DScalar::Scalar;
template <class T>
class AutoDiffAllocator
{
public:
    T operator()(const int i, double v) const { return T(i, v); }
};

template <>
class AutoDiffAllocator<double>
{
public:
    double operator()(const int i, double v) const { return v; }
};

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

template <typename AutoDiffVect>
AutoDiffVect get_T_vector(const Eigen::MatrixXd& data, const int size)
{
    typedef typename AutoDiffVect::Scalar T;
    AutoDiffVect T_vector;
    DiffScalarBase::setVariableCount(size);
    const AutoDiffAllocator<T> allocate_auto_diff_scalar;
    T_vector.resize(size);
    for (int i = 0; i < size; ++i) {
        T_vector(i) = allocate_auto_diff_scalar(i, data(i));
    }
    return T_vector;
}
template <typename AutoDiffVect>
void get_double_vector(const AutoDiffVect& T_vector, const int size, Eigen::MatrixXd& double_t)
{
    double_t.resize(size, 1);
    for (int i = 0; i < size; ++i) {
        double_t(i) = T_vector(i).getValue();
    }
}
} // namespace function
} // namespace wmtk