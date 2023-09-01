#include "autodiff.h"

namespace wmtk {
namespace energy {
using DScalar = DScalar2<double, Eigen::VectorXd, Eigen::MatrixXd>;
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

template <typename AutoDiffVect>
void get_local_vector(const Eigen::MatrixXd& data, const int size, AutoDiffVect& T_vector)
{
    typedef typename AutoDiffVect::Scalar T;
    DiffScalarBase::setVariableCount(size);
    const AutoDiffAllocator<T> allocate_auto_diff_scalar;
    T_vector.resize(size);
    for (int i = 0; i < size; ++i) {
        T_vector(i) = allocate_auto_diff_scalar(i, data(i));
    }
}
template <typename AutoDiffVect>
void get_double_vector(const AutoDiffVect& T_vector, const int size, Eigen::MatrixXd& double_t)
{}
} // namespace energy
} // namespace wmtk