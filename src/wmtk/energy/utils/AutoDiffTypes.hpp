#include "autodiff.h"


namespace wmtk {
namespace energy {
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
void get_local_vector(const Eigen::MatrixXd& data, const int size, AutoDiffVect& local_vector)
{
    typedef typename AutoDiffVect::Scalar T;
    const AutoDiffAllocator<T> allocate_auto_diff_scalar;
    local_vector.resize(size);
    for (int i = 0; i < size; ++i) {
        local_vector(i) = allocate_auto_diff_scalar(i, data(i));
    }
}
} // namespace energy
} // namespace wmtk