// The body of a unit of wmtk_test_eigen_heap (see eigen_heap_units.hpp). The including file
// defines WMTK_EIGEN_HEAP_UNIT, the function returning the unit, and WMTK_EIGEN_HEAP_UNIT_NAME.
#include <Eigen/Core>
#include <cstdint>

#include "eigen_heap_units.hpp"

namespace wmtk_eigen_heap {
namespace {

int default_align()
{
    return EIGEN_DEFAULT_ALIGN_BYTES;
}

int malloc_already_aligned()
{
    return EIGEN_MALLOC_ALREADY_ALIGNED;
}

void* make(int n)
{
    auto* v = new Eigen::VectorXd(n);
    for (int i = 0; i < n; ++i) (*v)[i] = i;
    return v;
}

void destroy(void* v)
{
    delete static_cast<Eigen::VectorXd*>(v);
}

void shrink(void* v, int n)
{
    static_cast<Eigen::VectorXd*>(v)->conservativeResize(n);
}

bool holds_iota(const void* v, int n)
{
    const Eigen::VectorXd& x = *static_cast<const Eigen::VectorXd*>(v);
    if (x.size() != n) return false;
    for (int i = 0; i < n; ++i) {
        if (x[i] != i) return false;
    }
    return true;
}

size_t misaligned_blocks()
{
    size_t count = 0;
    for (int k = 0; k < 64; ++k) {
        void* p = Eigen::internal::aligned_malloc(8 + 24 * k);
        if (reinterpret_cast<std::uintptr_t>(p) % EIGEN_DEFAULT_ALIGN_BYTES != 0) ++count;
        Eigen::internal::aligned_free(p);
    }
    return count;
}

} // namespace

Unit WMTK_EIGEN_HEAP_UNIT()
{
    return {
        WMTK_EIGEN_HEAP_UNIT_NAME,
        default_align,
        malloc_already_aligned,
        make,
        destroy,
        shrink,
        holds_iota,
        misaligned_blocks};
}

} // namespace wmtk_eigen_heap
