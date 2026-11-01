#pragma once

#include <cstddef>

// The units of wmtk_test_eigen_heap: the same functions, built by each of its translation units
// with different vectorization settings (tests/CMakeLists.txt), so that each one inlines Eigen's
// allocator as its settings say -- or calls whichever copy of it the linker kept.
namespace wmtk_eigen_heap {

struct Unit
{
    const char* name;
    /// The unit's EIGEN_DEFAULT_ALIGN_BYTES and EIGEN_MALLOC_ALREADY_ALIGNED.
    int (*default_align)();
    int (*malloc_already_aligned)();
    /// A new Eigen::VectorXd of n coefficients 0, 1, ..., n - 1, and its deletion.
    void* (*make)(int n);
    void (*destroy)(void* v);
    /// conservativeResize(n), and whether the vector is 0, 1, ..., n - 1.
    void (*shrink)(void* v, int n);
    bool (*holds_iota)(const void* v, int n);
    /// How many of 64 blocks Eigen::internal::aligned_malloc() gives this unit are not aligned to
    /// its EIGEN_DEFAULT_ALIGN_BYTES, which its vectorized code assumes.
    size_t (*misaligned_blocks)();
};

Unit plain_unit();
Unit novec_unit();
Unit avx_unit();

} // namespace wmtk_eigen_heap
