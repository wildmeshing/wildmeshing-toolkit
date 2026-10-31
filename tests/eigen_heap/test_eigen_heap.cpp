#include <catch2/catch_test_macros.hpp>
#include <vector>

#include "eigen_heap_units.hpp"

using namespace wmtk_eigen_heap;

namespace {
// The units, the AVX one only where it is built and the CPU runs it. Where it is built and the CPU
// does not, none: the linker may have kept its copies of Eigen's inline functions for the others.
std::vector<Unit> units(bool& skip)
{
    std::vector<Unit> out = {plain_unit(), novec_unit()};
    skip = false;
#ifdef WMTK_TEST_EIGEN_HEAP_AVX
    if (!__builtin_cpu_supports("avx2") || !__builtin_cpu_supports("fma")) skip = true;
    out.push_back(avx_unit());
#endif
    return out;
}
} // namespace

// cmake/recipes/eigen.cmake: units built with different vectorization -- one linking ipc-toolkit,
// one built for AVX -- must allocate a dynamic matrix the same way, or one frees with free() a
// block the other made with handmade_aligned_malloc(), and they must align it the same, or one
// reads with aligned AVX loads a block the other aligned to 16.
TEST_CASE("eigen-heap-one-allocator-across-units", "[eigen]")
{
    bool skip = false;
    const std::vector<Unit> all = units(skip);
    if (skip) SKIP("the AVX unit is built, but this CPU has no AVX2/FMA");

    for (const Unit& u : all) {
        INFO("unit " << u.name);
        CHECK(u.default_align() == all.front().default_align());
        CHECK(u.malloc_already_aligned() == all.front().malloc_already_aligned());
        CHECK(u.misaligned_blocks() == 0);
    }
    for (const Unit& a : all) {
        for (const Unit& b : all) {
            INFO("allocated in " << a.name << ", resized and freed in " << b.name);
            for (int n = 1; n < 200; n += 7) {
                void* v = a.make(n + 9);
                b.shrink(v, n);
                CHECK(b.holds_iota(v, n));
                b.destroy(v);
            }
        }
    }
}
