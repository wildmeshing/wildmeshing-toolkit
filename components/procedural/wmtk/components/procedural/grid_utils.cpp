#include "grid_utils.hpp"
namespace wmtk::components::procedural {

namespace {
template <int64_t D>
int64_t _grid_index(const std::array<int64_t, D>& d, const std::array<int64_t, D>& i)
{
    int64_t v = i[0];
    for (int64_t j = 1; j < D; ++j) {
        v = v * d[j] + i[j];
    }
    return v;
}
} // namespace

int64_t grid_index(const std::array<int64_t, 3>& d, const std::array<int64_t, 3>& i)
{
    return _grid_index<3>(d, i);
}
int64_t grid_index(const std::array<int64_t, 2>& d, const std::array<int64_t, 2>& i)
{
    return _grid_index<2>(d, i);
}
} // namespace wmtk::components::procedural
