#pragma once
#include <array>
#include <cstdint>

namespace wmtk::components::procedural {


int64_t grid_index(const std::array<int64_t, 3>& dimensions, const std::array<int64_t, 3>& i);
int64_t grid_index(const std::array<int64_t, 2>& dimensions, const std::array<int64_t, 2>& i);
} // namespace wmtk::components::procedural
