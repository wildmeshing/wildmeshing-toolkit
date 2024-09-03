#pragma once
#include <array>
#include <cstdint>

namespace wmtk {
class Mesh;
class Tuple;
} // namespace wmtk

namespace wmtk::tests::tools {
int8_t global_index_max_subdart_size(const Mesh& m, const Tuple& a, const Tuple& b);
int8_t global_index_max_subdart_size(
    const std::array<int64_t, 4>& a,
    const std::array<int64_t, 4>& b);
} // namespace wmtk::tests::tools
