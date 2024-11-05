#pragma once
#include <array>
#include <cstdint>

namespace wmtk {
class Mesh;
class Tuple;
} // namespace wmtk

namespace wmtk::tests::tools {
std::array<int64_t, 4> global_ids(const Mesh& m, const Tuple& a);
}
