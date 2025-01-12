#pragma once
#include <cstdint>
#include <vector>

namespace wmtk {
class Mesh;
} // namespace wmtk
namespace wmtk::components::multimesh {


void from_facet_surjection(Mesh& parent, Mesh& child, const std::vector<int64_t>& parent_simplices);

} // namespace wmtk::components::multimesh
