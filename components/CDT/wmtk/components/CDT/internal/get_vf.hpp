#pragma once
#include <tuple>
#include <vector>

namespace wmtk::components::internal {

std::tuple<std::pair<std::vector<double>, uint32_t>, std::pair<std::vector<uint32_t>, uint32_t>>
get_vf(const TriMesh& trimesh);
}
