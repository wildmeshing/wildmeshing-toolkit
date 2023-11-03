#pragma once
#include <wmtk/TriMesh.h>
namespace wmtk::utils {

bool is_valid_trimesh_topology(const TriMesh& m);
std::array<std::vector<size_t>, 2> validate_trimesh_topology(const TriMesh& m);

} // namespace wmtk::utils

