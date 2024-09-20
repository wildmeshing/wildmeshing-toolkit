#pragma once
#include <wmtk/Mesh.hpp>
#include <wmtk/TriMesh.hpp>

namespace wmtk::components {

std::shared_ptr<Mesh>
CDT(const TriMesh& trimesh, const bool inner_only, const bool rational_output);

} // namespace wmtk::components
