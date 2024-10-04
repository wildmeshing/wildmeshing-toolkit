#pragma once
#include "read_mesh.hpp"
#include <vector>
namespace wmtk {

class Mesh;

std::shared_ptr<Mesh> read_mesh(
    const std::filesystem::path& filename,
    const bool ignore_z = false,
    const std::vector<std::string>& tetrahedron_attributes = {});

} // namespace wmtk
