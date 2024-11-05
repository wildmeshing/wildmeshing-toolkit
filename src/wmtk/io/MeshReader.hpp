#pragma once
#include <vector>
#include "read_mesh.hpp"

namespace wmtk {

class Mesh;

std::shared_ptr<Mesh> read_mesh(
    const std::filesystem::path& filename,
    const bool ignore_z_if_zero = false,
    const std::vector<std::string>& tetrahedron_attributes = {});

} // namespace wmtk
