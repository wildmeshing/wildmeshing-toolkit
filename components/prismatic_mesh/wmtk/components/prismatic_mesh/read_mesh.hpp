#pragma once

#include "prismatic_mesh.hpp"

namespace wmtk::components::prismatic_mesh::detail {
PrismaticMeshInput load_prismatic_vtu(const std::filesystem::path& path);
PrismaticMeshInput load_prismatic_msh(const std::filesystem::path& path);
} // namespace wmtk::components::prismatic_mesh::detail
