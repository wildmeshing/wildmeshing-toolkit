#pragma once

#include <filesystem>
#include <map>
#include <paraviewo/ParaviewWriter.hpp>
#include <string>
#include <vector>
#include <wmtk/Types.hpp>

namespace wmtk::components::prismatic_mesh::detail {

using MeshFields = std::map<std::string, MatrixXd>;

// Gmsh 4.1 ASCII volume mesh. Connectivity uses compact zero-based output rows.
void write_msh(
    const std::filesystem::path& path,
    const MatrixXd& vertices,
    const std::vector<paraviewo::CellElement>& cells,
    const MeshFields& point_fields,
    const MeshFields& cell_fields);

} // namespace wmtk::components::prismatic_mesh::detail
