#pragma once

#include <wmtk/Mesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>

#include <wmtk/utils/Logger.hpp>

namespace wmtk::components::internal {


std::shared_ptr<wmtk::TetMesh> CDT_internal(
    const wmtk::TriMesh& m,
    std::vector<std::array<bool, 4>>& local_f_on_input);

} // namespace wmtk::components::internal
