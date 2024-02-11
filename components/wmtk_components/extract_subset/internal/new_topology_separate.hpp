#pragma once

#include <functional>
#include <numeric>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>
#include <wmtk/simplex/top_dimension_cofaces.hpp>
#include <wmtk/utils/mesh_utils.hpp>
#include "utils.hpp"

namespace wmtk::components::internal {

std::unique_ptr<wmtk::Mesh> topology_separate(wmtk::Mesh& m, bool pos);
} // namespace wmtk::components::internal