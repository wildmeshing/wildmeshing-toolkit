#pragma once

#include <functional>
#include <numeric>
#include <wmtk/TetMesh.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>
#include <wmtk/utils/mesh_utils.hpp>
#include "utils.hpp"

namespace wmtk::components::internal {
wmtk::TetMesh topology_separate_3d_old(wmtk::TetMesh m);
} // namespace wmtk::components::internal