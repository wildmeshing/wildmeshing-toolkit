#pragma once

#include <functional>
#include <numeric>
#include <wmtk/TriMesh.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>
#include <wmtk/utils/mesh_utils.hpp>
#include "utils.hpp"

namespace wmtk::components::internal {

wmtk::TriMesh topology_separate_2d(wmtk::TriMesh m);
} // namespace wmtk::components::internal