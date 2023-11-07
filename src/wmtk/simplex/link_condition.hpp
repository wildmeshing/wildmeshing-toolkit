#pragma once

#include <wmtk/EdgeMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include "SimplexCollection.hpp"

namespace wmtk::simplex {
bool link_condition(const EdgeMesh& mesh, const Tuple edge);
bool link_condition(const TriMesh& mesh, const Tuple edge);
} // namespace wmtk::simplex