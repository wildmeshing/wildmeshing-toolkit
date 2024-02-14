#pragma once

#include <wmtk/EdgeMesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include "SimplexCollection.hpp"

namespace wmtk::simplex {
/**
 * @brief Check if the edge to collapse satisfying the link condition.
 *
 * @param mesh Input EdgeMesh, TriMesh, TetMesh
 * @param edge The edge tuple to collapse
 * @return true if the edge to collapse satisfying the link condition.
 */
bool link_condition(const EdgeMesh& mesh, const Tuple& edge);
bool link_condition(const TriMesh& mesh, const Tuple& edge);
bool link_condition(const TetMesh& mesh, const Tuple& edge);
bool link_condition(const Mesh& mesh, const Tuple& edge);
} // namespace wmtk::simplex
