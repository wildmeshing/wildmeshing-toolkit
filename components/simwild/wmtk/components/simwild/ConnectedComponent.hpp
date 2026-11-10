#pragma once

#include <set>
#include <utility>
#include <vector>
#include <wmtk/Types.hpp>

namespace wmtk::components::simwild {

/**
 * An intersection of tags represented as a set. If CellTag is an empty set, a triangle/tet is part of the ambient (background) mesh.
 */
using CellTag = std::set<int64_t>;

/**
 * A cell's own labels of its corners, for tight_seal_geom's "split_fill" mode: (vertex, region)
 * pairs, at most one per corner. A corner that a split made has none.
 */
using CornerLabels = std::vector<std::pair<size_t, size_t>>;

/**
 * @brief A collection of triangles (tets in 3D) that represent a connected component.
 *
 * @param faces Vector of face IDs in the connected component.
 * @param volume Total volume (area in 2D) of the connected component.
 * @param touches_boundary Whether the connected component touches the boundary of the mesh.
 */
struct ConnectedComponent
{
    std::vector<size_t> cells;
    double volume = 0.0;
    bool touches_boundary = false;
};

} // namespace wmtk::components::simwild