#pragma once

#include <set>
#include <wmtk/Types.hpp>

namespace wmtk::components::image_simulation {

/**
 * An intersection of tags represented as a set. If CellTag is an empty set, a triangle/tet is part of the ambient (background) mesh.
 */
using CellTag = std::set<int64_t>;

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

} // namespace wmtk::components::image_simulation