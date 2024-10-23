#pragma once

#include <wmtk/TriMesh.hpp>

namespace wmtk::components::shortest_edge_collapse {

struct ShortestEdgeCollapseOptions
{
    /**
     * vertex positions (double)
     */
    attribute::MeshAttributeHandle position_handle;
    /**
     * The desired edge length relative to the AABB.
     */
    double length_rel;
    /**
     * Are boundary vertices allowed to be collapsed?
     */
    bool lock_boundary = false;
    /**
     * The envelope size relative to the AABB.
     */
    std::optional<double> envelope_size;
    /**
     * If this attribute is specified, it is used to check for inversions. The mesh must be of top
     * dimension, e.g., a TriMesh in 2D or a TetMesh in 3D.
     */
    attribute::MeshAttributeHandle inversion_position_handle;
    /**
     * Any other attribute goes here. They are handled with the default attribute behavior.
     */
    std::vector<attribute::MeshAttributeHandle> pass_through_attributes;
};

} // namespace wmtk::components::shortest_edge_collapse