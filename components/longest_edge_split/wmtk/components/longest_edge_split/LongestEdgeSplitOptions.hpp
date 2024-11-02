#pragma once

#include <wmtk/Mesh.hpp>

namespace wmtk::components::longest_edge_split {

struct LongestEdgeSplitOptions
{
    /**
     * vertex positions (double)
     */
    attribute::MeshAttributeHandle position_handle;
    /**
     * If this mesh is part of a multimesh, specify the vertex positions of all other meshes here,
     * if they have any.
     */
    std::vector<attribute::MeshAttributeHandle> other_position_handles;
    /**
     * The desired edge length relative to the AABB.
     */
    double length_rel;

    /**
     * Any other attribute goes here. They are handled with the default attribute behavior.
     */
    std::vector<attribute::MeshAttributeHandle> pass_through_attributes;
};

} // namespace wmtk::components::longest_edge_split