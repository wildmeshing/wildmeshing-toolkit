#pragma once
#include <wmtk/Mesh.hpp>
#include <wmtk/TriMesh.hpp>

#include "LongestEdgeSplitOptions.hpp"

namespace wmtk::components::longest_edge_split {

/**
 * @brief Perform longes-edge split on a mesh.
 *
 * This function generates new attributes that are not removed automatically.
 *
 * @param mesh The root mesh.
 * @param options All options required for performing the longes-edge split.
 */
void longest_edge_split(Mesh& mesh, const LongestEdgeSplitOptions& options);

/**
 * @brief Perform longes-edge split on a mesh.
 *
 * This function generates new attributes that are not removed automatically.
 *
 * This function wraps the behavior of `longest_edge_split` with options. For details on the
 * default values of the options, look at LongestEdgeSplitOptions.
 *
 */
void longest_edge_split(
    Mesh& mesh,
    const attribute::MeshAttributeHandle& position_handle,
    const double length_rel,
    std::optional<bool> lock_boundary = {},
    const std::vector<attribute::MeshAttributeHandle>& pass_through = {});

} // namespace wmtk::components::longest_edge_split
