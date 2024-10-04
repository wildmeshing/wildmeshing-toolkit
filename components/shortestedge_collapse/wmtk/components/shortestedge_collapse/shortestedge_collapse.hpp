#pragma once
#include <wmtk/Mesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/attribute/MeshAttributeHandle.hpp>

#include "ShortestEdgeCollapseOptions.hpp"

namespace wmtk::components {

/**
 * @brief Perform shortest-edge collapse on a triangular surface mesh.
 *
 * This function generates new attributes that are not removed automatically.
 *
 * @param mesh The triangular surface mesh.
 * @param options All options required for performing the shortest-edge collapse.
 */
void shortestedge_collapse(Mesh& mesh, const ShortestEdgeCollapseOptions& options);

/**
 * @brief Perform shortest-edge collapse on a triangular surface mesh.
 *
 * This function generates new attributes that are not removed automatically.
 *
 * This function wraps the behavior of `shortestedge_collapse` with options. For details on the
 * default values of the options, look at ShortestEdgeCollapseOptions.
 *
 */
void shortestedge_collapse(
    Mesh& mesh,
    const attribute::MeshAttributeHandle& position_handle,
    const double length_rel,
    std::optional<bool> lock_boundary = {},
    std::optional<double> envelope_size = {},
    std::optional<attribute::MeshAttributeHandle> inversion_position_handle = {},
    const std::vector<attribute::MeshAttributeHandle>& pass_through = {});

} // namespace wmtk::components
