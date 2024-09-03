#pragma once

#include <wmtk/Mesh.hpp>


namespace wmtk::components {

/**
 * @brief Generate a multi-mesh from a mesh with a tag that represents the substructure, the mesh is
 * changed
 */
void multimesh_from_tag(
    std::shared_ptr<Mesh>& mesh_in,
    attribute::MeshAttributeHandle& substructure_label,
    int64_t substructure_value);

} // namespace wmtk::components
