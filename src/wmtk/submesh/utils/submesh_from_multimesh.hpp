#pragma once

#include <wmtk/submesh/Embedding.hpp>
#include <wmtk/submesh/SubMesh.hpp>

namespace wmtk::submesh::utils {

/**
 * @brief Make the mesh an Embedding and construct SubMeshes from all children.
 *
 * The method does not delete any child meshes.
 *
 * @param mesh The root mesh that becomes the embedding mesh.
 * @return Shared pointer of the Embedding.
 */
std::shared_ptr<Embedding> submesh_from_multimesh(const std::shared_ptr<Mesh>& mesh);

/**
 * @brief Make the mesh an Embedding and construct SubMeshes from all children.
 *
 * The method does not delete any child meshes.
 *
 * @param mesh The root mesh that becomes the embedding mesh.
 * @param submesh_map A map from child meshes to the new SubMeshes.
 * @return Shared pointer of the Embedding.
 */
std::shared_ptr<Embedding> submesh_from_multimesh(
    const std::shared_ptr<Mesh>& mesh,
    std::map<std::shared_ptr<Mesh>, std::shared_ptr<SubMesh>>& submesh_map);

} // namespace wmtk::submesh::utils
