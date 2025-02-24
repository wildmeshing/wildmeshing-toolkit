#pragma once

#include <wmtk/submesh/Embedding.hpp>
#include <wmtk/submesh/SubMesh.hpp>

namespace wmtk::submesh::utils {

std::shared_ptr<Embedding> submesh_from_multimesh(const std::shared_ptr<Mesh>& mesh);

std::shared_ptr<Embedding> submesh_from_multimesh(
    const std::shared_ptr<Mesh>& mesh,
    std::map<std::shared_ptr<Mesh>, std::shared_ptr<SubMesh>>& submesh_map);

} // namespace wmtk::submesh::utils
