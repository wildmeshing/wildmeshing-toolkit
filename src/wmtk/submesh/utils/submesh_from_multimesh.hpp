#pragma once

#include <wmtk/submesh/Embedding.hpp>

namespace wmtk::submesh::utils {

std::shared_ptr<Embedding> submesh_from_multimesh(const std::shared_ptr<Mesh>& mesh);

} // namespace wmtk::submesh::utils
