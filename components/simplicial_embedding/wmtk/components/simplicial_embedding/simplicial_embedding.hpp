#pragma once
#include <wmtk/Mesh.hpp>

#include "SimplicialEmbeddingOptions.hpp"

namespace wmtk::components::simplicial_embedding {

void simplicial_embedding(Mesh& mesh, const SimplicialEmbeddingOptions& options);

} // namespace wmtk::components::simplicial_embedding
