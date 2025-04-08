#pragma once
#include <nlohmann/json.hpp>
#include <wmtk/Mesh.hpp>

#include "SimplicialEmbeddingOptions.hpp"

namespace wmtk::components::simplicial_embedding {

void simplicial_embedding(Mesh& mesh, const SimplicialEmbeddingOptions& options);

void simplicial_embedding(const nlohmann::json& j);

} // namespace wmtk::components::simplicial_embedding
