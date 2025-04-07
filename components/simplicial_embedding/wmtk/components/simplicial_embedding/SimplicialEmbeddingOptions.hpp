#pragma once

#include <wmtk/Mesh.hpp>

namespace wmtk::components::simplicial_embedding {

struct SimplicialEmbeddingOptions
{
    /**
     * All simplex dimensions must have an int64_t scalar attribute where the tags are stored.
     */
    std::map<PrimitiveType, attribute::MeshAttributeHandle> tag_attributes;
    /**
     * The value that should be simplicially embedded.
     */
    int64_t value;
    /**
     * Other attributes that should be processed with the default behavior.
     */
    std::vector<attribute::MeshAttributeHandle> pass_through_attributes;
    /**
     * If false, this component forms a simplicial complex out of the tags, i.e., all faces of
     * tagged simplices are also tagged.
     */
    bool generate_simplicial_embedding = true;
};

} // namespace wmtk::components::simplicial_embedding