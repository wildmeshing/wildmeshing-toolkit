#include "simplicial_embedding.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/components/utils/get_attributes.hpp>
#include <wmtk/io/Cache.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/primitive_range.hpp>

#include "internal/SimplicialEmbedding.hpp"

namespace wmtk::components {

void simplicial_embedding(Mesh& mesh, const SimplicialEmbeddingOptions& options)
{
    using namespace internal;

    std::vector<attribute::MeshAttributeHandle> tag_attr_vec;
    for (const PrimitiveType& ptype : wmtk::utils::primitive_below(mesh.top_simplex_type())) {
        tag_attr_vec.emplace_back(options.tag_attributes.at(ptype));
    }

    SimplicialEmbedding rs(mesh, tag_attr_vec, options.value, options.pass_through_attributes);
    rs.regularize_tags(options.generate_simplicial_embedding);

    // clean up attributes
    {
        std::vector<attribute::MeshAttributeHandle> keeps = options.pass_through_attributes;
        keeps.insert(keeps.end(), tag_attr_vec.begin(), tag_attr_vec.end());
        mesh.clear_attributes(keeps);
    }

    mesh.consolidate();
}

} // namespace wmtk::components