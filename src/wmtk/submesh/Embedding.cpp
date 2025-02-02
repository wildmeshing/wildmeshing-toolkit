#include "Embedding.hpp"

#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/primitive_range.hpp>

#include "SubMesh.hpp"

namespace wmtk::submesh {

Embedding::Embedding(const std::shared_ptr<Mesh>& mesh)
    : m_mesh(mesh)
{
    m_tag_attribute_name[PrimitiveType::Vertex] = "WMTK_submesh_tag_v";
    m_tag_attribute_name[PrimitiveType::Edge] = "WMTK_submesh_tag_e";
    m_tag_attribute_name[PrimitiveType::Triangle] = "WMTK_submesh_tag_f";
    m_tag_attribute_name[PrimitiveType::Tetrahedron] = "WMTK_submesh_tag_t";

    Mesh& m = *m_mesh;

    // register tag attributes
    for (const PrimitiveType& pt : utils::primitive_below(m.top_simplex_type())) {
        if (m.has_attribute<int64_t>(m_tag_attribute_name[pt], pt)) {
            log_and_throw_error(
                "Cannot create embedding. Mesh already has an attribute with name {}",
                m_tag_attribute_name[pt]);
        }

        m_tag_handle[pt] = m.register_attribute_typed<int64_t>(m_tag_attribute_name[pt], pt, 1);
    }
}

std::shared_ptr<SubMesh> Embedding::add_submesh()
{
    if (m_submesh_counter == 63) {
        log_and_throw_error("An embedding can only hold up to 63 submeshes");
    }

    std::shared_ptr<SubMesh> sub = std::make_shared<SubMesh>(*this, m_submesh_counter);
    m_submeshes.emplace_back(sub);

    ++m_submesh_counter;

    return sub;
}

Mesh& Embedding::mesh()
{
    return *m_mesh;
}

attribute::TypedAttributeHandle<int64_t>& Embedding::tag_handle(const PrimitiveType pt)
{
    return m_tag_handle.at(pt);
}

attribute::Accessor<int64_t> Embedding::tag_accessor(const PrimitiveType pt)
{
    return m_mesh->create_accessor(m_tag_handle.at(pt));
}


} // namespace wmtk::submesh
