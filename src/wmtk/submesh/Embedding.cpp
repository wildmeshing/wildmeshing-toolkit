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

const Mesh& Embedding::mesh() const
{
    return *m_mesh;
}

attribute::TypedAttributeHandle<int64_t>& Embedding::tag_handle(const PrimitiveType pt)
{
    return m_tag_handle.at(pt);
}

attribute::Accessor<int64_t> Embedding::tag_accessor(const PrimitiveType pt)
{
    return mesh().create_accessor(m_tag_handle.at(pt));
}

std::vector<Tuple> Embedding::get_all(PrimitiveType type) const
{
    return mesh().get_all(type);
}

std::vector<simplex::IdSimplex> Embedding::get_all_id_simplex(PrimitiveType type) const
{
    return mesh().get_all_id_simplex(type);
}

int64_t Embedding::top_cell_dimension() const
{
    return mesh().top_cell_dimension();
}

Tuple Embedding::switch_tuple(const Tuple& tuple, PrimitiveType type) const
{
    return mesh().switch_tuple(tuple, type);
}

bool Embedding::is_boundary(PrimitiveType pt, const Tuple& tuple) const
{
    return mesh().is_boundary(pt, tuple);
}

int64_t Embedding::id(const simplex::Simplex& s) const
{
    return mesh().id(s);
}

int64_t Embedding::id(const Tuple& tuple, PrimitiveType pt) const
{
    return mesh().id(tuple, pt);
}


} // namespace wmtk::submesh
