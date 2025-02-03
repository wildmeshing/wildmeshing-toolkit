#include "SubMesh.hpp"

#include "Embedding.hpp"

#include <wmtk/simplex/IdSimplex.hpp>
#include <wmtk/simplex/faces.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/primitive_range.hpp>

namespace wmtk::submesh {

SubMesh::SubMesh(Embedding& embedding, int64_t submesh_id)
    : m_embedding(embedding)
    , m_submesh_id(submesh_id)
{}

Mesh& SubMesh::mesh()
{
    return m_embedding.mesh();
}

const Mesh& SubMesh::mesh() const
{
    return m_embedding.mesh();
}

void SubMesh::add_simplex(const Tuple& tuple, PrimitiveType pt)
{
    auto acc = m_embedding.tag_accessor(pt);
    acc.scalar_attribute(tuple) |= (int64_t)1 << m_submesh_id;

    const simplex::Simplex s(mesh(), pt, tuple);

    auto faces = simplex::faces(mesh(), s);

    for (const simplex::Simplex& f : faces) {
        auto a = m_embedding.tag_accessor(f.primitive_type());
        a.scalar_attribute(f.tuple()) |= (int64_t)1 << m_submesh_id;
    }
}

void SubMesh::add_simplex(const simplex::IdSimplex& simplex)
{
    const PrimitiveType& pt = simplex.primitive_type();
    const Tuple t = mesh().get_tuple_from_id_simplex(simplex);
    add_simplex(t, pt);
}

PrimitiveType SubMesh::top_simplex_type(const Tuple& tuple) const
{
    const Mesh& m = mesh();

    for (const PrimitiveType& pt : utils::primitive_below(m.top_simplex_type())) {
        if (contains(tuple, pt)) {
            return pt;
        }
    }

    log_and_throw_error("No simplex of the tuple contains the submesh tag.");
}

PrimitiveType SubMesh::top_simplex_type() const
{
    const Mesh& m = mesh();

    for (const PrimitiveType& pt : utils::primitive_below(m.top_simplex_type())) {
        const auto tuples = m.get_all(pt);
        for (const Tuple& t : tuples) {
            if (contains(t, pt)) {
                return pt;
            }
        }
    }

    log_and_throw_error("No simplex of the tuple contains the submesh tag.");
}

Tuple SubMesh::switch_tuple(const Tuple& tuple, PrimitiveType pt) const
{
    const int8_t pt_id = get_primitive_type_id(pt);
    const int8_t max_pt_id = get_primitive_type_id(top_simplex_type(tuple));

    if (pt_id >= max_pt_id) {
        log_and_throw_error("Submesh `switch_tuple` cannot be used for cell switches.");
    }

    return mesh().switch_tuple(tuple, pt);
}

bool SubMesh::contains(const Tuple& tuple, PrimitiveType pt) const
{
    const auto acc = m_embedding.tag_accessor(pt);
    return (acc.const_scalar_attribute(tuple) & ((int64_t)1 << m_submesh_id)) != 0;
}

} // namespace wmtk::submesh
