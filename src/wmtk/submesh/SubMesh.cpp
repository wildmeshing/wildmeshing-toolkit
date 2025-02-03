#include "SubMesh.hpp"

#include "Embedding.hpp"

#include <wmtk/simplex/IdSimplex.hpp>
#include <wmtk/simplex/cofaces_single_dimension_iterable.hpp>
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

    return local_switch_tuple(tuple, pt);
}

std::vector<Tuple> SubMesh::switch_tuple_vector(const Tuple& tuple, PrimitiveType pt) const
{
    const int8_t pt_id = get_primitive_type_id(pt);
    const int8_t max_pt_id = get_primitive_type_id(top_simplex_type(tuple));

    if (pt_id > max_pt_id) {
        log_and_throw_error("Required PrimitiveType switch does not exist in submesh.");
    }
    if (pt_id == 0) {
        log_and_throw_error("Cannot perform global switches for vertex PrimitiveType. Use "
                            "`switch_tuple` instead of `switch_tuple_vector`.");
    }

    assert(pt_id <= max_pt_id);

    const PrimitiveType pt_face = get_primitive_type_from_id(pt_id - 1);

    const simplex::Simplex s_face(mesh(), pt_face, tuple);

    std::vector<Tuple> neighs;
    neighs.reserve(2);
    for (const Tuple& t : simplex::cofaces_single_dimension_iterable(mesh(), s_face, pt)) {
        if (contains(t, pt)) {
            neighs.emplace_back(t);
        }
    }

    assert(!neighs.empty());
    assert(neighs[0] == tuple);

    return neighs;
}

bool SubMesh::contains(const Tuple& tuple, PrimitiveType pt) const
{
    const auto acc = m_embedding.tag_accessor(pt);
    return (acc.const_scalar_attribute(tuple) & ((int64_t)1 << m_submesh_id)) != 0;
}

Tuple SubMesh::local_switch_tuple(const Tuple& tuple, PrimitiveType pt) const
{
    return mesh().switch_tuple(tuple, pt);
}

} // namespace wmtk::submesh
