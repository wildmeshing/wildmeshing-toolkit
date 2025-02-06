#include "SubMesh.hpp"

#include "Embedding.hpp"

#include <wmtk/simplex/IdSimplex.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/simplex/cofaces_single_dimension_iterable.hpp>
#include <wmtk/simplex/faces.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>
#include <wmtk/simplex/neighbors_single_dimension.hpp>
#include <wmtk/simplex/open_star_iterable.hpp>
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
    const int64_t pt_dim = get_primitive_type_id(pt);
    if (m_top_cell_dimension < pt_dim) {
        logger().trace("Top cell dimension changed from {} to {}", m_top_cell_dimension, pt_dim);
        m_top_cell_dimension = pt_dim;
    }

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

std::vector<Tuple> SubMesh::get_all(const PrimitiveType pt) const
{
    const auto all = mesh().get_all(pt);
    std::vector<Tuple> sub;
    for (const Tuple& t : all) {
        if (contains(t, pt)) {
            sub.emplace_back(t);
        }
    }
    return sub;
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
    // const Mesh& m = mesh();
    //
    // for (const PrimitiveType& pt : utils::primitive_below(m.top_simplex_type())) {
    //     const auto tuples = m.get_all(pt);
    //     for (const Tuple& t : tuples) {
    //         if (contains(t, pt)) {
    //             return pt;
    //         }
    //     }
    // }
    //
    // log_and_throw_error("No simplex of the tuple contains the submesh tag.");

    if (m_top_cell_dimension < 0) {
        log_and_throw_error("No simplex of the tuple contains the submesh tag.");
    }

    return get_primitive_type_from_id(m_top_cell_dimension);
}

int64_t SubMesh::top_cell_dimension() const
{
    assert(m_top_cell_dimension >= 0);
    assert(m_top_cell_dimension < 4);
    return m_top_cell_dimension;
}

Tuple SubMesh::switch_tuple(const Tuple& tuple, PrimitiveType pt) const
{
    const int8_t pt_id = get_primitive_type_id(pt);
    const int8_t max_pt_id = get_primitive_type_id(top_simplex_type(tuple));

    if (pt_id > max_pt_id) {
        // invalid switch
        log_and_throw_error("Required PrimitiveType switch does not exist in submesh.");
    }
    if (pt_id == max_pt_id) {
        // global switch
        const PrimitiveType pt_face = get_primitive_type_from_id(pt_id - 1);
        const simplex::Simplex s_face(mesh(), pt_face, tuple);

        Tuple other;
        int64_t other_counter = 0;
        for (const Tuple& t : simplex::cofaces_single_dimension_iterable(mesh(), s_face, pt)) {
            if (contains(t, pt)) {
                ++other_counter;
                if (other_counter == 2) {
                    other = t;
                }
            }
        }

        if (other_counter != 2) {
            log_and_throw_error(
                "SubMesh `switch_tuple` cannot be used on non-manifold or boundary simplices.");
        }
        return other;
    }

    // local switch
    return local_switch_tuple(tuple, pt);
}

// std::vector<Tuple> SubMesh::switch_tuple_vector(const Tuple& tuple, PrimitiveType pt) const
//{
//     const int8_t pt_id = get_primitive_type_id(pt);
//     const int8_t max_pt_id = get_primitive_type_id(top_simplex_type(tuple));
//
//     if (pt_id > max_pt_id) {
//         log_and_throw_error("Required PrimitiveType switch does not exist in submesh.");
//     }
//     if (pt_id < max_pt_id) {
//        // log_and_throw_error("Cannot perform global switches for vertex PrimitiveType. Use "
//        //                     "`switch_tuple` instead of `switch_tuple_vector`.");
//        return {local_switch_tuple(tuple, pt)};
//    }
//
//    assert(pt_id <= max_pt_id);
//
//    const PrimitiveType pt_face = get_primitive_type_from_id(pt_id - 1);
//
//    const simplex::Simplex s_face(mesh(), pt_face, tuple);
//
//    std::vector<Tuple> neighs;
//    neighs.reserve(2);
//    for (const Tuple& t : simplex::cofaces_single_dimension_iterable(mesh(), s_face, pt)) {
//        if (contains(t, pt)) {
//            neighs.emplace_back(t);
//        }
//    }
//
//    assert(!neighs.empty());
//    assert(neighs[0] == tuple);
//
//    return neighs;
//}

bool SubMesh::is_boundary(PrimitiveType pt, const Tuple& tuple) const
{
    if (!contains(tuple, pt)) {
        log_and_throw_error("Cannot check for boundary if simplex is not contained in submesh");
    }

    const simplex::Simplex s(pt, tuple);

    const PrimitiveType top_pt = top_simplex_type();
    if (top_pt == PrimitiveType::Vertex) {
        return true;
    }
    const PrimitiveType face_pt = get_primitive_type_from_id(top_cell_dimension() - 1);

    const auto neighbors = simplex::neighbors_single_dimension(mesh(), s, face_pt);

    // check if any incident facet has less than two neighbors
    int64_t n_neighbors = 0;
    for (const simplex::Simplex& face_simplex : neighbors) {
        if (!contains(face_simplex)) {
            continue;
        }
        ++n_neighbors;

        int64_t cell_counter = 0;
        for (const Tuple& cell_tuple :
             simplex::cofaces_single_dimension_iterable(mesh(), face_simplex, top_pt)) {
            if (contains(cell_tuple, top_pt)) {
                ++cell_counter;
            }
        }
        if (cell_counter < 2) {
            return true;
        }
    }

    if (n_neighbors == 0) {
        // this simplex has no cell incident and is therefore considered boundary
        return true;
    }

    return false;
}

bool SubMesh::is_boundary(const Tuple& tuple, PrimitiveType pt) const
{
    return is_boundary(pt, tuple);
}

bool SubMesh::contains(const Tuple& tuple, PrimitiveType pt) const
{
    const auto acc = m_embedding.tag_accessor(pt);
    return (acc.const_scalar_attribute(tuple) & ((int64_t)1 << m_submesh_id)) != 0;
}

bool SubMesh::contains(const simplex::IdSimplex& s) const
{
    const auto acc = m_embedding.tag_accessor(s.primitive_type());
    return (acc.const_scalar_attribute(s) & ((int64_t)1 << m_submesh_id)) != 0;
}

bool SubMesh::contains(const simplex::Simplex& s) const
{
    return contains(s.tuple(), s.primitive_type());
}

int64_t SubMesh::id(const Tuple& tuple, PrimitiveType pt) const
{
    return mesh().id(tuple, pt);
}

int64_t SubMesh::id(const simplex::Simplex& s) const
{
    return id(s.tuple(), s.primitive_type());
}

Tuple SubMesh::local_switch_tuple(const Tuple& tuple, PrimitiveType pt) const
{
    return mesh().switch_tuple(tuple, pt);
}

template <typename T>
void SubMesh::add_from_tag_attribute(
    const attribute::TypedAttributeHandle<T>& tag_attribute,
    const T tag_value)
{
    const attribute::Accessor<T> acc = mesh().create_const_accessor(tag_attribute);
    const PrimitiveType pt = tag_attribute.primitive_type();

    const auto tuples = mesh().get_all(pt);
    for (const Tuple& t : tuples) {
        if (acc.const_scalar_attribute(t) == tag_value) {
            add_simplex(t, pt);
        }
    }
}

template void SubMesh::add_from_tag_attribute(
    const attribute::TypedAttributeHandle<int64_t>&,
    const int64_t);
template void SubMesh::add_from_tag_attribute(
    const attribute::TypedAttributeHandle<char>&,
    const char);

} // namespace wmtk::submesh
