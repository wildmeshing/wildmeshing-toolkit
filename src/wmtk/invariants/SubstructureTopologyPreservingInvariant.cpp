#include "SubstructureTopologyPreservingInvariant.hpp"
#include <wmtk/Mesh.hpp>

#include <wmtk/simplex/RawSimplexCollection.hpp>
#include <wmtk/simplex/cofaces_single_dimension.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>
#include <wmtk/simplex/link.hpp>
#include <wmtk/simplex/open_star.hpp>

namespace wmtk::invariants {
SubstructureTopologyPreservingInvariant::SubstructureTopologyPreservingInvariant(
    const Mesh& m,
    const MeshAttributeHandle<long>& substructure_face_tag_handle,
    const MeshAttributeHandle<long>& substructure_edge_tag_handle,
    const long substructure_tag_value)
    : MeshInvariant(m)
    , m_substructure_face_tag_handle(substructure_face_tag_handle)
    , m_substructure_edge_tag_handle(substructure_edge_tag_handle)
    , m_substructure_tag_value(substructure_tag_value)
{}

bool SubstructureTopologyPreservingInvariant::before(const Tuple& t) const
{
    // assuming t represents an edge
    switch (mesh().top_cell_dimension()) {
    case 2: return before_tri(t);
    case 3: return before_tet(t);
    default:
        throw std::runtime_error(
            "Unknown cell dimension in SubstructureTopologyPreservingInvariant.");
        break;
    }
    return false;
}

bool SubstructureTopologyPreservingInvariant::before_tri(const Tuple& t) const
{
    using namespace simplex;

    const auto edge_tag_acc = mesh().create_const_accessor(m_substructure_edge_tag_handle);

    // edge e = (u,v)

    Simplex edge_e(PrimitiveType::Edge, t);
    Simplex vertex_u(PrimitiveType::Vertex, t);
    Simplex vertex_v(PrimitiveType::Vertex, mesh().switch_vertex(t));

    RawSimplexCollection lk_u_0(link(mesh(), vertex_u));
    RawSimplexCollection lk_u_1;
    RawSimplexCollection lk_u_2;

    for (const Simplex& e_u :
         cofaces_single_dimension_simplices(mesh(), vertex_u, PrimitiveType::Edge)) {
        if (edge_tag_acc.const_scalar_attribute(e_u.tuple()) == m_substructure_tag_value) {
            std::vector<Tuple> vertices_dummy_tri =
                faces_single_dimension_tuples(mesh(), e_u, PrimitiveType::Vertex);
            vertices_dummy_tri.emplace_back(Tuple()); // add dummy vertex

            RawSimplex dummy_tri(mesh(), vertices_dummy_tri);
            RawSimplex opp_dummy_edge = dummy_tri.opposite_face(mesh(), vertex_u.tuple());
            lk_u_0.add(opp_dummy_edge);
            lk_u_0.add(opp_dummy_edge.faces());

            lk_u_1.add(opp_dummy_edge);
            lk_u_1.add(opp_dummy_edge.faces());

            RawSimplex raw_e_u(mesh(), e_u);
            lk_u_2.add(raw_e_u.opposite_face(mesh(), vertex_u.tuple()));
        }
    }

    // TODO if u is an order 3 vertex ...

    lk_u_0.sort_and_clean();
    lk_u_1.sort_and_clean();
    lk_u_2.sort_and_clean();

    // same code for vertex v

    RawSimplexCollection lk_v_0(link(mesh(), vertex_v));
    RawSimplexCollection lk_v_1;
    RawSimplexCollection lk_v_2;

    for (const Simplex& e_v :
         cofaces_single_dimension_simplices(mesh(), vertex_v, PrimitiveType::Edge)) {
        if (edge_tag_acc.const_scalar_attribute(e_v.tuple()) == m_substructure_tag_value) {
            std::vector<Tuple> vertices_dummy_tri =
                faces_single_dimension_tuples(mesh(), e_v, PrimitiveType::Vertex);
            vertices_dummy_tri.emplace_back(Tuple()); // add dummy vertex

            RawSimplex dummy_tri(mesh(), vertices_dummy_tri);
            RawSimplex opp_dummy_edge = dummy_tri.opposite_face(mesh(), vertex_v.tuple());
            lk_v_0.add(opp_dummy_edge);
            lk_v_0.add(opp_dummy_edge.faces());

            lk_v_1.add(opp_dummy_edge);
            lk_v_1.add(opp_dummy_edge.faces());

            RawSimplex raw_e_v(mesh(), e_v);
            lk_v_2.add(raw_e_v.opposite_face(mesh(), vertex_v.tuple()));
        }
    }

    // TODO if v is an order 3 vertex ...

    lk_v_0.sort_and_clean();
    lk_v_1.sort_and_clean();
    lk_v_2.sort_and_clean();

    if (!RawSimplexCollection::get_intersection(lk_u_2, lk_v_2).simplex_vector().empty()) {
        return false;
    }

    // lk_e
    RawSimplexCollection lk_e_0(link(mesh(), edge_e));
    RawSimplexCollection lk_e_1;

    RawSimplex raw_edge_e(mesh(), edge_e);

    if (edge_tag_acc.const_scalar_attribute(edge_e.tuple()) == m_substructure_tag_value) {
        RawSimplex dummy_vertex({-1});
        lk_e_0.add(dummy_vertex);
        lk_e_1.add(dummy_vertex);
    }

    lk_e_0.sort_and_clean();
    lk_e_1.sort_and_clean();

    RawSimplexCollection intersection_u_v_0 =
        RawSimplexCollection::get_intersection(lk_u_0, lk_v_0);
    if (!RawSimplexCollection::are_simplex_collections_equal(intersection_u_v_0, lk_e_0)) {
        return false;
    }
    RawSimplexCollection intersection_u_v_1 =
        RawSimplexCollection::get_intersection(lk_u_1, lk_v_1);
    if (!RawSimplexCollection::are_simplex_collections_equal(intersection_u_v_1, lk_e_1)) {
        return false;
    }

    return true;
}

bool SubstructureTopologyPreservingInvariant::before_tet(const Tuple& t) const
{
    using namespace simplex;

    const auto edge_tag_acc = mesh().create_const_accessor(m_substructure_edge_tag_handle);
    const auto face_tag_acc = mesh().create_const_accessor(m_substructure_face_tag_handle);

    // edge e = (u,v)

    Simplex edge_e(PrimitiveType::Edge, t);
    Simplex vertex_u(PrimitiveType::Vertex, t);
    Simplex vertex_v(PrimitiveType::Vertex, mesh().switch_vertex(t));

    RawSimplexCollection lk_u_0(link(mesh(), vertex_u));
    RawSimplexCollection lk_u_1;
    RawSimplexCollection lk_u_2;

    for (const Simplex& f_u :
         cofaces_single_dimension_simplices(mesh(), vertex_u, PrimitiveType::Face)) {
        if (face_tag_acc.const_scalar_attribute(f_u.tuple()) == m_substructure_tag_value) {
            std::vector<Tuple> vertices_dummy_tet =
                faces_single_dimension_tuples(mesh(), f_u, PrimitiveType::Vertex);
            vertices_dummy_tet.emplace_back(Tuple()); // add dummy vertex

            RawSimplex dummy_tet(mesh(), vertices_dummy_tet);

            // add face opposite of dummy_tet and all its  faces
            RawSimplex opp_dummy_face = dummy_tet.opposite_face(mesh(), vertex_u.tuple());
            lk_u_0.add(opp_dummy_face);
            lk_u_0.add(opp_dummy_face.faces());

            RawSimplex raw_f_u(mesh(), f_u);
            // add edge of f_u opposite of vertex u and its 2 vertices
            RawSimplex opp_dummy_edge = raw_f_u.opposite_face(mesh(), vertex_u.tuple());
            lk_u_1.add(opp_dummy_edge);
            lk_u_1.add(opp_dummy_edge.faces());
        }
    }

    for (const Simplex& e_u :
         cofaces_single_dimension_simplices(mesh(), vertex_u, PrimitiveType::Edge)) {
        if (edge_tag_acc.const_scalar_attribute(e_u.tuple()) == m_substructure_tag_value) {
            std::vector<Tuple> vertices_dummy_tri =
                faces_single_dimension_tuples(mesh(), e_u, PrimitiveType::Vertex);
            vertices_dummy_tri.emplace_back(Tuple()); // add dummy vertex

            RawSimplex dummy_tri(mesh(), vertices_dummy_tri);
            RawSimplex opp_dummy_edge = dummy_tri.opposite_face(mesh(), vertex_u.tuple());
            lk_u_0.add(opp_dummy_edge);
            lk_u_0.add(opp_dummy_edge.faces());

            lk_u_1.add(opp_dummy_edge);
            lk_u_1.add(opp_dummy_edge.faces());

            RawSimplex raw_e_u(mesh(), e_u);
            lk_u_2.add(raw_e_u.opposite_face(mesh(), vertex_u.tuple()));
        }
    }

    // TODO if u is an order 3 vertex ...

    lk_u_0.sort_and_clean();
    lk_u_1.sort_and_clean();
    lk_u_2.sort_and_clean();

    // same code for vertex v

    RawSimplexCollection lk_v_0(link(mesh(), vertex_v));
    RawSimplexCollection lk_v_1;
    RawSimplexCollection lk_v_2;

    for (const Simplex& f_v :
         cofaces_single_dimension_simplices(mesh(), vertex_v, PrimitiveType::Face)) {
        if (face_tag_acc.const_scalar_attribute(f_v.tuple()) == m_substructure_tag_value) {
            std::vector<Tuple> vertices_dummy_tet =
                faces_single_dimension_tuples(mesh(), f_v, PrimitiveType::Vertex);
            vertices_dummy_tet.emplace_back(Tuple()); // add dummy vertex

            RawSimplex dummy_tet(mesh(), vertices_dummy_tet);

            // add face opposite of dummy_tet and all its  faces
            RawSimplex opp_dummy_face = dummy_tet.opposite_face(mesh(), vertex_v.tuple());
            lk_v_0.add(opp_dummy_face);
            lk_v_0.add(opp_dummy_face.faces());

            RawSimplex raw_f_u(mesh(), f_v);
            // add edge of f_u opposite of vertex u and its 2 vertices
            RawSimplex opp_dummy_edge = raw_f_u.opposite_face(mesh(), vertex_v.tuple());
            lk_v_1.add(opp_dummy_edge);
            lk_v_1.add(opp_dummy_edge.faces());
        }
    }

    for (const Simplex& e_v :
         cofaces_single_dimension_simplices(mesh(), vertex_v, PrimitiveType::Edge)) {
        if (edge_tag_acc.const_scalar_attribute(e_v.tuple()) == m_substructure_tag_value) {
            std::vector<Tuple> vertices_dummy_tri =
                faces_single_dimension_tuples(mesh(), e_v, PrimitiveType::Vertex);
            vertices_dummy_tri.emplace_back(Tuple()); // add dummy vertex

            RawSimplex dummy_tri(mesh(), vertices_dummy_tri);
            RawSimplex opp_dummy_edge = dummy_tri.opposite_face(mesh(), vertex_v.tuple());
            lk_v_0.add(opp_dummy_edge);
            lk_v_0.add(opp_dummy_edge.faces());

            lk_v_1.add(opp_dummy_edge);
            lk_v_1.add(opp_dummy_edge.faces());

            RawSimplex raw_e_v(mesh(), e_v);
            lk_v_2.add(raw_e_v.opposite_face(mesh(), vertex_v.tuple()));
        }
    }

    // TODO if v is an order 3 vertex ...

    lk_v_0.sort_and_clean();
    lk_v_1.sort_and_clean();
    lk_v_2.sort_and_clean();

    if (!RawSimplexCollection::get_intersection(lk_u_2, lk_v_2).simplex_vector().empty()) {
        return false;
    }

    // lk_e
    RawSimplexCollection lk_e_0(link(mesh(), edge_e));
    RawSimplexCollection lk_e_1;

    RawSimplex raw_edge_e(mesh(), edge_e);

    for (const Simplex& f_e :
         cofaces_single_dimension_simplices(mesh(), edge_e, PrimitiveType::Face)) {
        if (face_tag_acc.const_scalar_attribute(f_e.tuple()) == m_substructure_tag_value) {
            std::vector<Tuple> vertices_dummy_tet =
                faces_single_dimension_tuples(mesh(), f_e, PrimitiveType::Vertex);
            vertices_dummy_tet.emplace_back(Tuple()); // add dummy vertex

            RawSimplex raw_f_e(mesh(), f_e);

            RawSimplex dummy_tet(mesh(), vertices_dummy_tet);
            RawSimplex dummy_edge = dummy_tet.opposite_face(raw_f_e);

            lk_e_1.add(dummy_edge);
            lk_e_1.add(dummy_edge.faces());
            lk_e_1.add(raw_f_e.opposite_face(raw_edge_e));
        }
    }

    if (edge_tag_acc.const_scalar_attribute(edge_e.tuple()) == m_substructure_tag_value) {
        RawSimplex dummy_vertex({-1});
        lk_e_0.add(dummy_vertex);
        lk_e_1.add(dummy_vertex);
    }

    RawSimplexCollection intersection_u_v_0 =
        RawSimplexCollection::get_intersection(lk_u_0, lk_v_0);
    if (!RawSimplexCollection::are_simplex_collections_equal(intersection_u_v_0, lk_e_0)) {
        return false;
    }
    RawSimplexCollection intersection_u_v_1 =
        RawSimplexCollection::get_intersection(lk_u_1, lk_v_1);
    if (!RawSimplexCollection::are_simplex_collections_equal(intersection_u_v_1, lk_e_1)) {
        return false;
    }

    return true;
}

} // namespace wmtk::invariants