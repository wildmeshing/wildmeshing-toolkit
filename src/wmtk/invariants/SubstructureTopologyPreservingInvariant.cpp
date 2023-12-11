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
    , m_face_tag_handle(substructure_face_tag_handle)
    , m_edge_tag_handle(substructure_edge_tag_handle)
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

    // edge e = (u,v)

    Simplex edge_e(PrimitiveType::Edge, t);
    Simplex vertex_u(PrimitiveType::Vertex, t);
    Simplex vertex_v(PrimitiveType::Vertex, mesh().switch_vertex(t));

    SimplexCollection link_of_u = link(mesh(), vertex_u);
    // for (const Simplex& f_u :
    //      cofaces_single_dimension_simplices(mesh(), vertex_u, PrimitiveType::Face)) {
    //    //
    //}

    SimplexCollection link_of_v = link(mesh(), vertex_v);

    throw std::runtime_error("not implemented");
    return false;
}

bool SubstructureTopologyPreservingInvariant::before_tet(const Tuple& t) const
{
    using namespace simplex;

    const auto edge_tag_acc = mesh().create_const_accessor(m_edge_tag_handle);
    const auto face_tag_acc = mesh().create_const_accessor(m_face_tag_handle);

    // edge e = (u,v)

    Simplex edge_e(PrimitiveType::Edge, t);
    Simplex vertex_u(PrimitiveType::Vertex, t);
    Simplex vertex_v(PrimitiveType::Vertex, mesh().switch_vertex(t));

    RawSimplexCollection lk_u_0(link(mesh(), vertex_u));
    RawSimplexCollection lk_u_1;
    RawSimplexCollection lk_u_2;

    for (const Simplex& f_u :
         cofaces_single_dimension_simplices(mesh(), vertex_u, PrimitiveType::Face)) {
        if (face_tag_acc.const_scalar_attribute(f_u.tuple()) == m_tag_value) {
            std::vector<Tuple> vertices_f_u =
                faces_single_dimension_tuples(mesh(), f_u, PrimitiveType::Vertex);
            vertices_f_u.emplace_back(Tuple()); // dummy vertex

            RawSimplex dummy_tet(mesh(), vertices_f_u);

            lk_u_0.add(mesh(), PrimitiveType::Vertex, vertices_f_u);

            // add face opposite of dummy_tet and all its  faces
        }
    }
    lk_u_0.sort_and_clean();

    SimplexCollection link_of_v = link(mesh(), vertex_v);

    throw std::runtime_error("not implemented");
    return false;
}

} // namespace wmtk::invariants