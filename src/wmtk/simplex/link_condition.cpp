#include "link_condition.hpp"
#include <wmtk/utils/metaprogramming/as_mesh_variant.hpp>
#include "IdSimplexCollection.hpp"
#include "cofaces_single_dimension.hpp"
#include "cofaces_single_dimension_iterable.hpp"
#include "link.hpp"
#include "link_iterable.hpp"
#include "open_star.hpp"
#include "utils/SimplexComparisons.hpp"

namespace wmtk::simplex {
bool link_condition_closed_trimesh(const TriMesh& mesh, const Tuple& edge)
{
    const Simplex v_a = Simplex::vertex(mesh, edge);
    const Simplex v_b = Simplex::vertex(mesh, mesh.switch_tuple(edge, PrimitiveType::Vertex));
    const Simplex e_ab = Simplex::edge(mesh, edge);

    IdSimplexCollection link_a(mesh);
    link_a.reserve(24);
    for (const IdSimplex& s : link_iterable(mesh, v_a)) {
        link_a.add(s);
    }
    link_a.sort_and_clean();
    IdSimplexCollection link_b(mesh);
    link_b.reserve(24);
    for (const IdSimplex& s : link_iterable(mesh, v_b)) {
        link_b.add(s);
    }
    link_b.sort_and_clean();
    IdSimplexCollection link_ab(mesh);
    link_ab.reserve(2);
    for (const IdSimplex& s : link_iterable(mesh, e_ab)) {
        link_ab.add(s);
    }
    link_ab.sort_and_clean();

    const IdSimplexCollection link_a_link_b_intersection =
        IdSimplexCollection::get_intersection(link_a, link_b);

    return IdSimplexCollection::are_simplex_collections_equal(link_a_link_b_intersection, link_ab);
}

bool link_condition(const EdgeMesh& mesh, const Tuple& edge)
{
    const Tuple edge_switch_v = mesh.switch_tuple(edge, PrimitiveType::Vertex);
    if (mesh.is_boundary_vertex(edge) && mesh.is_boundary_vertex(edge_switch_v)) {
        return false;
    }
    if (utils::SimplexComparisons::equal(
            mesh,
            edge,
            PrimitiveType::Vertex,
            edge_switch_v,
            PrimitiveType::Vertex)) {
        return false;
    }
    return true;
}

bool link_condition(const TriMesh& mesh, const Tuple& edge)
{
    // step1 check link condition for closed case
    if (!link_condition_closed_trimesh(mesh, edge)) {
        return false;
    }
    // for the trimesh with boudanry cases, we add a dummy vertex w to the mesh, and connected it to
    // all the boundary edges, then the mesh becomes a closed manifold mesh for all vertices but w.

    // check if dummy vertex w is included in the lhs
    auto get_boundary_edges = [&mesh](const Tuple& _v) {
        const Simplex input_v(mesh, PrimitiveType::Vertex, _v);
        std::vector<Tuple> ret;
        // get incident_edges from open_star
        // incident_edges = cofaces_single_dimension_tuples(mesh, input_v,
        // PrimitiveType::Edge);
        auto incident_edges = cofaces_single_dimension_iterable(mesh, input_v, PrimitiveType::Edge);
        for (const Tuple& _e : incident_edges) {
            if (mesh.is_boundary(PrimitiveType::Edge, _e)) {
                ret.push_back(mesh.switch_tuple(_e, PrimitiveType::Vertex));
            }
        }
        return ret;
    };

    // case 1: edge ab is a boundary edge, in this case dummy vertex w is in lnk(ab), need to check
    // if there are any common edges connected with w in lnk_w^0(a)∩lnk_w^0(b)
    const auto boundary_neighbors_a = get_boundary_edges(edge);
    const auto boundary_neighbors_b =
        get_boundary_edges(mesh.switch_tuple(edge, PrimitiveType::Vertex));
    if (mesh.is_boundary_edge(edge)) {
        assert(boundary_neighbors_a.size() == 2); // if guarantee 2-manifold
        assert(boundary_neighbors_b.size() == 2); // if guarantee 2-manifold
        for (auto e_a : boundary_neighbors_a) {
            for (auto e_b : boundary_neighbors_b) {
                if (utils::SimplexComparisons::equal(
                        mesh,
                        Simplex(mesh, PrimitiveType::Vertex, e_a),
                        Simplex(mesh, PrimitiveType::Vertex, e_b))) {
                    // find common edge, link condition fails
                    return false;
                }
            }
        }
    } else {
        if (boundary_neighbors_a.size() == 0 || boundary_neighbors_b.size() == 0) {
            // in this case, lnk_w^0(a) ∩ lnk_w^0(b) == lnk(a) ∩ lnk(b) == lnk(ab) == lnk_w^0(ab)
            return true;
        } else {
            // in this case w \in lhs but not \in rhs
            return false;
        }
    }

    return true;
}

bool link_condition_closed_tetmesh(const TetMesh& mesh, const Tuple& edge)
{
    // for closed mesh, if link(a) \intersect link(b) == link(ab)
    const Simplex v_a = Simplex::vertex(mesh, edge);
    const Simplex v_b = Simplex::vertex(mesh, mesh.switch_vertex(edge));
    const Simplex e_ab = Simplex::edge(mesh, edge);

    IdSimplexCollection link_a(mesh);
    link_a.reserve(128);
    for (const IdSimplex& s : link_iterable(mesh, v_a)) {
        link_a.add(s);
    }
    link_a.sort_and_clean();
    IdSimplexCollection link_b(mesh);
    link_b.reserve(128);
    for (const IdSimplex& s : link_iterable(mesh, v_b)) {
        link_b.add(s);
    }
    link_b.sort_and_clean();
    IdSimplexCollection link_ab(mesh);
    link_ab.reserve(32);
    for (const IdSimplex& s : link_iterable(mesh, e_ab)) {
        link_ab.add(s);
    }
    link_ab.sort_and_clean();

    const IdSimplexCollection link_a_link_b_intersection =
        IdSimplexCollection::get_intersection(link_a, link_b);

    return IdSimplexCollection::are_simplex_collections_equal(link_a_link_b_intersection, link_ab);
}

bool link_condition(const TetMesh& mesh, const Tuple& edge)
{
    // close mesh check
    if (!link_condition_closed_tetmesh(mesh, edge)) {
        return false;
    }

    /**
     * The link condition on the boundary can be reduced to two cases.
     * 1. edge is boundary: Perform the link condition on the boundary triangles.
     * 2. edge is interior: Check if vertices are boundary. If both are, the link condition fails.
     */

    auto get_boundary_vertex_link = [&mesh](const Tuple& _v) {
        const Simplex input_v(mesh, PrimitiveType::Vertex, _v);
        // std::vector<Tuple> ret;
        IdSimplexCollection ret(mesh);
        // auto incident_faces =
        //    cofaces_single_dimension_tuples(mesh, input_v, PrimitiveType::Triangle);
        auto incident_faces =
            cofaces_single_dimension_iterable(mesh, input_v, PrimitiveType::Triangle);
        for (const Tuple& _f : incident_faces) {
            if (mesh.is_boundary(PrimitiveType::Triangle, _f)) {
                // assuming cofaces_single_dimension always return the tuple point to the input
                // vertex
                // ret.push_back(mesh.switch_tuples(_f, {PrimitiveType::Vertex,
                // PrimitiveType::Edge})); ret.push_back(mesh.switch_edge(mesh.switch_vertex(_f)));
                Tuple link_tuple =
                    mesh.switch_tuples(_f, {PrimitiveType::Vertex, PrimitiveType::Edge});
                ret.add(PrimitiveType::Edge, link_tuple);
                ret.add(PrimitiveType::Vertex, link_tuple);
                ret.add(
                    PrimitiveType::Vertex,
                    mesh.switch_tuple(link_tuple, PrimitiveType::Vertex));
            }
        }
        return ret;
    };

    auto get_boundary_edge_link = [&mesh](const Tuple& _v) {
        const Simplex input_e(mesh, PrimitiveType::Edge, _v);
        IdSimplexCollection ret(mesh);
        auto incident_faces =
            cofaces_single_dimension_iterable(mesh, input_e, PrimitiveType::Triangle);
        for (const Tuple& _f : incident_faces) {
            if (mesh.is_boundary(PrimitiveType::Triangle, _f)) {
                Tuple link_tuple =
                    mesh.switch_tuples(_f, {PrimitiveType::Edge, PrimitiveType::Vertex});
                ret.add(PrimitiveType::Vertex, link_tuple);
            }
        }
        return ret;
    };

    const Tuple& a_tuple = edge;
    const Tuple b_tuple = mesh.switch_tuple(edge, PrimitiveType::Vertex);

    if (mesh.is_boundary_edge(edge)) {
        IdSimplexCollection boundary_link_a = get_boundary_vertex_link(a_tuple);
        boundary_link_a.sort_and_clean();
        IdSimplexCollection boundary_link_b = get_boundary_vertex_link(b_tuple);
        boundary_link_b.sort_and_clean();

        IdSimplexCollection boundary_link_ab = get_boundary_edge_link(edge);
        boundary_link_ab.sort_and_clean();

        IdSimplexCollection boundary_link_a_link_b_intersection =
            IdSimplexCollection::get_intersection(boundary_link_a, boundary_link_b);

        return IdSimplexCollection::are_simplex_collections_equal(
            boundary_link_a_link_b_intersection,
            boundary_link_ab);

    } else if (mesh.is_boundary_vertex(a_tuple) && mesh.is_boundary_vertex(b_tuple)) {
        return false;
    }

    return true;
}

bool link_condition(const Mesh& mesh, const Tuple& edge)
{
    return std::visit(
        [&edge](auto&& m) noexcept {
            using MType = std::decay_t<decltype(m.get())>;
            if constexpr (std::is_same_v<MType, Mesh>) {
                throw std::runtime_error("Link condition called on an unknown type of mesh - could "
                                         "only cast it to Mesh");
            } else if constexpr (std::is_same_v<MType, PointMesh>) {
                return true;
            } else {
                return link_condition(m.get(), edge);
            }
        },
        wmtk::utils::metaprogramming::as_const_mesh_variant(mesh));
}
} // namespace wmtk::simplex
