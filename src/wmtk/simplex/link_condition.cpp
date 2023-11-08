#include "link_condition.hpp"
#include "link.hpp"
#include "open_star.hpp"

namespace wmtk::simplex {
bool link_condition_closed_trimesh(const TriMesh& mesh, const Tuple& edge)
{
    SimplexCollection lnk_a = link(mesh, Simplex(PrimitiveType::Vertex, edge), true); // link(a)
    SimplexCollection lnk_b = link(
        mesh,
        Simplex(PrimitiveType::Vertex, mesh.switch_tuple(edge, PrimitiveType::Vertex)),
        true); // link(b)
    SimplexCollection lnk_ab = link(mesh, Simplex(PrimitiveType::Edge, edge), true); // link(ab)

    SimplexCollection lnk_a_lnk_b_intersection = SimplexCollection::get_intersection(lnk_a, lnk_b);

    return SimplexCollection::are_simplex_collections_equal(lnk_a_lnk_b_intersection, lnk_ab);
}

bool link_condition(const EdgeMesh& mesh, const Tuple& edge)
{
    const Tuple edge_switch_v = mesh.switch_tuple(edge, PrimitiveType::Vertex);
    if (mesh.is_boundary_vertex(edge) && mesh.is_boundary_vertex(edge_switch_v)) {
        return false;
    }
    if (mesh.simplices_are_equal(
            Simplex(PrimitiveType::Vertex, edge),
            Simplex(PrimitiveType::Vertex, edge_switch_v))) {
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

    // check if dummy vertex w is included in the lhs
    auto get_boundary_edges = [&mesh](const Tuple& _v) {
        Simplex input_v(PrimitiveType::Vertex, _v);
        std::vector<Tuple> ret;
        // get one_ring_edges from open_star
        auto one_ring_edges = open_star(mesh, input_v).simplex_vector(PrimitiveType::Edge);
        for (const auto& _e : one_ring_edges) {
            if (mesh.is_boundary(_e.tuple(), PrimitiveType::Edge)) {
                if (mesh.simplices_are_equal(Simplex(PrimitiveType::Vertex, _e.tuple()), input_v)) {
                    ret.push_back(mesh.switch_tuple(_e.tuple(), PrimitiveType::Vertex));
                } else {
                    ret.push_back(_e.tuple());
                }
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
                if (mesh.simplices_are_equal(
                        Simplex(PrimitiveType::Vertex, e_a),
                        Simplex(PrimitiveType::Vertex, e_b))) {
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

} // namespace wmtk::simplex
