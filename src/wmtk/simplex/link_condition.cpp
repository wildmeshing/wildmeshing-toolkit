#include "link_condition.hpp"
#include "utils/SimplexComparisons.hpp"
#include "link.hpp"
#include "open_star.hpp"

namespace wmtk::simplex {
bool link_condition_closed_trimesh(const TriMesh& mesh, const Tuple& edge)
{
    const Simplex v_a = Simplex::vertex(edge);
    const Simplex v_b = Simplex::vertex(mesh.switch_tuple(edge, PrimitiveType::Vertex));
    const Simplex e_ab = Simplex::edge(edge);
    const SimplexCollection link_a = link(mesh, v_a); // link(a)
    const SimplexCollection link_b = link(mesh, v_b); // link(b)
    const SimplexCollection link_ab = link(mesh, e_ab); // link(ab)

    const SimplexCollection link_a_link_b_intersection =
        SimplexCollection::get_intersection(link_a, link_b);

    return SimplexCollection::are_simplex_collections_equal(link_a_link_b_intersection, link_ab);
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
        const Simplex input_v(PrimitiveType::Vertex, _v);
        std::vector<Tuple> ret;
        // get incident_edges from open_star
        auto incident_edges = open_star(mesh, input_v).simplex_vector(PrimitiveType::Edge);
        for (const Simplex& _e : incident_edges) {
            if (mesh.is_boundary(_e.tuple(), PrimitiveType::Edge)) {
                if (utils::SimplexComparisons::equal(
                        mesh,
                        Simplex(PrimitiveType::Vertex, _e.tuple()),
                        input_v)) {
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
                if (utils::SimplexComparisons::equal(
                        mesh,
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
