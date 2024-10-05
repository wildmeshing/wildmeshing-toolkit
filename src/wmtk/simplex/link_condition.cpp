#include "link_condition.hpp"
#include <wmtk/utils/metaprogramming/as_mesh_variant.hpp>
#include "cofaces_single_dimension.hpp"
#include "cofaces_single_dimension_iterable.hpp"
#include "link.hpp"
#include "link_iterable.hpp"
#include "link_single_dimension_iterable.hpp"
#include "open_star.hpp"
#include "utils/SimplexComparisons.hpp"

namespace wmtk::simplex {
bool link_condition_closed_trimesh(const TriMesh& mesh, const Tuple& edge)
{
    const Simplex v_a = Simplex::vertex(mesh, edge);
    const Simplex v_b = Simplex::vertex(mesh, mesh.switch_tuple(edge, PrimitiveType::Vertex));
    const Simplex e_ab = Simplex::edge(mesh, edge);
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
        const Simplex input_v(mesh, PrimitiveType::Vertex, _v);
        std::vector<Tuple> ret;
        // get incident_edges from open_star
        auto incident_edges = open_star(mesh, input_v).simplex_vector(PrimitiveType::Edge);
        for (const Simplex& _e : incident_edges) {
            if (mesh.is_boundary(PrimitiveType::Edge, _e.tuple())) {
                if (utils::SimplexComparisons::equal(
                        mesh,
                        Simplex(mesh, PrimitiveType::Vertex, _e.tuple()),
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
    //const SimplexCollection link_a = link(mesh, v_a); // link(a)
    //const SimplexCollection link_b = link(mesh, v_b); // link(b)
    //const SimplexCollection link_ab = link(mesh, e_ab); // link(ab)
    //
    // const SimplexCollection link_a_link_b_intersection =
    //    SimplexCollection::get_intersection(link_a, link_b);
    //
    // return SimplexCollection::are_simplex_collections_equal(link_a_link_b_intersection, link_ab);

    std::vector<simplex::IdSimplex> link_a;
    std::vector<simplex::IdSimplex> link_b;
    std::vector<simplex::IdSimplex> link_intersection_a_b;
    std::vector<simplex::IdSimplex> link_ab;

    // for (const simplex::IdSimplex& s : link_iterable(mesh, v_a)) {
    //     link_a.emplace_back(s);
    // }
    // std::sort(link_a.begin(), link_a.end());
    // for (const simplex::IdSimplex& s : link_iterable(mesh, v_b)) {
    //     link_b.emplace_back(s);
    // }
    // std::sort(link_b.begin(), link_b.end());
    // std::set_intersection(
    //     link_a.begin(),
    //     link_a.end(),
    //     link_b.begin(),
    //     link_b.end(),
    //     std::back_inserter(link_intersection_a_b));
    //
    // for (const simplex::IdSimplex& s : link_iterable(mesh, e_ab)) {
    //     link_ab.emplace_back(s);
    // }
    // std::sort(link_ab.begin(), link_ab.end());

    for (const Tuple& t : link_single_dimension_iterable(mesh, v_a, PrimitiveType::Vertex)) {
        link_a.emplace_back(mesh.get_id_simplex(t, PrimitiveType::Vertex));
    }
    std::sort(link_a.begin(), link_a.end());
    for (const Tuple& t : link_single_dimension_iterable(mesh, v_b, PrimitiveType::Vertex)) {
        link_b.emplace_back(mesh.get_id_simplex(t, PrimitiveType::Vertex));
    }
    std::sort(link_b.begin(), link_b.end());
    std::set_intersection(
        link_a.begin(),
        link_a.end(),
        link_b.begin(),
        link_b.end(),
        std::back_inserter(link_intersection_a_b));

    for (const Tuple& t : link_single_dimension_iterable(mesh, e_ab, PrimitiveType::Vertex)) {
        link_ab.emplace_back(mesh.get_id_simplex(t, PrimitiveType::Vertex));
    }
    std::sort(link_ab.begin(), link_ab.end());

    if (link_ab.size() != link_intersection_a_b.size()) {
        return false;
    }
    for (size_t i = 0; i < link_ab.size(); ++i) {
        if (!(link_ab[i] == link_intersection_a_b[i])) {
            return false;
        }
    }
    return true;
}

bool link_condition(const TetMesh& mesh, const Tuple& edge)
{
    // close mesh check
    if (!link_condition_closed_tetmesh(mesh, edge)) {
        return false;
    }

    // add dummy vertex and link it to boundary faces
    // the check can be convert from link_dummy(a) \intersect link_dummy(b) == link_dummy(ab) to:
    // if ab is a boundary edge, then if any boundary face incident a shares a common edge with any
    // boundary face incident b, return false.
    // if ab is not boundary edge, then if a is not on
    // boundary or b is not on boundary, return true, otherwise false

    auto get_boundary_faces = [&mesh](const Tuple& _v) {
        const Simplex input_v(mesh, PrimitiveType::Vertex, _v);
        std::vector<Tuple> ret;
        // get incident_faces from open_star
        // auto incident_faces = cofaces_single_dimension_tuples(mesh, input_v,
        // PrimitiveType::Triangle);
        auto incident_faces =
            cofaces_single_dimension_iterable(mesh, input_v, PrimitiveType::Triangle);
        for (const Tuple& _f : incident_faces) {
            if (mesh.is_boundary(PrimitiveType::Triangle, _f)) {
                // if (utils::SimplexComparisons::equal(
                //         mesh,
                //         Simplex(PrimitiveType::Vertex, _f.tuple()),
                //         input_v)) {
                //     // tuple edge is _v - x
                //     ret.push_back(mesh.switch_edge(mesh.switch_vertex(_f.tuple())));
                // } else if ((utils::SimplexComparisons::equal(
                //                mesh,
                //                Simplex(PrimitiveType::Vertex, mesh.switch_vertex(_f.tuple())),
                //                input_v))) {
                //     // tuple edge is x - _v
                //     ret.push_back(mesh.switch_edge(_f.tuple()));
                // } else {
                //     // tuple edge is x - y
                //     ret.push_back(_f.tuple());
                // }

                // assuming cofaces_single_dimension always return the tuple point to the input
                // vertex
                ret.push_back(mesh.switch_edge(mesh.switch_vertex(_f)));
            }
        }
        return ret;
    };

    const auto boundary_neighbors_a = get_boundary_faces(edge);
    const auto boundary_neighbors_b =
        get_boundary_faces(mesh.switch_tuple(edge, PrimitiveType::Vertex));
    if (mesh.is_boundary_edge(edge)) {
        for (const Tuple& f_a : boundary_neighbors_a) {
            for (const Tuple& f_b : boundary_neighbors_b) {
                if (utils::SimplexComparisons::equal(
                        mesh,
                        Simplex(mesh, PrimitiveType::Edge, f_a),
                        Simplex(mesh, PrimitiveType::Edge, f_b))) {
                    // find common face (with dummy vertex), link condition fails
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
