#include "SimplicialComplex.hpp"

#include "simplex/SimplexCollection.hpp"
#include "simplex/closed_star.hpp"
#include "simplex/top_level_cofaces.hpp"
#include "simplex/link.hpp"
#include "simplex/open_star.hpp"
#include "simplex/simplex_boundary.hpp"

#include "TriMesh.hpp"

namespace wmtk {

internal::SimplexSet SimplicialComplex::get_simplices(const PrimitiveType& ptype) const
{
    internal::SimplexSet ret(_simplices.key_comp());
    for (const Simplex& s : _simplices) {
        if (s.primitive_type() == ptype) {
            ret.insert(s);
        }
    }

    return ret;
}

std::vector<Simplex> SimplicialComplex::get_simplex_vector() const
{
    return std::vector<Simplex>(_simplices.begin(), _simplices.end());
}

bool SimplicialComplex::add_simplex(const Simplex& s)
{
    const auto [it, was_successful] = _simplices.insert(s);
    return was_successful;
}

void SimplicialComplex::unify_with_complex(const SimplicialComplex& other)
{
    // this is N log(N) complexity
    for (const Simplex& s : other.get_simplices()) {
        add_simplex(s);
    }
}

bool SimplicialComplex::operator==(const SimplicialComplex& other) const
{
    if (_simplices.size() != other._simplices.size()) {
        return false;
    }
    // this is N log(N) complexity
    for (const auto& t1 : _simplices) {
        const auto it = other._simplices.find(t1);
        if (it == other._simplices.end()) {
            return false;
        }
    }

    return true;
}

SimplicialComplex SimplicialComplex::get_union(
    const SimplicialComplex& sc1,
    const SimplicialComplex& sc2)
{
    SimplicialComplex u = sc1;
    u.unify_with_complex(sc2);
    return u;
}

SimplicialComplex SimplicialComplex::get_intersection(
    const SimplicialComplex& A,
    const SimplicialComplex& B)
{
    SimplicialComplex sc_union = A;
    SimplicialComplex sc_intersection(A.key_comp());

    for (const auto& s : B.get_simplices()) {
        if (!sc_union.add_simplex(s)) {
            // s is already in A --> s is in the intersection of A and B
            sc_intersection.add_simplex(s);
        }
    }

    return sc_intersection;
}

SimplicialComplex SimplicialComplex::boundary(const Mesh& m, const Simplex& s)
{
    simplex::SimplexCollection coll = simplex::simplex_boundary(m, s);
    return SimplicialComplex(coll.simplex_vector(), m);
}

SimplicialComplex SimplicialComplex::simplex_with_boundary(const Mesh& m, const Simplex& s)
{
    simplex::SimplexCollection coll = simplex::simplex_boundary(m, s, false);
    coll.add(s);
    coll.sort_and_clean();
    return SimplicialComplex(coll.simplex_vector(), m);
}

bool SimplicialComplex::simplices_w_boundary_intersect(
    const Mesh& m,
    const Simplex& s1,
    const Simplex& s2)
{
    SimplicialComplex s1_bd = simplex_with_boundary(m, s1);
    SimplicialComplex s2_bd = simplex_with_boundary(m, s2);
    SimplicialComplex s1_s2_int = get_intersection(s1_bd, s2_bd);
    return (s1_s2_int.get_simplices().size() != 0);
}

SimplicialComplex SimplicialComplex::top_coface_simplex(const Mesh& m, const Simplex& s)
{
    simplex::SimplexCollection coll = simplex::top_level_cofaces(m, s);
    return SimplicialComplex(coll.simplex_vector(), m);
}

SimplicialComplex SimplicialComplex::closed_star(const Mesh& m, const Simplex& s)
{
    simplex::SimplexCollection coll = simplex::closed_star(m, s);
    return SimplicialComplex(coll.simplex_vector(), m);
}

SimplicialComplex SimplicialComplex::link(const Mesh& m, const Simplex& s)
{
    simplex::SimplexCollection coll = simplex::link(m, s);
    return SimplicialComplex(coll.simplex_vector(), m);
}

SimplicialComplex SimplicialComplex::open_star(const Mesh& m, const Simplex& s)
{
    simplex::SimplexCollection coll = simplex::open_star(m, s);
    return SimplicialComplex(coll.simplex_vector(), m);
}

bool SimplicialComplex::link_cond(const Mesh& m, Tuple t)
{
    SimplicialComplex lnk_a = link(m, Simplex(PrimitiveType::Vertex, t)); // lnk(a)
    SimplicialComplex lnk_b =
        link(m, Simplex(PrimitiveType::Vertex, m.switch_tuple(t, PrimitiveType::Vertex))); // lnk(b)
    SimplicialComplex lhs = get_intersection(lnk_a, lnk_b);

    SimplicialComplex rhs = link(m, Simplex(PrimitiveType::Edge, t)); // lnk(ab)

    return (lhs == rhs);
}

// work for 2-manifold case only for now
bool SimplicialComplex::link_cond_bd_2d(const Mesh& m, Tuple t)
{
    // step1 check normal link condition
    if (!link_cond(m, t)) {
        return false;
    }
    // check if dummy vertex w is included in the lhs

    auto get_bd_edges = [&m](const Tuple& _v) {
        Simplex input_v(PrimitiveType::Vertex, _v);
        std::vector<Tuple> ret;
        // get one_ring_edges from open_star
        auto one_ring_edges = open_star(m, input_v).get_simplices(PrimitiveType::Edge);
        for (const auto& _e : one_ring_edges) {
            if (m.is_boundary(_e.tuple())) {
                if (m.simplices_are_equal(Simplex(PrimitiveType::Vertex, _e.tuple()), input_v)) {
                    ret.push_back(m.switch_tuple(_e.tuple(), PrimitiveType::Vertex));
                } else {
                    ret.push_back(_e.tuple());
                }
            }
        }
        return ret;
    };
    // case 1: edge ab is a boundary edge, in this case dummy vertex w is in lnk(ab), need to check
    // if there are any common edges connected with w in lnk_w^0(a)∩lnk_w^0(b)
    auto bd_neighbors_a = get_bd_edges(t);
    auto bd_neighbors_b = get_bd_edges(m.switch_tuple(t, PrimitiveType::Vertex));
    if (m.is_boundary(t)) {
        assert(bd_neighbors_a.size() == 2); // if guarantee 2-manifold
        assert(bd_neighbors_b.size() == 2); // if guarantee 2-manifold
        for (auto e_a : bd_neighbors_a) {
            for (auto e_b : bd_neighbors_b) {
                if (m.simplices_are_equal(
                        Simplex(PrimitiveType::Vertex, e_a),
                        Simplex(PrimitiveType::Vertex, e_b))) {
                    // find common edge, link condition fails
                    return false;
                }
            }
        }
    } else {
        if (bd_neighbors_a.size() == 0 || bd_neighbors_b.size() == 0) {
            // in this case, lnk_w^0(a) ∩ lnk_w^0(b) == lnk(a) ∩ lnk(b) == lnk(ab) == lnk_w^0(ab)
            return true;
        } else {
            // in this case w \in lhs but not \in rhs
            return false;
        }
    }

    return true;
}


// Toplogical-holding condition, not necessarily guarantee geometric embedding
bool SimplicialComplex::edge_collapse_possible_2d(const TriMesh& m, const Tuple& t)
{
    // cannot collapse edges connecting two boundaries unless the edge itself is a boundary
    if (m.is_boundary_vertex(t) && m.is_boundary_vertex(m.switch_vertex(t)) && !m.is_boundary(t)) {
        return false;
    }

    auto opp = [&m](const Tuple& t) { return m.switch_vertex(m.switch_face(t)); };

    const Tuple h0 = m.next_edge(t);
    const Tuple h0_next = m.next_edge(h0);

    if (!m.is_boundary(h0)) {
        const Tuple h0_opp = opp(h0);
        if (h0_opp == h0_next) {
            return false;
        }
    }

    // valence 1 check
    if (!m.is_boundary(t)) {
        const Tuple t_opp = opp(t);
        const Tuple h1 = m.next_edge(t_opp);
        const Tuple h1_next = m.next_edge(h1);
        if (h0 == t_opp && h1 == t) {
            return false;
        }
        if (!m.is_boundary(h1)) {
            const Tuple h1_opp = opp(h1);
            if (h1_opp == h1_next) {
                return false;
            }
        }
    }

    return true;
}

std::vector<Simplex> SimplicialComplex::vertex_one_ring(const Mesh& m, Tuple t)
{
    simplex::SimplexCollection coll = simplex::link(m, Simplex::vertex(t));
    return coll.simplex_vector(PrimitiveType::Vertex);
}

std::vector<Simplex> SimplicialComplex::vertex_one_ring(const TriMesh& m, Tuple t)
{
    std::vector<Simplex> one_ring;
    one_ring.reserve(6);

    Tuple iter = t;
    do {
        one_ring.emplace_back(Simplex::vertex(m.switch_vertex(iter)));
        if (m.is_boundary(iter)) {
            break;
        }
        iter = m.switch_edge(m.switch_face(iter));
    } while (iter != t);

    // in case of a boundary, collect the other side too
    if (iter != t || m.is_boundary(t)) {
        iter = m.switch_edge(t);
        do {
            one_ring.emplace_back(Simplex::vertex(m.switch_vertex(iter)));
            if (m.is_boundary(iter)) {
                break;
            }
            iter = m.switch_edge(m.switch_face(iter));
        } while (iter != t);
    }

    return one_ring;
}

std::vector<Simplex> SimplicialComplex::k_ring(const Mesh& m, Tuple t, int k)
{
    if (k < 1) return {};

    SimplicialComplex sc(vertex_one_ring(m, t), m);
    for (int i = 2; i <= k; ++i) {
        const auto simplices = sc.get_simplices();
        for (const Simplex& s : simplices) {
            SimplicialComplex sc_or(vertex_one_ring(m, s.tuple()), m);
            sc.unify_with_complex(sc_or);
        }
    }

    auto vs = sc.get_simplices(PrimitiveType::Vertex);
    return std::vector<Simplex>(vs.begin(), vs.end());
}

} // namespace wmtk
