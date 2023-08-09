#include "SimplicialComplex.hpp"

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

SimplicialComplex SimplicialComplex::boundary(const Simplex& s, const Mesh& m)
{
    SimplicialComplex sc(m);

    constexpr PrimitiveType PV = PrimitiveType::Vertex;
    constexpr PrimitiveType PE = PrimitiveType::Edge;
    constexpr PrimitiveType PF = PrimitiveType::Face;

    const Tuple t = s.tuple();

    auto sw = [&m](const Tuple& _t, const PrimitiveType& _ptype) {
        return m.switch_tuple(_t, _ptype);
    };


    // exhaustive implementation
    switch (s.primitive_type()) {
    case PrimitiveType::Tetrahedron:
        // bd(tet) = 4triangles + 6 edges + 4vertices
        sc.add_simplex(Simplex(PV, t)); // A
        sc.add_simplex(Simplex(PV, sw(t, PV))); // B
        sc.add_simplex(Simplex(PV, sw(sw(t, PE),
                                      PV))); // C
        sc.add_simplex(Simplex(PV, sw(sw(t, PF),
                                      PV))); // D
        sc.add_simplex(Simplex(PE, t)); // AB
        sc.add_simplex(Simplex(PE, sw(t, PE))); // AC
        sc.add_simplex(Simplex(PE, sw(sw(t, PV),
                                      PE))); // BC
        sc.add_simplex(Simplex(PE, sw(sw(t, PF),
                                      PE))); // AD
        sc.add_simplex(Simplex(PE, sw(sw(sw(t, PV), PF),
                                      PE))); // BD
        sc.add_simplex(Simplex(PE, sw(sw(sw(sw(t, PE), PV), PF),
                                      PE))); // CD
        sc.add_simplex(Simplex(PF, t)); // ABC
        sc.add_simplex(Simplex(PF, sw(t, PF))); // ABD
        sc.add_simplex(Simplex(PF, sw(sw(t, PE),
                                      PF))); // ACD
        sc.add_simplex(Simplex(PF, sw(sw(sw(t, PV), PE),
                                      PF))); // BCD
        break;
    case PF: // bd(triangle) = 3edges + 3vertices
        sc.add_simplex(Simplex(PV, t));
        sc.add_simplex(Simplex(PV, sw(t, PV)));
        sc.add_simplex(Simplex(PV, sw(sw(t, PE), PV)));
        sc.add_simplex(Simplex(PE, t));
        sc.add_simplex(Simplex(PE, sw(t, PE)));
        sc.add_simplex(Simplex(PE, sw(sw(t, PV), PE)));
        /* code */
        break;
    case PE:
        // bd(edge) = 2 vertices
        sc.add_simplex(Simplex(PV, t));
        sc.add_simplex(Simplex(PV, sw(t, PV)));
        /* code */
        break;
    case PV: break;
    default: assert(false); break;
    }

    return sc;
}

SimplicialComplex SimplicialComplex::simplex_with_boundary(const Simplex& s, const Mesh& m)
{
    SimplicialComplex sc = boundary(s, m);
    sc.add_simplex(s);
    return sc;
}

bool SimplicialComplex::simplices_w_boundary_intersect(
    const Simplex& s1,
    const Simplex& s2,
    const Mesh& m)
{
    SimplicialComplex s1_bd = simplex_with_boundary(s1, m);
    SimplicialComplex s2_bd = simplex_with_boundary(s2, m);
    SimplicialComplex s1_s2_int = get_intersection(s1_bd, s2_bd);
    return (s1_s2_int.get_simplices().size() != 0);
}

SimplicialComplex SimplicialComplex::closed_star(const Simplex& s, const Mesh& m)
{
    SimplicialComplex sc(m);

    constexpr PrimitiveType PV = PrimitiveType::Vertex;
    constexpr PrimitiveType PE = PrimitiveType::Edge;
    constexpr PrimitiveType PF = PrimitiveType::Face;
    constexpr PrimitiveType PT = PrimitiveType::Tetrahedron;

    const Tuple t = s.tuple();

    auto sw = [&m](const Tuple& _t, const PrimitiveType& _ptype) {
        return m.switch_tuple(_t, _ptype);
    };

    // const int &cell_dim = m->cell_dimension(); // TODO: 2 for trimesh, 3 for tetmesh need it in Mesh class
    const int cell_dim = dynamic_cast<const TriMesh*>(&m) ? 2 : 3;
    if (cell_dim == 2) {
        switch (s.primitive_type()) {
        case PV: {
            std::queue<Tuple> q;
            q.push(s.tuple());
            while (!q.empty()) {
                const Tuple cur_t = q.front();
                q.pop();
                if (sc.add_simplex(Simplex(PF, cur_t))) {
                    if (!m.is_boundary(cur_t)) {
                        q.push(sw(cur_t, PF));
                    }
                    if (!m.is_boundary(sw(cur_t, PE))) {
                        q.push(sw(sw(cur_t, PE), PF));
                    }
                }
            }
            break;
        }
        case PE:
            sc.add_simplex(Simplex(PF, t));
            if (!m.is_boundary(t)) {
                sc.add_simplex(Simplex(PF, sw(t, PF)));
            }
            break;
        case PF: sc.add_simplex(s); break;
        case PT: assert(false); break;
        default: assert(false); break;
        }
    } else if (cell_dim == 3) {
        switch (s.primitive_type()) {
        case PV: {
            std::queue<Tuple> q;
            q.push(t);
            while (!q.empty()) {
                Tuple cur_t = q.front();
                q.pop();
                if (sc.add_simplex(Simplex(PF, cur_t))) {
                    const Tuple t1 = cur_t;
                    const Tuple t2 = sw(cur_t, PF);
                    const Tuple t3 = sw(sw(cur_t, PE), PF);
                    if (!m.is_boundary(t1)) {
                        q.push(sw(t1, PT));
                    }
                    if (!m.is_boundary(t2)) {
                        q.push(sw(t2, PT));
                    }
                    if (!m.is_boundary(t3)) {
                        q.push(sw(t3, PT));
                    }
                }
            }
            break;
        }
        case PE: {
            std::queue<Tuple> q;
            q.push(t);
            while (!q.empty()) {
                Tuple cur_t = q.front();
                q.pop();
                if (sc.add_simplex(Simplex(PT, cur_t))) {
                    if (!m.is_boundary(cur_t)) {
                        q.push(sw(cur_t, PT));
                    }
                    if (!m.is_boundary(sw(cur_t, PF))) {
                        q.push(sw(sw(cur_t, PF), PT));
                    }
                }
            }
            break;
        }
        case PF: {
            sc.add_simplex(Simplex(PT, t));
            if (!m.is_boundary(t)) {
                sc.add_simplex(Simplex(PT, sw(t, PT)));
            }
            break;
        }
        case PT: {
            sc.add_simplex(s);
            break;
        }
        default: {
            assert(false);
            break;
        }
        }
    }

    const auto top_simplices = sc.get_simplices();
    for (const Simplex& ts : top_simplices) {
        sc.unify_with_complex(boundary(ts, m));
    }
    return sc;
}

SimplicialComplex SimplicialComplex::link(const Simplex& s, const Mesh& m)
{
    SimplicialComplex sc_clst = closed_star(s, m);
    SimplicialComplex sc(m);
    for (const Simplex& ss : sc_clst.get_simplices()) {
        if (!SimplicialComplex::simplices_w_boundary_intersect(s, ss, m)) {
            sc.add_simplex(ss);
        }
    }

    return sc;
}

SimplicialComplex SimplicialComplex::open_star(const Simplex& s, const Mesh& m)
{
    SimplicialComplex sc_clst = closed_star(s, m);
    SimplicialComplex sc(m);
    sc.add_simplex(s);
    for (const Simplex& ss : sc_clst.get_simplices()) {
        if (ss.primitive_type() <= s.primitive_type()) {
            continue;
        }
        if (simplices_w_boundary_intersect(s, ss, m)) {
            sc.add_simplex(ss);
        }
    }

    return sc;
}

bool SimplicialComplex::link_cond(const Mesh& m, Tuple t)
{
    SimplicialComplex lnk_a = link(Simplex(PrimitiveType::Vertex, t), m); // lnk(a)
    SimplicialComplex lnk_b = link(
        Simplex(PrimitiveType::Vertex, m.switch_tuple(t, PrimitiveType::Vertex)),
        m); // lnk(b)
    SimplicialComplex lhs = get_intersection(lnk_a, lnk_b);

    SimplicialComplex rhs = link(Simplex(PrimitiveType::Edge, t), m); // lnk(ab)

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
        auto one_ring_edges = open_star(input_v, m).get_simplices(PrimitiveType::Edge);
        for (auto _e : one_ring_edges) {
            if (m.is_boundary(_e.tuple())) {
                if (m.simplex_is_equal(Simplex(PrimitiveType::Vertex, _e.tuple()), input_v)) {
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
                if (m.simplex_is_equal(
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
bool SimplicialComplex::edge_collapse_possible_2d(const Mesh& m, Tuple t)
{
    // initial vertex join conditions:

    // cannot collapse edges connecting two boundaries unless the edge itself is a boundary
    // assert(tip(h) != boundary || tip(opp(h)) != boundary || h == boundary);
    auto is_bd_v = [&m](const Tuple& _v) {
        Simplex input_v(PrimitiveType::Vertex, _v);
        // get one_ring_edges from open_star
        auto one_ring_edges = open_star(input_v, m).get_simplices(PrimitiveType::Edge);
        for (auto _e : one_ring_edges) {
            if (m.is_boundary(_e.tuple())) {
                return true;
            }
        }
        return false;
    };
    if (!(!is_bd_v(t) || !is_bd_v(m.switch_tuple(t, PrimitiveType::Vertex)) || m.is_boundary(t))) {
        return false;
    }

    auto next = [&m](const Tuple& _h) {
        return m.switch_tuple(m.switch_tuple(_h, PrimitiveType::Vertex), PrimitiveType::Edge);
    };

    auto opp = [&m](const Tuple& _h) {
        return m.switch_tuple(m.switch_tuple(_h, PrimitiveType::Face), PrimitiveType::Vertex);
    };

    // valence 1 check
    // assert(next(h) != opp(h) || next(opp(h)) != h);
    if (!m.is_boundary(t)) {
        if (next(t) == opp(t) && next(opp(t)) == t) {
            return false;
        }
    }

    // two face joins checks
    // auto h0 = next(h);
    // auto h1 = next(opp(h));
    // conditions needed for join_face to work
    // assert(face(h0) != face(opp(h0)));
    // assert(face(h1) != face(opp(h1)));
    // LEYI: equivalent check:
    // assert(opp(h0) != next(h0))
    // assert(opp(h1) != next(h1))

    auto h0 = next(t);
    if (!m.is_boundary(h0)) {
        if (opp(h0) == next(h0)) {
            return false;
        }
    }

    if (!m.is_boundary(t)) {
        auto h1 = next(opp(t));
        if (!m.is_boundary(h1)) {
            if (opp(h1) == next(h1)) {
                return false;
            }
        }
    }

    return true;
}

std::vector<Simplex> SimplicialComplex::vertex_one_ring(const Mesh& m, Tuple t)
{
    Simplex s(PrimitiveType::Vertex, t);
    const SimplicialComplex sc_link = link(s, m);
    auto vs = sc_link.get_simplices(PrimitiveType::Vertex);
    return std::vector<Simplex>(vs.begin(), vs.end());
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
