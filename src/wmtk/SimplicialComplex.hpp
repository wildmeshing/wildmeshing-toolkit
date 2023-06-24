#pragma once

#include <vector>
#include <set>
#include <cassert>
#include <queue>
#include "TriMesh.hpp"
#include "TetMesh.hpp"

using namespace wmtk;

// note that this code only works for triangle/tet meshes
class Simplex
{
    int _d;   // dimension
    Tuple _t; // tuple
    int _global_id
public:
    Simplex(const int &d, const Tuple &t, const Mesh& m) : _d{d}, _t{t} 
    {
        _global_id = m.id(t, d);
    }

    int global_id() const { return _global_id; }
    int dimension() const { return _d; }
    const Tuple &tuple() const { return _t; }

    bool operator<(const Simplex &rhs) const
    {
        if (_d < rhs._d)
        {
            return true;
        }
        if (_d > rhs._d)
        {
            return false;
        }
        return global_id() < rhs.global_id();
    }

    bool operator==(const Simplex &rhs) const
    {
        return (_d == rhs._d) && (global_id() == rhs.global_id());
    }
};

class SimplicialComplex
{
private:
    std::set<Simplex> simplexes;

public:
    const std::set<Simplex> &get_simplices() const
    {
        return simplexes;
    }

    std::set<Simplex> get_simplices(const int &dim) const
    {
        std::set<Simplex> ret;
        for (const Simplex &s : simplexes)
        {
            if (s.dimension() == dim)
            {
                ret.insert(s);
            }
        }

        return ret;
    }

    /**
     * @brief Add simplex to the complex if it is not already in it.
     *
     * @returns false if simplex is already in the complex
     */
    bool add_simplex(const Simplex &s)
    {
        assert(s.dimension() <= 4);
        const auto [it, was_successful] = simplexes.insert(s);
        return was_successful;
    }

    void unify_with_complex(const SimplicialComplex &other)
    {
        // this is N log(N) complexity
        for (const Simplex &s : other.get_simplices())
        {
            add_simplex(s);
        }
    }

    bool operator==(const SimplicialComplex &other) const
    {
        if (simplexes.size() != other.simplexes.size())
        {
            return false;
        }
        // this is N log(N) complexity
        for (const auto &t1 : simplexes)
        {
            const auto it = other.simplexes.find(t1);
            if (it == other.simplexes.end())
            {
                return false;
            }
        }

        return true;
    }

    SimplicialComplex &operator=(const SimplicialComplex &) = default;

    SimplicialComplex() = default;

    SimplicialComplex(const std::vector<Tuple> &tv, const int dim, const Mesh& m)
    {
        for (const Tuple &t : tv)
        {
            add_simplex(Simplex(dim, t, m));
        }
    }
};

inline SimplicialComplex get_union(const SimplicialComplex &sc1, const SimplicialComplex &sc2)
{
    SimplicialComplex u = sc1;
    u.unify_with_complex(sc2);
    return u;
}

inline SimplicialComplex get_intersection(const SimplicialComplex &A, const SimplicialComplex &B)
{
    SimplicialComplex sc_union = A;
    SimplicialComplex sc_intersection;

    for (const auto &s : B.get_simplices())
    {
        if (!sc_union.add_simplex(s))
        {
            // s is already in A --> s is in the intersection of A and B
            sc_intersection.add_simplex(s);
        }
    }

    return sc_intersection;
}

//////////////////////////////////
// List of Operators
// bd: boundary
// clbd: closed boudnary
// st: start
// clst: closed star
// lnk: link
//////////////////////////////////

// ∂s
/**
 * @brief get the boundary of a simplex
 */
SimplicialComplex boundary(const Simplex &s, const Mesh &m)
{
    SimplicialComplex sc;

    // exhaustive implementation
    switch (s.dimension())
    {
    case 3:                                     
        // bd(tet) = 4triangles + 6 edges + 4vertices
        sc.add_simplex(Simplex(0, s.tuple(), m));                                     // A
        sc.add_simplex(Simplex(0, m.switch_tuple(s.tuple(),static_cast<PrimitiveType>(0)), m));                            // B
        sc.add_simplex(Simplex(0, m.switch_tuple(m.switch_tuple(s.tuple(),static_cast<PrimitiveType>(1)),static_cast<PrimitiveType>(0)), m));                   // C
        sc.add_simplex(Simplex(0, m.switch_tuple(m.switch_tuple(s.tuple(),static_cast<PrimitiveType>(2)),static_cast<PrimitiveType>(0)), m));                   // D
        sc.add_simplex(Simplex(1, s.tuple(), m));                                     // AB
        sc.add_simplex(Simplex(1, m.switch_tuple(s.tuple(),static_cast<PrimitiveType>(1)), m));                            // AC
        sc.add_simplex(Simplex(1, m.switch_tuple(m.switch_tuple(s.tuple(),static_cast<PrimitiveType>(0)),static_cast<PrimitiveType>(1)), m));                   // BC
        sc.add_simplex(Simplex(1, m.switch_tuple(m.switch_tuple(s.tuple(),static_cast<PrimitiveType>(2)),static_cast<PrimitiveType>(1)), m));                   // AD
        sc.add_simplex(Simplex(1, m.switch_tuple(m.switch_tuple(m.switch_tuple(s.tuple(),static_cast<PrimitiveType>(0)),static_cast<PrimitiveType>(2)),static_cast<PrimitiveType>(1)), m));          // BD
        sc.add_simplex(Simplex(1, m.switch_tuple(m.switch_tuple(m.switch_tuple(m.switch_tuple(s.tuple(),static_cast<PrimitiveType>(1)),static_cast<PrimitiveType>(0)),static_cast<PrimitiveType>(2)),static_cast<PrimitiveType>(1)), m)); // CD
        sc.add_simplex(Simplex(2, s.tuple(), m));                                     // ABC
        sc.add_simplex(Simplex(2, m.switch_tuple(s.tuple(),static_cast<PrimitiveType>(2)), m));                            // ABD
        sc.add_simplex(Simplex(2, m.switch_tuple(m.switch_tuple(s.tuple(),static_cast<PrimitiveType>(1)),static_cast<PrimitiveType>(2)), m));                   // ACD
        sc.add_simplex(Simplex(2, m.switch_tuple(m.switch_tuple(m.switch_tuple(s.tuple(),static_cast<PrimitiveType>(0)),static_cast<PrimitiveType>(1)),static_cast<PrimitiveType>(2)), m));          // BCD
        break;
    case 2: // bd(triangle) = 3edges + 3vertices
        sc.add_simplex(Simplex(0, s.tuple(), m));
        sc.add_simplex(Simplex(0, m.switch_tuple(s.tuple(),static_cast<PrimitiveType>(0)), m));
        sc.add_simplex(Simplex(0, m.switch_tuple(m.switch_tuple(s.tuple(),static_cast<PrimitiveType>(1)),static_cast<PrimitiveType>(0)), m));
        sc.add_simplex(Simplex(1, s.tuple(), m));
        sc.add_simplex(Simplex(1, m.switch_tuple(s.tuple(),static_cast<PrimitiveType>(1)), m));
        sc.add_simplex(Simplex(1, m.switch_tuple(m.switch_tuple(s.tuple(),static_cast<PrimitiveType>(0)),static_cast<PrimitiveType>(1)), m));
        /* code */
        break;
    case 1:
        // bd(edge) = 2 vertices
        sc.add_simplex(Simplex(0, s.tuple(), m));
        sc.add_simplex(Simplex(0, s.tuple().switch_tuple(static_cast<PrimitiveType>(0), m), m));
        /* code */
        break;
    case 0:
        break;
    default:
        assert(false);
        break;
    }

    return sc;
}

// ∂s∪{s}
/**
 * @brief get complex of a simplex and its boundary
 */
SimplicialComplex simplex_with_boundary(const Simplex &s, const Mesh &m)
{
    SimplicialComplex sc = boundary(s, m);
    sc.add_simplex(s);
    return sc;
}

// Simplex s1,s2, check if A∩B!=∅
// check is intersect(∂s1, ∂s2) has intersections
/**
 * @brief check if simplices with their boundary intersect
 */
inline bool simplices_w_boundary_intersect(const Simplex &s1, const Simplex &s2, const Mesh &m)
{
    SimplicialComplex s1_bd = simplex_with_boundary(s1, m);
    SimplicialComplex s2_bd = simplex_with_boundary(s2, m);
    SimplicialComplex s1_s2_int = get_intersection(s1_bd, s2_bd);
    return (s1_s2_int.get_simplices().size() != 0);
}

SimplicialComplex closed_star(const Simplex &s, const Mesh &m)
{
    SimplicialComplex sc;
    const int &cell_dim = m->cell_dimension(); // TODO: 2 for trimesh, 3 for tetmesh need it in Mesh class

    if (cell_dim == 2)
    {
        switch (s.dimension())
        {
        case 0:
        {
            std::queue<Tuple> q;
            q.push(s.tuple());
            while (!q.empty())
            {
                const Tuple t = q.front();
                q.pop();
                if (sc.add_simplex(Simplex(2, t, m)))
                {
                    if (!m.is_boundary(t))
                    {
                        q.push(m.switch_tuple(t, static_cast<PrimitiveType>(2)));
                    }
                    if (!m.is_boundary(m.switch_tuple(t, static_cast<PrimitiveType>(1))))
                    {
                        q.push(m.switch_tuple(m.switch_tuple(t,static_cast<PrimitiveType>(1)),static_cast<PrimitiveType>(2)));
                    }
                }
            }
            break;
        }
        case 1:
            sc.add_simplex(Simplex(2, s.tuple(), m));
            if (!m.is_boundary(s.tuple()))
            {
                sc.add_simplex(Simplex(2, m.switch_tuple(s.tuple(), static_cast<PrimitiveType>(2)), m));
            }
            break;
        case 2:
            sc.add_simplex(s);
            break;
        default:
            assert(false);
            break;
        }
    }
    else if (cell_dim == 3)
    {
        switch (s.dimension())
        {
        case 0:
        {
            std::queue<Tuple> q;
            q.push(s.tuple());
            while (!q.empty())
            {
                Tuple t = q.front();
                q.pop();
                if (sc.add_simplex(Simplex(3, t, m)))
                {
                    const Tuple t1 = t;
                    const Tuple t2 = m.switch_tuple(t,static_cast<PrimitiveType>(2));
                    const Tuple t3 = m.switch_tuple(m.switch_tuple(t,static_cast<PrimitiveType>(1)),static_cast<PrimitiveType>(2));
                    if (!m.is_boundary(t1))
                    {
                        q.push(m.switch_tuple(t1,static_cast<PrimitiveType>(3)));
                    }
                    if (!m.is_boundary(t2))
                    {
                        q.push(m.switch_tuple(t2,static_cast<PrimitiveType>(3)));
                    }
                    if (!m.is_boundary(t3))
                    {
                        q.push(m.switch_tuple(t3,static_cast<PrimitiveType>(3)));
                    }
                }
            }
            break;
        }
        case 1:
        {
            std::queue<Tuple> q;
            q.push(s.tuple());
            while (!q.empty())
            {
                Tuple t = q.front();
                q.pop();
                if (sc.add_simplex(Simplex(3, t, m)))
                {
                    if (!m.is_boundary(t))
                    {
                        q.push(m.switch_tuple(t,static_cast<PrimitiveType>(3)));
                    }
                    if (!m.is_boundary(m.switch_tuple(t,static_cast<PrimitiveType>(2))))
                    {
                        q.push(m.switch_tuple(m.switch_tuple(t,static_cast<PrimitiveType>(2)),static_cast<PrimitiveType>(3)));
                    }
                }
            }
            break;
        }
        case 2:
        {
            sc.add_simplex(Simplex(3, s.tuple(), m));
            if (!m.is_boundary(s.tuple()))
            {
                sc.add_simplex(Simplex(3, m.switch_tuple(s.tuple(),static_cast<PrimitiveType>(3)), m));
            }
            break;
        }
        case 3:
        {
            sc.add_simplex(s);
            break;
        }
        default:
        {
            assert(false);
            break;
        }
        }
    }

    const auto top_simplices = sc.get_simplices();
    for (const Simplex &ts : top_simplices)
    {
        sc.unify_with_complex(boundary(ts, m));
    }
    return sc;
}

SimplicialComplex link(const Simplex &s, const Mesh &m)
{
    SimplicialComplex sc_clst = closed_star(s, m);
    SimplicialComplex sc;
    for (const Simplex &ss : sc_clst.get_simplices())
    {
        if (!simplices_w_boundary_intersect(s, ss, m))
        {
            sc.add_simplex(ss);
        }
    }

    return sc;
}

SimplicialComplex open_star(const Simplex &s, const Mesh &m)
{
    SimplicialComplex sc_clst = closed_star(s, m);
    SimplicialComplex sc;
    sc.add_simplex(s);
    for (const Simplex &ss : sc_clst.get_simplices())
    {
        if (ss.dimension() <= s.dimension())
        {
            continue;
        }
        if (simplices_w_boundary_intersect(s, ss, m))
        {
            sc.add_simplex(ss);
        }
    }

    return sc;
}

//////////////////////////////////
// check link condition
// input Tuple t --> edge (a,b)
// check if lnk(a) ∩ lnk(b) == lnk(ab)
//////////////////////////////////
bool link_cond(Tuple t, const Mesh &m)
{
    SimplicialComplex lhs = link(Simplex(0, t, m), m); // lnk(a)
    lhs.unify_with_complex(link(Simplex(0, m.switch_tuple(t,static_cast<PrimitiveType>(0)), m), m)); // Union lnk(b)

    SimplicialComplex rhs = link(Simplex(1, t, m), m); // lnk(ab)
    return (lhs == rhs);
}

//////////////////////////////////
// k-ring
//////////////////////////////////
/**
 * @brief get one ring neighbors of vertex in _t_
 */
std::vector<Tuple> vertex_one_ring(Tuple t, const Mesh &m)
{
    Simplex s(0, t);
    SimplicialComplex sc_link = link(s, m);
    std::set<Simplex> one_ring_simplices = sc_link.get_simplices(0);
    return std::vector<Tuple>(one_ring_simplices.begin(), one_ring_simplices.end());
}

std::vector<Tuple> k_ring(Tuple t, const Mesh &m, int k)
{
    if (k < 1)
        return {};

    SimplicialComplex sc(vertex_one_ring(t, m), 0);
    for (int i = 2; i <= k; ++i)
    {
        const auto simplices = sc.get_simplices();
        for (const Simplex &s : simplices)
        {
            SimplicialComplex sc_or(vertex_one_ring(s.tuple(), m), 0);
            sc.unify_with_complex(sc_or);
        }
    }

    std::set<Simplex> k_ring_simplices = sc.get_simplices();
    return std::vector<Tuple>(k_ring_simplices.begin(), k_ring_simplices.end());
}
