#pragma once

#include <queue>
#include <set>
#include <vector>
#include "Simplex.hpp"
#include "TetMesh.hpp"
#include "TriMesh.hpp"

namespace wmtk {

namespace internal {


struct SimplexLessFunctor
{
    const Mesh& m;

    SimplexLessFunctor(const Mesh& mm)
        : m(mm)
    {}

    bool operator()(const Simplex& s0, const Simplex& s1) const
    {
        return m.simplex_is_less(s0, s1);
    }
};

using SimplexSet = std::set<Simplex, SimplexLessFunctor>;

} // namespace internal

class SimplicialComplex
{
private:
    internal::SimplexSet simplices;

public:
    const internal::SimplexSet& get_simplices() const { return simplices; }

    internal::SimplexSet get_simplices(const PrimitiveType& ptype) const;

    /**
     * @brief Add simplex to the complex if it is not already in it.
     *
     * @returns false if simplex is already in the complex
     */
    bool add_simplex(const Simplex& s);

    void unify_with_complex(const SimplicialComplex& other);

    bool operator==(const SimplicialComplex& other) const;

    //    SimplicialComplex& operator=(const SimplicialComplex&) = default;

    SimplicialComplex(const Mesh& mm)
        : simplices(internal::SimplexLessFunctor(mm))
    {}

    SimplicialComplex(const internal::SimplexLessFunctor& slf)
        : simplices(slf)
    {}

    SimplicialComplex(const std::vector<Simplex>& ss, const Mesh& mm)
        : simplices(internal::SimplexLessFunctor(mm))
    {
        for (const Simplex& s : ss) {
            add_simplex(s);
        }
    }

    auto key_comp() const { return simplices.key_comp(); }
};

SimplicialComplex get_union(const SimplicialComplex& sc1, const SimplicialComplex& sc2);

SimplicialComplex get_intersection(const SimplicialComplex& A, const SimplicialComplex& B);

/**
 * @brief get the boundary of a simplex
 */
SimplicialComplex boundary(const Simplex& s, const Mesh& m);

/**
 * @brief get complex of a simplex and its boundary
 */
SimplicialComplex simplex_with_boundary(const Simplex& s, const Mesh& m);

/**
 * @brief check if simplices with their boundary intersect
 */
bool simplices_w_boundary_intersect(const Simplex& s1, const Simplex& s2, const Mesh& m);

SimplicialComplex closed_star(const Simplex& s, const Mesh& m);

SimplicialComplex link(const Simplex& s, const Mesh& m);

SimplicialComplex open_star(const Simplex& s, const Mesh& m);

//////////////////////////////////
// check link condition
// input Tuple t --> edge (a,b)
// check if lnk(a) ∩ lnk(b) == lnk(ab)
//////////////////////////////////
bool link_cond(Tuple t, const Mesh& m);

//////////////////////////////////
// k-ring
//////////////////////////////////

/**
 * @brief get one ring neighbors of vertex in _t_
 */
std::vector<Simplex> vertex_one_ring(Tuple t, const Mesh& m);

std::vector<Simplex> k_ring(Tuple t, const Mesh& m, int k);

} // namespace wmtk