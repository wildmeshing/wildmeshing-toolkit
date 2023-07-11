#pragma once

#include <queue>
#include <set>
#include <vector>
#include "Mesh.hpp"
#include "Simplex.hpp"

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
    internal::SimplexLessFunctor _slf;
    internal::SimplexSet _simplices;

public:
    const internal::SimplexSet& get_simplices() const { return _simplices; }

    internal::SimplexSet get_simplices(const PrimitiveType& ptype) const;

    std::vector<Simplex> get_simplex_vector() const;

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
        : _slf(mm)
        , _simplices(_slf)
    {}

    SimplicialComplex(const internal::SimplexLessFunctor& slf)
        : _slf(slf)
        , _simplices(_slf)
    {}

    SimplicialComplex(const std::vector<Simplex>& ss, const Mesh& mm)
        : _slf(mm)
        , _simplices(_slf)
    {
        for (const Simplex& s : ss) {
            add_simplex(s);
        }
    }

    auto key_comp() const { return _simplices.key_comp(); }


    static SimplicialComplex get_union(const SimplicialComplex& sc1, const SimplicialComplex& sc2);

    static SimplicialComplex get_intersection(
        const SimplicialComplex& A,
        const SimplicialComplex& B);

    /**
     * @brief get the boundary of a simplex
     */
    static SimplicialComplex boundary(const Simplex& s, const Mesh& m);

    /**
     * @brief get complex of a simplex and its boundary
     */
    static SimplicialComplex simplex_with_boundary(const Simplex& s, const Mesh& m);

    /**
     * @brief check if simplices with their boundary intersect
     */
    static bool simplices_w_boundary_intersect(const Simplex& s1, const Simplex& s2, const Mesh& m);

    static SimplicialComplex closed_star(const Simplex& s, const Mesh& m);

    static SimplicialComplex link(const Simplex& s, const Mesh& m);

    static SimplicialComplex open_star(const Simplex& s, const Mesh& m);

    //////////////////////////////////
    // check link condition
    // input Tuple t --> edge (a,b)
    // check if lnk(a) ∩ lnk(b) == lnk(ab)
    //////////////////////////////////
    static bool link_cond(Tuple t, const Mesh& m);
    static bool link_cond_bd_2d(Tuple t, const Mesh& m);

    //////////////////////////////////
    // k-ring
    //////////////////////////////////

    /**
     * @brief get one ring neighbors of vertex in _t_
     */
    static std::vector<Simplex> vertex_one_ring(Tuple t, const Mesh& m);

    static std::vector<Simplex> k_ring(Tuple t, const Mesh& m, int k);
};

} // namespace wmtk