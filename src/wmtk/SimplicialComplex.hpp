#pragma once

#include <queue>
#include <set>
#include <vector>
#include "Mesh.hpp"
#include "Simplex.hpp"
#include "simplex/internal/SimplexLessFunctor.hpp"

#if defined(ENABLE_WMTK_WARN_SIMPLICIAL_COMPLEX_DEPRECATION)
#define WMTK_WARN_SIMPLICIAL_COMPLEX_DEPRECATION [[deprecated("Use functions and classes provided in <wmtk/simplex/...> instead")]]
#else
#define WMTK_WARN_SIMPLICIAL_COMPLEX_DEPRECATION 
#endif
namespace wmtk {

namespace internal {


using SimplexLessFunctor = wmtk::simplex::internal::SimplexLessFunctor;

using SimplexSet = std::set<Simplex, SimplexLessFunctor>;

} // namespace internal

// DEPRECATED
class SimplicialComplex
{
private:
    internal::SimplexLessFunctor _slf;
    internal::SimplexSet _simplices;

public:
    const internal::SimplexSet& get_simplices() const { return _simplices; }

    internal::SimplexSet get_simplices(const PrimitiveType& ptype) const;

    internal::SimplexSet get_vertices() const { return get_simplices(PrimitiveType::Vertex); }
    internal::SimplexSet get_edges() const { return get_simplices(PrimitiveType::Edge); }
    internal::SimplexSet get_faces() const { return get_simplices(PrimitiveType::Face); }
    internal::SimplexSet get_tetrahedra() const
    {
        return get_simplices(PrimitiveType::Tetrahedron);
    }

    std::vector<Simplex> get_simplex_vector() const;

    /**
     * @brief Add simplex to the complex if it is not already in it.
     *
     * @returns false if simplex is already in the complex
     */
    bool add_simplex(const Simplex& s);

    void unify_with_complex(const SimplicialComplex& other);

    bool operator==(const SimplicialComplex& other) const;

    // SimplicialComplex& operator=(const SimplicialComplex&) = default;

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

    WMTK_WARN_SIMPLICIAL_COMPLEX_DEPRECATION
        static SimplicialComplex
    get_union(const SimplicialComplex& sc1, const SimplicialComplex& sc2);

    WMTK_WARN_SIMPLICIAL_COMPLEX_DEPRECATION
        static SimplicialComplex
    get_intersection(const SimplicialComplex& A, const SimplicialComplex& B);

    /**
     * @brief DEPRECATED Get the boundary of a simplex.
     *
     * The boundary of a simplex are all incident lower dimensional simplices.
     * - Tetrahedron: 4 faces, 6 edges, 4 vertices
     * - Triange: 3 edges, 3 vertices
     * - Edge: 2 vertices
     * - Vertex: none
     *
     */
    WMTK_WARN_SIMPLICIAL_COMPLEX_DEPRECATION
                 static SimplicialComplex
    boundary(const Mesh& m, const Simplex& s);

    /**
     * @brief DEPRECATED the union of a simplex and its boundary
     */
    WMTK_WARN_SIMPLICIAL_COMPLEX_DEPRECATION
                 static SimplicialComplex
    simplex_with_boundary(const Mesh& m, const Simplex& s);

    /**
     * @brief DEPRECATED check if the intersection of simplices with their boundary is an empty set
     */
    WMTK_WARN_SIMPLICIAL_COMPLEX_DEPRECATION
        static bool
    simplices_w_boundary_intersect(const Mesh& m, const Simplex& s1, const Simplex& s2);

    /**
     * @brief DEPRECATED get all the the top_simplex of m which is a coface of Simplex s, this
     * function can be used in computing closed_star and open_star
     */
    WMTK_WARN_SIMPLICIAL_COMPLEX_DEPRECATION
        static SimplicialComplex
    top_coface_simplex(const Mesh& m, const Simplex& s);
    /**
     * @brief DEPRECATED The union of all simplices with boundary that have s in their boundary.
     *
     * Example: The closed star of a vertex in a triangular mesh contains all triangles incident to
     * the vertex and all vertices and edges incident to those triangles.
     */
    WMTK_WARN_SIMPLICIAL_COMPLEX_DEPRECATION
        static SimplicialComplex
    closed_star(const Mesh& m, const Simplex& s);

    /**
     * @brief DEPRECATED The boundary of the closed star.
     *
     * Example: The link of a vertex in a triangle mesh is the ring of edges and vertices
     * surrounding it.
     */
    WMTK_WARN_SIMPLICIAL_COMPLEX_DEPRECATION
        static SimplicialComplex
    link(const Mesh& m, const Simplex& s);

    /**
     * @brief DEPRECATED The closed star without its boundary.
     *
     * For performance reasons, `closed_star` should be used whenever possible.
     */
    WMTK_WARN_SIMPLICIAL_COMPLEX_DEPRECATION
        static SimplicialComplex
    open_star(const Mesh& m, const Simplex& s);
    static SimplicialComplex open_star(const TriMesh& m, const Simplex& s);

    //////////////////////////////////
    // check link condition
    // input Tuple t --> edge (a,b)
    // check if lnk(a) ∩ lnk(b) == lnk(ab)
    //////////////////////////////////
    WMTK_WARN_SIMPLICIAL_COMPLEX_DEPRECATION
    static bool
    link_cond(const Mesh& m, Tuple t);
    static bool link_cond_bd_2d(const Mesh& m, Tuple t);

    //////////////////////////////////
    // link condition for edge mesh
    // input Tuple t ---> edge (a,b)
    // check if edge(a,b) is the only edge in EdgeMesh m
    //////////////////////////////////
    WMTK_WARN_SIMPLICIAL_COMPLEX_DEPRECATION
    static bool
    link_cond_bd_1d(const Mesh& m, Tuple t);

    // could be a replacement for link_cond_bd_2d
    WMTK_WARN_SIMPLICIAL_COMPLEX_DEPRECATION
    static bool
    edge_collapse_possible_2d(const TriMesh& m, const Tuple& t);
    //////////////////////////////////
    // k-ring
    //////////////////////////////////

    /**
     * @brief DEPRECATED get one ring vertex neighbors of vertex in _t_
     *
     * The vertex one ring does not include the vertex of the tuple itself.
     */
    WMTK_WARN_SIMPLICIAL_COMPLEX_DEPRECATION
        static std::vector<Simplex>
        vertex_one_ring(const Mesh& m, Tuple t);

    /**
     * @brief DEPRECATED get one ring vertex neighbors of vertex in _t_
     *
     * The vertex one ring does not include the vertex of the tuple itself.
     */
    WMTK_WARN_SIMPLICIAL_COMPLEX_DEPRECATION
        static std::vector<Simplex>
        vertex_one_ring(const TriMesh& m, Tuple t);

    static std::vector<Simplex> k_ring(const Mesh& m, Tuple t, int k);
};

} // namespace wmtk
