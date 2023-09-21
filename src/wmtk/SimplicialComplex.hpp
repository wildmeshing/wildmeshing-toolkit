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
    const Mesh* m;

    SimplexLessFunctor(const Mesh& mm)
        : m(&mm)
    {}

    bool operator()(const Simplex& s0, const Simplex& s1) const
    {
        return m->simplex_is_less(s0, s1);
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

    SimplicialComplex& operator=(const SimplicialComplex&) = default;

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
     * @brief Get the boundary of a simplex.
     *
     * The boundary of a simplex are all incident lower dimensional simplices.
     * - Tetrahedron: 4 faces, 6 edges, 4 vertices
     * - Triange: 3 edges, 3 vertices
     * - Edge: 2 vertices
     * - Vertex: none
     *
     */
    static SimplicialComplex boundary(const Mesh& m, const Simplex& s);

    /**
     * @brief the union of a simplex and its boundary
     */
    static SimplicialComplex simplex_with_boundary(const Mesh& m, const Simplex& s);

    /**
     * @brief check if the intersection of simplices with their boundary is an empty set
     */
    static bool simplices_w_boundary_intersect(const Mesh& m, const Simplex& s1, const Simplex& s2);
    
    /**
     * @brief get all the the top_simplex of m which is a coface of Simplex s, this function can be used in computing closed_star and open_star
     */
    static SimplicialComplex top_coface_simplex(const Mesh& m, const Simplex& s);
    /**
     * @brief The union of all simplices with boundary that have s in their boundary.
     *
     * Example: The closed star of a vertex in a triangular mesh contains all triangles incident to
     * the vertex and all vertices and edges incident to those triangles.
     */
    static SimplicialComplex closed_star(const Mesh& m, const Simplex& s);

    /**
     * @brief The boundary of the closed star.
     *
     * Example: The link of a vertex in a triangle mesh is the ring of edges and vertices
     * surrounding it.
     */
    static SimplicialComplex link(const Mesh& m, const Simplex& s);

    /**
     * @brief The closed star without its boundary.
     *
     * For performance reasons, `closed_star` should be used whenever possible.
     */
    static SimplicialComplex open_star(const Mesh& m, const Simplex& s);

    //////////////////////////////////
    // check link condition
    // input Tuple t --> edge (a,b)
    // check if lnk(a) ∩ lnk(b) == lnk(ab)
    //////////////////////////////////
    static bool link_cond(const Mesh& m, Tuple t);
    static bool link_cond_bd_2d(const Mesh& m, Tuple t);

    // could be a replacement for link_cond_bd_2d
    static bool edge_collapse_possible_2d(const TriMesh& m, const Tuple& t);
    //////////////////////////////////
    // k-ring
    //////////////////////////////////

    /**
     * @brief get one ring vertex neighbors of vertex in _t_
     *
     * The vertex one ring does not include the vertex of the tuple itself.
     */
    static std::vector<Simplex> vertex_one_ring(const Mesh& m, Tuple t);

    /**
     * @brief get one ring vertex neighbors of vertex in _t_
     *
     * The vertex one ring does not include the vertex of the tuple itself.
     */
    static std::vector<Simplex> vertex_one_ring(const TriMesh& m, Tuple t);

    static std::vector<Simplex> k_ring(const Mesh& m, Tuple t, int k);
};

} // namespace wmtk