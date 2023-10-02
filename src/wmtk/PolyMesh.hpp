#pragma once

#include <Eigen/Core>

/**
 * Proposed Implementation of the General Polygonal Mesh Data Structure
 *
 * A general polygonal mesh is represented using tuples of a consistently oriented face, edge,
 * and vertex together with orientation-reversing switch vertex, switch edge, and switch face maps.
 *
 * Using the tuple structure, the global cid is a halfedge (equivalently, an edge with a normal
 * direction), and a local vid. The vertex index can be interpreted as the tip of the edge and is
 * thus equivalent to specifying a vertex of the edge as well as a direction, which together with
 * the normal direction specifies an orientation of the face. The local eid and local fid are both
 * set to default -1.
 *
 * The implementation is done using the standard halfedge operations, next and previous halfedge,
 * encoded as pairs of edge attributes (two per edge, one for each halfedge). Opp is inferred from
 * the pairing of halfedges into edges. The switch vertex, switch edge, and switch face operations
 * can then be implemented as simple combinations of next, prev, and opp along with the additional
 * operation of switching the vertex of the halfedge. Switching the vertex corresponds to switching
 * the orientation of the surface, so next becomes prev and vice versa.
 *
 * Note that halfedge only supports oriented surfaces. However, the oriented halfedge could easily
 * be extended to support nonorientable surfaces by adding an additional edge attributes to indicate
 * whether the two adjacent faces should be glued with consistent or anti-consistent orientation.
 *
 * The choice of halfedges directed with the local vertex index is natural for the given tuple
 * interface as the halfedge is a 1-simplex in the given face, and using halfedge allows for some
 * theoretical simplifications as well as more compact memory usage.
 *
 * Also note that the natural generalization of this implementation is the facet-edge data
 * structure of Dobkin and Laszlo, where the primitive is an oriented face-edge
 * triple. However, while the halfedge can be encoded as an edge attribute, the facet-edge
 * data needs to be encoded on actual pairs of faces and edges as each face can be incident
 * to many edges and vice versa.
 */

/**
 * Helper Functions
 *
 * TODO: These can be put in another file, e.g., polymesh_topology_initialization, to conform
 * with the code organization used for trimesh and tetmesh. Or they can be made static member
 * functions. Or they could be defined only in the cpp file.
 */

/**
 * Convert from matrix (F) mesh representation to NOB data structure
 *
 * @param F: #f*n, each row represents the vertex id (ccw) of current face
 * @return next_he: (N) size #h vector, next halfedge id
 * @return opp: (O) size #h vector, opposite halfedge id
 * @return bnd_loops: (B) collection of boundary face ids.
 */
std::tuple<VectorXl, VectorXl, VectorXl> fv_to_nob(Eigen::Ref<const RowVectors3l> F);

/**
 * Extend next_he and opp to add extra halfedges along the boundaries.
 *
 * @param next_he: next-halfedge map same length as opp
 * @param opp: halfedge map, for boundary halfedges -1
 * @return next_he_ext: next_halfedge map, same length as opp_ext; newly added halfedges are linked
 *                      into boundary loops
 * @return opp_ext: opp with an extra coupled halfedge added at the end for each boundary halfedge
 *                  all halfedges have a pair
 */
std::tuple<VectorXl, VectorXl> build_boundary_loops(
    Eigen::Ref<const VectorXl> next_he,
    Eigen::Ref<const VectorXl> opp);

/**
 * Build orbits following next id recorded in perm.
 *
 * @param perm: a permutation given by a list of non-repeating integers in the range 0..len(perm)
 * @return cycles: a list of lists, each list represents a cycle of perm
 */
std::vector<std::vector<int>> build_orbits(const std::vector<int>& perm);

/**
 * End of Helper Functions
 */

namespace wmtk {


class PolyMesh : public Mesh
{
public:
    PolyMesh();
    PolyMesh(const PolyMesh& o);
    PolyMesh(PolyMesh&& o);
    PolyMesh& operator=(const PolyMesh& o);
    PolyMesh& operator=(PolyMesh&& o);

    // TODO This only makes sense for simplices as currently named
    // It is also potentially ambiguous as our top type is a face, but our basic mesh primitive
    // is a halfedge
    PrimitiveType top_simplex_type() const override { return PrimitiveType::Face; }

    /**
     * @brief Primitive for changing connectivity without changing the number of edges.
     * 
     * Requires t and s to have consistent orientation
     */
    Tuple splice(const Tuple& t, const Tuple& s);

    /**
     * @brief Primitive for adding a bubble component with a single edge.
     */
    Tuple make_edge();

    /**
     * @brief Primitive for removing a bubble component with a single edge.
     */
    Tuple delete_bubble(const Tuple& t);

    /**
     * @brief Split the common face of t and s with the new edge between their respective
     * vertices (with ccw orientation)
     * 
     * Requires t and s to share a face but not a vertex and have consistent orientation
     */
    Tuple split_face(const Tuple& t, const Tuple& s);

    /**
     * @brief Join the two faces across the edge of t
     * 
     * Requires the edge of t to be adjacent to two distinct faces and have consistent orientation
     */
    Tuple join_face(const Tuple& t, const Tuple& s);

    /**
     * @brief Split the common vertex of t and s with the new edge between the merge of their
     * respective faces
     * 
     * Requires t and s to share a vertex but not a face and have consistent orientation
     */
    Tuple split_vertex(const Tuple& t, const Tuple& s);

    /**
     * @brief Join the two endpoints the edge of t
     * 
     * Requires at least one endpoint of h to have valence > 1, and if vertices are distinct cannot
     * be both boundary, unless the edge is boundary,
     */
    Tuple join_vertex(const Tuple& t);

    /**
     * @brief Mark a face as a hole in the surface.
     */
    Tuple make_hole(const Tuple& t);

    /**
     * @brief Mark a face as not a hole in the surface.
     */
    Tuple fill_hole(const Tuple& t);

    // TODO These only make sense for triangle meshes, but we do want them in the interface
    // since they can be implemented using splice and bubble operations
    Tuple split_edge(const Tuple& t, Accessor<long>& hash_accessor) override;
    Tuple collapse_edge(const Tuple& t, Accessor<long>& hash_accessor) override;
    Tuple swap_edge(const Tuple& t);

    Tuple switch_tuple(const Tuple& tuple, PrimitiveType type) const override;

    /**
     * @brief Jump to the next halfedge by performing a switch of vertex and edge
     */
    Tuple next_halfedge(const Tuple& tuple) const;

    /**
     * @brief Jump to the previous halfedge by performing a switch of edge and vertex
     */
    Tuple prev_halfedge(const Tuple& tuple) const;

    /**
     * @brief Jump to the opposite halfedge by performing a switch of face and vertex
     */
    Tuple opp_halfedge(const Tuple& tuple) const;

    bool is_connectivity_valid() const override;

    Tuple tuple_from_id(const PrimitiveType type, const long gid) const override;

    bool is_ccw(const Tuple& tuple) const override;
    bool is_boundary(const Tuple& tuple) const override;
    bool is_boundary_vertex(const Tuple& tuple) const override;
    bool is_boundary_edge(const Tuple& tuple) const override;

    bool is_valid(const Tuple& tuple, ConstAccessor<long>& hash_accessor) const override;

    void initialize(
        Eigen::Ref<const VectorXl> next,
        Eigen::Ref<const VectorXl> opp,
        Eigen::Ref<const VectorXl> bnd_loops);
    void initialize(Eigen::Ref<const RowVectors3l> F);

protected:
    long id(const Tuple& tuple, PrimitiveType type) const override;

private:
    /**
     * @brief Generate a new vertex handle
     * 
     * TODO: Check if should be implemented using request_simplex_indices
     */
    long new_vertex();

    /**
     * @brief Generate a clone of an existing vertex handle
     * 
     * TODO: Check if this is necessary if handles are just indices
     */
    long clone_vertex(long v);

    /**
     * @brief Delete the vertex with the given id
     */
    void delete_vertex(long v);

    /**
     * @brief Set the id of the vertex corresponding to the given tuple to v
     */
    void set_vertex(const Tuple& tuple, long v);

    /**
     * @brief Generate a new face handle
     */
    long new_face();

    /**
     * @brief Generate a clone of an existing face handle
     */
    long clone_face(long f);

    /**
     * @brief Delete the face with the given id
     */
    void delete_face(long f);

    /**
     * @brief Set the id of the face corresponding to the given tuple to f
     */
    void set_face(const Tuple& tuple, long f);

    /**
     * @brief Generate a new edge handle
     */
    long new_edge();

protected:
    attribute::MeshAttributeHandle<long> m_hn_handle;
    attribute::MeshAttributeHandle<long> m_hp_handle;

    attribute::MeshAttributeHandle<long> m_hv_handle;
    attribute::MeshAttributeHandle<long> m_hf_handle;

    attribute::MeshAttributeHandle<long> m_fh_handle;
    attribute::MeshAttributeHandle<long> m_vh_handle;

    attribute::MeshAttributeHandle<bool> m_f_is_hole_handle;
};

} // namespace wmtk
