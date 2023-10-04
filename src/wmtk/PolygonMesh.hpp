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
 * TODO: These can be put in another file, e.g., polygon_mesh_topology_initialization, to conform
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


class PolygonMesh : public Mesh
{
public:
    PolygonMesh();
    PolygonMesh(const PolygonMesh& o);
    PolygonMesh(PolygonMesh&& o);
    PolygonMesh& operator=(const PolygonMesh& o);
    PolygonMesh& operator=(PolygonMesh&& o);

    // TODO This only makes sense for simplices as currently named
    // It is also potentially ambiguous as our top type is a face, but our basic mesh primitive
    // is a halfedge
    PrimitiveType top_simplex_type() const override { return PrimitiveType::Face; }


    // TODO These only make sense for triangle meshes, but we do want them in the interface
    // since they can be implemented using splice and bubble operations
    Tuple split_edge(const Tuple& t, Accessor<long>& hash_accessor) override;
    Tuple collapse_edge(const Tuple& t, Accessor<long>& hash_accessor) override;

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
