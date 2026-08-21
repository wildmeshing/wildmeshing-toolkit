#pragma once

#include <wmtk/utils/VectorUtils.h>
#include <wmtk/AttributeCollection.hpp>
#include <wmtk/SlotPool.hpp>
#include <wmtk/Types.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/simplex/SimplexCollection.hpp>
#include <wmtk/threading/enumerable_thread_specific.hpp>
#include <wmtk/threading/vertex_mutex.hpp>
#include <wmtk/utils/Logger.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <vector>

namespace wmtk {

class TriMesh
{
public:
    // Cell Tuple Navigator
    class Tuple
    {
    private:
        size_t m_vid = -1;
        size_t m_eid = -1;
        size_t m_fid = -1;
        size_t m_hash = 0;

        void update_hash(const TriMesh& m);

    public:
        void print_info();

        //         v2        /
        //       /    \      /
        //  e1  /      \  e0 /
        //     v0 - - - v1   /
        //         e2        /
        /**
         * Construct a new Tuple object with global vertex/triangle index and local edge index
         *
         * @param vid vertex id
         * @param eid edge id (local)
         * @param fid face id
         * @note edge ordering
         */
        Tuple() {}
        Tuple(size_t vid, size_t eid, size_t fid, const TriMesh& m)
            : m_vid(vid)
            , m_eid(eid)
            , m_fid(fid)
        {
            update_hash(m);
        }


        /**
         * returns global vertex id.
         * @param m TriMesh where the tuple belongs.
         * @return size_t
         */
        inline size_t vid(const TriMesh&) const { return m_vid; }

        /**
         * returns a global unique face id
         *
         * @param m TriMesh where the tuple belongs.
         * @return size_t
         */
        inline size_t fid(const TriMesh&) const { return m_fid; }


        /**
         * returns a global unique edge id
         *
         * @param m TriMesh where the tuple belongs.
         * @return size_t
         * @note The global id may not be consecutive. The edges are undirected and different tetra
         * share the same edge.
         */
        size_t eid(const TriMesh& m) const;

        /**
         * returns the local eid of the tuple
         *
         * @param m TriMesh where the tuple belongs.
         * @return size_t
         * @note use mostly for constructing consistent tuples in operations
         */
        size_t local_eid(const TriMesh& m) const { return m_eid; };
        /**
         * Switch operation.
         *
         * @param m Mesh
         * @return another Tuple that share the same face, edge, but different vertex.
         */
        Tuple switch_vertex(const TriMesh& m) const;
        /**
         *
         * @param m
         * @return another Tuple that share the same face, vertex, but different edge.
         */
        Tuple switch_edge(const TriMesh& m) const;
        /**
         * Step to the next triangle around this edge.
         *
         * The faces incident to an edge are ordered by fid, and this advances one place
         * along that order, wrapping exactly once. On a manifold edge that is the triangle
         * on the other side, which is what this has always returned. On a non-manifold
         * edge, applying it edge_valence() times is the identity; it no longer picks an
         * arbitrary fan member, as it did when the fan was assumed to hold at most two
         * faces. Callers that genuinely need the manifold case -- a swap, say -- should
         * test is_manifold_edge() rather than assume it.
         *
         * @param m Mesh
         * @return Tuple for the next triangle, sharing the same edge and vertex.
         * @note nullopt on a boundary edge, which has nowhere to step to.
         */
        std::optional<Tuple> switch_face(const TriMesh& m) const;

        /**
         * Every other triangle around this edge, in increasing face id order.
         *
         * Empty on a boundary edge, one entry when manifold.
         */
        std::vector<Tuple> switch_faces(const TriMesh& m) const;

        /**
         * @brief check if a Tuple is valid
         *
         * @param m the Mesh
         * @return false if 1. the fid of the Tuple is -1, 2. either the vertex or the face
         * refered to by the Tuple is removed, 3. the hash of the Tuple is not the same as
         * the hash of the triangle it refers to in the mesh
         *
         */
        bool is_valid(const TriMesh& m) const;

        /**
         * Positively oriented 3 vertices (represented by Tuples) in a tri.
         * @return std::array<Tuple, 3> each tuple owns a different vertex.
         */
        std::array<Tuple, 3> oriented_tri_vertices(const TriMesh& m) const;
        friend bool operator<(const Tuple& a, const Tuple& t)
        {
            return (
                std::tie(a.m_vid, a.m_eid, a.m_fid, a.m_hash) <
                std::tie(t.m_vid, t.m_eid, t.m_fid, t.m_hash));
        }
    };

    class SmartTuple
    {
        const TriMesh& m_mesh;
        Tuple m_tuple;

    public:
        SmartTuple(const TriMesh& mesh, const Tuple& t)
            : m_mesh(mesh)
            , m_tuple(t)
        {}

        const Tuple& tuple() const { return m_tuple; }
        const TriMesh& mesh() const { return m_mesh; }

        SmartTuple& operator=(const SmartTuple& t)
        {
            m_tuple = t.m_tuple;
            return *this;
        }

        bool is_valid() const { return m_tuple.is_valid(m_mesh); }
        size_t vid() const { return m_tuple.vid(m_mesh); }
        size_t eid() const { return m_tuple.eid(m_mesh); }
        size_t fid() const { return m_tuple.fid(m_mesh); }
        SmartTuple switch_vertex() const { return {m_mesh, m_tuple.switch_vertex(m_mesh)}; }
        SmartTuple switch_edge() const { return {m_mesh, m_tuple.switch_edge(m_mesh)}; }
        std::optional<SmartTuple> switch_face() const
        {
            const std::optional<Tuple> t = m_tuple.switch_face(m_mesh);
            if (t) {
                return std::optional<SmartTuple>({m_mesh, t.value()});
            }
            return {};
        }
    };

    /**
     * (internal use) Maintains a list of triangles connected to the given vertex, and a flag to
     * mark removal.
     *
     */
    class VertexConnectivity
    {
    public:
        /**
         * @brief incident triangles of a given vertex
         *
         */
        std::vector<size_t> m_conn_tris;
        /**
         * @brief is the vertex removed
         *
         */
        bool m_is_removed = false;

        inline size_t& operator[](const size_t index)
        {
            assert(index < m_conn_tris.size());
            return m_conn_tris[index];
        }

        inline size_t operator[](const size_t index) const
        {
            assert(index < m_conn_tris.size());
            return m_conn_tris[index];
        }
    };

    /**
     * (internal use) Maintains a list of vertices of the given triangle
     *
     */
    class TriangleConnectivity
    {
    public:
        /**
         * @brief incident vertices of a given triangle
         *
         */
        std::array<size_t, 3> m_indices;
        /**
         * @brief is the triangle removed
         *
         */
        bool m_is_removed = false;
        /**
         * @brief the hash is changed every time there is an operation that influences the
         * triangle
         *
         */
        size_t hash = 0;

        inline size_t& operator[](size_t index)
        {
            assert(index < 3);
            return m_indices[index];
        }

        inline size_t operator[](size_t index) const
        {
            assert(index < 3);
            return m_indices[index];
        }
        /**
         * @param vid global
         * @return local vid of the vertex given the triangle
         * \n -1 if the vertex is not incident to the triangle
         */
        inline int find(size_t v_id) const
        {
            for (int j = 0; j < 3; j++) {
                if (v_id == m_indices[j]) {
                    return j;
                }
            }
            return -1;
        }
    };

    TriMesh() {}
    virtual ~TriMesh() {}

    /**
     * Generate the connectivity of the mesh
     * @param n_vertices Input number of vertices
     * @param tris triangle connectivity
     */
    void init(size_t n_vertices, const std::vector<std::array<size_t, 3>>& tris);

    /**
     * @brief Generate the connectivity of the mesh from an IGL-style F matrix.
     *
     * @param #F by 3 list of vertex indices.
     */
    void init(const MatrixXi& F);

    /**
     * Generate a vector of Tuples from global vertex index and __local__ edge index
     * @note Each vertex generate Tuple that has the smallest fid to be among
     * incident triangles'.
     * Local vid to be in the same order as thier indices
     * in the m_conn_tris.
     * Local eid assigned counter clockwise as in the ilustrated
     * example
     * @return vector of Tuples refering to each vertex
     */
    std::vector<Tuple> get_vertices() const;

    /**
     * Generate a vector of Tuples for each edge
     * @note ensures the fid assigned is the smallest between faces adjacent to the
     * edge
     * @return vector of Tuples refering to unique edges
     */
    std::vector<Tuple> get_edges() const;

    /**
     * Generate a vector of Tuples from global face index
     * @note Local vid is the first of the m_idices
     * Local eid assigned counter clockwise as in the ilustrated example
     * @return vector of Tuples refering to each face
     */
    std::vector<Tuple> get_faces() const;

    /**
     * Generate a tuple using local vid and global fid
     * @param vid1, vid2 are local vids TODO: these are global vids
     * @param fid globale fid for the triangle
     * @note tuple refers to vid1
     * @return vector of Tuples
     */
    Tuple tuple_from_edge(size_t vid1, size_t vid2, size_t fid) const;

    Tuple tuple_from_vids(size_t vid0, size_t vid1, size_t vid2) const;

    simplex::Vertex simplex_from_vertex(const Tuple& t) const;
    simplex::Edge simplex_from_edge(const Tuple& t) const;
    simplex::Face simplex_from_face(const Tuple& t) const;
    simplex::Face simplex_from_face(const size_t fid) const;

    Tuple tuple_from_simplex(const simplex::Face& s) const;
    // Tuple tuple_from_simplex(const simplex::Edge& s) const;

    simplex::SimplexCollection simplex_incident_triangles(const simplex::Vertex& v) const;
    simplex::SimplexCollection simplex_incident_triangles(const simplex::Edge& e) const;
    simplex::SimplexCollection simplex_link_vertices(const simplex::Vertex& v) const;
    simplex::SimplexCollection simplex_link_vertices(const simplex::Edge& e) const;
    simplex::SimplexCollection simplex_link_edges(const simplex::Vertex& v) const;

    template <typename T>
    using vector = std::vector<T>;

public:
    AbstractAttributeContainer* p_vertex_attrs = nullptr;
    AbstractAttributeContainer* p_edge_attrs = nullptr;
    AbstractAttributeContainer* p_face_attrs = nullptr;

public:
    /**
     * @brief Preallocation factor: init/consolidate reserve capacity =
     * max(floor, ceil(factor * live_count)) so operations can grab fresh slots
     * without resizing the storage. When a pass exhausts the reserved capacity the
     * affected operations fail (retried later after a consolidate). Values < 1 are
     * clamped to 1.
     */
    void set_preallocation_factor(double factor)
    {
        if (factor >= 1.0) m_preallocation_factor = factor;
    }
    double preallocation_factor() const { return m_preallocation_factor; }

    // Atomically reserve `n` contiguous fresh triangle/vertex slots. Returns the
    // first index of the block, or INVALID_SLOT if that would exceed the preallocated
    // capacity (the caller must then abort the operation before mutating).
    size_t request_tri_slots(size_t n);
    size_t request_vert_slots(size_t n);

private:
    size_t reserved_capacity(size_t live_count) const
    {
        const size_t floor = 64;
        double c = std::ceil(m_preallocation_factor * static_cast<double>(live_count));
        size_t capacity = static_cast<size_t>(c);
        if (capacity < live_count) capacity = live_count;
        return capacity < floor ? floor : capacity;
    }

    SlotPool<VertexConnectivity> m_vertex_connectivity;
    SlotPool<TriangleConnectivity> m_tri_connectivity;
    double m_preallocation_factor = 6.0;
    bool m_use_link_condition = true;

protected:
    /**
     * Edge-connected components of the fan of `vid`, as positions into that fan.
     *
     * `component_of[i]` is the index into `representatives` of the component containing
     * `m_vertex_connectivity[vid].m_conn_tris[i]`, and `representatives` holds each
     * component's smallest fid in increasing order. Two faces are in the same component
     * when they share an edge containing `vid`.
     *
     * Both outputs are sized by the fan rather than by the mesh, and nothing is cached:
     * the fan is already the whole input, so recomputing costs the same as reading a
     * stored answer would, minus the obligation to keep it current.
     */
    void vertex_fan_components(
        size_t vid,
        std::vector<size_t>& component_of,
        std::vector<size_t>& representatives) const;

private:
    /**
     * @brief Get the next avaiblie global index for the triangle
     *
     * @return size_t
     */
    size_t get_next_empty_slot_t();
    /**
     * @brief Get the next avaiblie global index for the vertex
     *
     * @return size_t
     */
    size_t get_next_empty_slot_v();

public:
    /**
     * @brief User specified invariants that can't be violated
     * @param std::vector<Tuple> a vector of Tuples that are concerned in a given operation
     * @return true if the invairnats are not violated
     */
    virtual bool invariants(const std::vector<Tuple>&) { return true; }
    /**
     * @brief User specified preparations and desideratas for an edge split
     * @param the edge Tuple to be split
     * @return true if the preparation succeed
     */
    virtual bool split_edge_before(const Tuple& t) { return true; }
    /**
     * @brief User specified modifications and desideratas after an edge split
     * @param the edge Tuple to be split
     * @return true if the modifications succeed
     */
    virtual bool split_edge_after(const Tuple& t) { return true; }

    /**
     * @brief User specified preparations and desideratas for an edge collapse
     * including the link check as collapse prerequisite
     *
     * @param the edge Tuple to be split
     * @return true if the preparation succeed
     */
    virtual bool collapse_edge_before(const Tuple& t)
    {
        if (!m_use_link_condition) {
            return true;
        }
        return check_link_condition(t);
    }
    /**
     * @brief User specified modifications and desideratas after an edge collapse
     * @param the edge Tuple to be collapsed
     * @return true if the modifications succeed
     */
    virtual bool collapse_edge_after(const Tuple& t) { return true; }
    /**
     * @brief User specified modifications and desideras after an edge swap
     * @param the edge Tuple to be swaped
     * @return true if the modifications succeed
     */
    virtual bool swap_edge_after(const Tuple& t) { return true; }
    /**
     * @brief User specified preparations and desideratas for an edge swap
     * including 1.can't swap on boundary edge. 2. when swap edge between v1, v2,
     * there can't exist edges between the two opposite vertices v3, v4
     *
     * @param the edge Tuple to be swaped
     * @return true if the preparation succeed
     */
    virtual bool swap_edge_before(const Tuple& t);
    /**
     * @brief User specified preparations and desideratas for an edge smooth
     *
     * @param the edge Tuple to be smoothed
     * @return true if the preparation succeed
     */
    virtual bool smooth_before(const Tuple& t) { return true; }
    /**
     * @brief User specified modifications and desideras after an edge smooth
     * @param the edge Tuple to be smoothed
     * @return true if the modifications succeed
     */
    virtual bool smooth_after(const Tuple& t) { return true; }

    /**
     * @brief User specified preparations and desideratas for a face split
     * @param the face Tuple to be split
     * @return true if the preparation succeed
     */
    virtual bool split_face_before(const Tuple& t) { return true; }
    /**
     * @brief User specified modifications and desideratas after a face split
     * @param the face Tuple to be split
     * @return true if the modifications succeed
     */
    virtual bool split_face_after(const Tuple& t) { return true; }

public:
    /**
     * @brief get the current largest global fid
     *
     * @return size_t
     */
    size_t tri_capacity() const { return m_tri_connectivity.live(); }
    /**
     * @brief get the current largest global vid
     *
     * @return size_t
     */
    size_t vert_capacity() const { return m_vertex_connectivity.live(); }

    /**
     * @name Dimension-generic cell accessors
     *
     * A "cell" is the top-dimensional element: a triangle here, a tet in TetMesh. These
     * three members plus EDGES_PER_CELL are the whole interface the dimension-generic
     * helpers in wmtk/utils (ParallelCollect, SizingField) need, so the same helper works
     * on both meshes without traits or overloads.
     * @{
     */
    static constexpr int EDGES_PER_CELL = 3;
    size_t cell_capacity() const { return tri_capacity(); }
    Tuple tuple_from_cell(size_t cid) const { return tuple_from_tri(cid); }
    /** @} */
    /**
     * @brief removing the elements that are removed
     *
     * @param bnd_output when turn on will write the boundary vertices to "bdn_table.dmat"
     */
    void consolidate_mesh();

    /**
     * @brief Mark the given triangles, and any vertex left without an incident triangle,
     * as removed.
     *
     * The 2D counterpart of TetMesh::remove_tets_by_ids, used by the output filters to
     * drop the region outside the input. Call consolidate_mesh() afterwards to compact.
     */
    void remove_tris_by_ids(const std::vector<size_t>& fids)
    {
        for (const size_t fid : fids) {
            m_tri_connectivity[fid].m_is_removed = true;
            for (int j = 0; j < 3; j++) {
                vector_erase(m_vertex_connectivity[m_tri_connectivity[fid][j]].m_conn_tris, fid);
            }
        }
        for (auto& v : m_vertex_connectivity) {
            if (v.m_is_removed) continue;
            if (v.m_conn_tris.empty()) v.m_is_removed = true;
        }
    }
    /**
     * @brief a duplicate of Tuple::switch_vertex funciton
     */
    Tuple switch_vertex(const Tuple& t) const { return t.switch_vertex(*this); }
    /**
     * @brief a duplicate of Tuple::switch_edge funciton
     */
    Tuple switch_edge(const Tuple& t) const { return t.switch_edge(*this); }
    /**
     * @brief a duplicate of Tuple::switch_face funciton
     * @note Returns nullptr if current Tuple referes to a boundary triangle
     */
    std::optional<Tuple> switch_face(const Tuple& t) const { return t.switch_face(*this); }

    /**
     * @brief prerequisite for collapse
     * @param t Tuple referes to the edge to be collapsed
     * @returns true is the link check is passed
     */
    bool check_link_condition(const Tuple& t) const;

    /**
     * @brief Should collapse_edge_before enforce the link condition?
     *
     * The link condition guarantees a collapse preserves the homotopy type and keeps the
     * mesh a simplicial complex. Turning it off allows collapses that change topology and
     * that create non-manifold edges and vertices, which the data structure now
     * represents; the collapse itself still merges any duplicate triangles it produces so
     * the result stays a simplicial complex.
     *
     * Defaults to true, which is the historical behaviour of every caller.
     */
    void set_use_link_condition(bool use_it) { m_use_link_condition = use_it; }
    bool use_link_condition() const { return m_use_link_condition; }

    /**
     * @brief verify the connectivity validity of the mesh
     * @note a valid mesh can have triangles that are is_removed == true
     */
    bool check_mesh_connectivity_validity() const;
    /**
     * @brief verify the edge manifoldness of the mesh
     */
    bool check_edge_manifold() const;

    /**
     * @brief Number of triangles incident to the edge the Tuple points at.
     *
     * 1 on the boundary, 2 when manifold, more when not. O(valence).
     */
    size_t edge_valence(const TriMesh::Tuple& t) const;

    /**
     * @brief Does exactly one triangle share this edge?
     *
     * @param t Tuple refering to an edge
     */
    bool is_boundary_edge(const TriMesh::Tuple& t) const;

    /**
     * @brief Do exactly two triangles share this edge?
     *
     * Note that a boundary edge is not manifold by this definition; callers that need
     * "manifold or boundary" should test is_manifold_edge(t) || is_boundary_edge(t).
     *
     * Stops counting at three, so a pole with a large fan costs no more than a normal
     * edge. swap_edge_before() asks this of every candidate edge.
     */
    bool is_manifold_edge(const TriMesh::Tuple& t) const;

    /**
     * @brief Number of edge-connected components in the fan of a vertex.
     *
     * 1 for a manifold vertex (and for a vertex on a non-manifold edge, whose faces are
     * still joined through that edge), more for a pinch point. 0 for an isolated vertex.
     */
    size_t vertex_component_count(const size_t vid) const;
    size_t vertex_component_count(const TriMesh::Tuple& t) const
    {
        return vertex_component_count(t.vid(*this));
    }

    bool is_manifold_vertex(const size_t vid) const { return vertex_component_count(vid) <= 1; }

    /**
     * @brief Jump to the next edge-connected component of the fan of the Tuple's vertex.
     *
     * The returned Tuple points at the same vertex but at a face that no sequence of
     * switch_edge/switch_face can reach from `t`. Applying it once per component returns
     * to the starting component. Returns nullopt when the vertex is manifold, i.e. when
     * there is no other component to jump to.
     */
    std::optional<Tuple> switch_component(const TriMesh::Tuple& t) const;

#ifdef WMTK_DEBUG_BRUTE_FORCE_OPS
    /**
     * Cross-check of the incremental operations against a brute-force reference.
     *
     * Compiled only under -DWMTK_DEBUG_BRUTE_FORCE_OPS=ON. The reference rebuilds the whole
     * mesh as a plain list of vertex triples -- the OFF view of it -- applies the operation
     * there in the obvious way, and re-emits. split_edge and collapse_edge then compare
     * that string against their own result and throw on any difference.
     *
     * Two conventions have to match for the comparison to mean anything, and both are the
     * real operation's: a collapse keeps the *second* endpoint of the edge and retires the
     * first, and a duplicate group keeps its smallest fid.
     */

    /// Alive vids, then alive faces in fid order, each rotated to start at its smallest vid
    /// so that orientation survives but the arbitrary starting corner does not.
    std::string debug_canonical_form() const;

    /// The mesh as it would be after collapsing (v_removed, v_kept), as a canonical form.
    std::string debug_reference_collapse(size_t v_removed, size_t v_kept) const;

    /**
     * The mesh as it would be after swapping edge (v0,v1).
     *
     * `fa` is the face the operation's tuple sits on and `fb` the one across the edge. The
     * swap replaces v1 by fb's apex in fa, and v0 by fa's apex in fb, which is the pair of
     * triangles on the other diagonal.
     */
    std::string debug_reference_swap(size_t v0, size_t v1, size_t fa, size_t fb) const;

    /**
     * The mesh as it would be after splitting face `fid` at `new_v`.
     *
     * v0, v1 and v2 are the face's vertices in the operation's own order, which fixes which
     * child lands in which slot; `tri_cap` is tri_capacity() before the split reserved its
     * slots, for the same reason as debug_reference_split.
     */
    std::string debug_reference_split_face(
        size_t fid,
        size_t v0,
        size_t v1,
        size_t v2,
        size_t new_v,
        size_t tri_cap) const;

    /**
     * The mesh as it would be after splitting edge (v0,v1) with `new_v` in the middle.
     *
     * `tri_cap` is tri_capacity() as it was *before* the split reserved its new slots.
     * Those slots are live but unfilled at the point the caller needs the reference, so
     * reading them would put phantom (0,0,0) faces in the expected string.
     */
    std::string debug_reference_split(size_t v0, size_t v1, size_t new_v, size_t tri_cap) const;
#endif

    /**
     * @brief check if the vertex that's represented by a Tuple is at the boundary of the mesh
     *
     * @param t Tuple refering to an edge
     */
    bool is_boundary_vertex(const TriMesh::Tuple& t) const
    {
        auto ve = get_one_ring_edges_for_vertex(t);
        for (auto e : ve)
            if (is_boundary_edge(e)) return true;
        return false;
    }

    /**
     * Split an edge
     *
     * @param t Input Tuple for the edge to split.
     * @param[out] new_edges a vector of Tuples refering to the triangles incident to the new vertex
     * introduced
     * @return if split succeed
     */
    bool split_edge(const Tuple& t, std::vector<Tuple>& new_t);

    /**
     * Collapse an edge
     *
     * @param t Input Tuple for the edge to be collapsed.
     * @param[out] new_edges a vector of Tuples refering to the triangles incident to the new vertex
     * introduced
     * @note collapse edge a,b and generate a new vertex c
     * @return if collapse succeed
     */
    virtual bool collapse_edge(const Tuple& t, std::vector<Tuple>& new_t);

    /**
     * Collpase an edge connectivity part
     */
    void collapse_edge_conn(
        const Tuple& loc0,
        std::vector<Tuple>& new_tris,
        Tuple& return_t,
        size_t& new_vid,
        std::vector<std::pair<size_t, TriangleConnectivity>>& old_tris,
        std::vector<std::pair<size_t, VertexConnectivity>>& old_vertices,
        std::vector<std::pair<size_t, size_t>>& same_edge_vid_fid,
        std::vector<size_t>& n12_intersect_fids);

    /**
     * collapse edge rollback
     */

    void collapse_edge_rollback(
        size_t& new_vid,
        std::vector<std::pair<size_t, TriangleConnectivity>>& old_tris,
        std::vector<std::pair<size_t, VertexConnectivity>>& old_vertices,
        std::vector<std::pair<size_t, size_t>>& same_edge_vid_fid,
        std::vector<size_t>& n12_intersect_fids);


    /**
     * Swap an edge
     *
     * @param t Input Tuple for the edge to be swaped.
     * @param[out] new_edges a vector of Tuples refering to the triangles incident to the new
     * edge introduced
     * @note swap edge a,b to edge c,d
     * @return if swap succeed
     */
    bool swap_edge(const Tuple& t, std::vector<Tuple>& new_t);

    /**
     * Smooth a vertex
     *
     * @param t Input Tuple for the vertex
     * @note no geometry changed here
     * @return if smooth succeed
     */
    bool smooth_vertex(const Tuple& t);

    /**
     * @brief Split a face in 3 faces.
     *
     * @param t Input tuple for the face to split.
     * @param[out] new_t A vector of Tuples refering to the triangles incident to the new vertex.
     * introduced
     * @return true, if split succeed
     */
    bool split_face(const Tuple& t, std::vector<Tuple>& new_t);

    /**
     * @brief Count the number of the one ring tris for a vertex
     *
     * @param t tuple pointing to a vertex
     * @return one-ring tris number
     */
    size_t get_valence_for_vertex(const Tuple& t) const
    {
        return m_vertex_connectivity[t.vid(*this)].m_conn_tris.size();
    }

    /**
     * @brief Number of triangles incident to a vertex, by id.
     *
     * The same count as get_valence_for_vertex, for callers that hold a vid rather than a
     * Tuple. Around 6 on a well-shaped mesh; worth checking before anything that walks the
     * one ring, since a degenerate mesh can push it much higher.
     */
    size_t vertex_valence(const size_t vid) const
    {
        return m_vertex_connectivity[vid].m_conn_tris.size();
    }

    /**
     * @brief Get the one ring tris for a vertex
     *
     * @param t tuple pointing to a vertex
     * @return a vector of Tuples refering to one-ring tris
     */
    std::vector<Tuple> get_one_ring_tris_for_vertex(const Tuple& t) const;
    const std::vector<size_t>& get_one_ring_fids_for_vertex(const Tuple& t) const;
    const std::vector<size_t>& get_one_ring_fids_for_vertex(const size_t vid) const;
    /**
     * @brief Get the vids of the incident one ring tris for a vertex
     *
     * @param t tuple pointing to a vertex
     * @return a vector of vids that can have duplicates
     */
    std::vector<size_t> get_one_ring_vids_for_vertex_duplicate(const size_t& t) const;
    void get_one_ring_vids_for_vertex_duplicate(const size_t& t, std::vector<size_t>& one_ring)
        const;

    std::vector<size_t> get_incident_fids_for_edge(const Tuple& t) const;
    std::vector<size_t> get_incident_fids_for_edge(const size_t vid0, const size_t vid1) const;

    /**
     * @brief Get all edges that are incident to the vertex of Tuple `t`.
     *
     * The return tuples contain the edge and the adjacent vertex:
     *      return_tuple.switch_vertex().vid == t.vid()
     *
     * @param t tuple pointing to a vertex
     * @return one-ring
     */
    std::vector<Tuple> get_one_ring_edges_for_vertex(const Tuple& t) const;
    std::vector<Tuple> get_one_ring_edges_for_vertex(const size_t vid) const;

    /**
     * @brief Get the incident vertices for a triangle
     *
     * @param t tuple pointing to an face
     * @return tuples of incident vertices
     */
    std::array<Tuple, 3> oriented_tri_vertices(const Tuple& t) const;

    /**
     * @brief Get the incident vertices for a triangle
     *
     * @param t tuple pointing to an face
     * @return global vids of incident vertices
     */
    std::array<size_t, 3> oriented_tri_vids(const Tuple& t) const;
    std::array<size_t, 3> oriented_tri_vids(const size_t i) const;

    std::array<Tuple, 2> get_edge_vertices(const Tuple& t) const;
    std::array<size_t, 2> get_edge_vids(const Tuple& t) const;

    /**
     * Generate a face Tuple using global fid
     * @param fid global fid for the triangle
     * @note Use the local vid of the first vertex among the incident vertices in the connectivity
     * of the triangle
     * @return a face Tuple
     */
    Tuple tuple_from_tri(size_t fid) const
    {
        if (fid >= m_tri_connectivity.capacity() || m_tri_connectivity[fid].m_is_removed)
            return Tuple();
        auto vid = m_tri_connectivity[fid][0];
        return Tuple(vid, 1, fid, *this);
    }
    /**
     * Generate avertex Tuple using local vid and global fid
     * @param vid globale vid for the triangle
     * @note tuple refers to vid
     * @return an invalid Tuple when vid is out of range, removed, or incident to no
     *         triangle -- the same contract tuple_from_tri has always had.
     *
     * The guard is not decoration. Without it this read m_vertex_connectivity[vid][0]
     * unconditionally, and VertexConnectivity::operator[] only asserts, so in Release a
     * removed or isolated vertex indexed an EMPTY vector: nullptr[0], i.e. a segfault.
     * for_each_vertex builds a Tuple for every slot in [0, vert_capacity()) and only then
     * asks is_valid, so any caller that ran it on an unconsolidated mesh crashed -- 22 of
     * 30 identical runs on Thingi-2D 193539, and 12 models into a 2333-model sweep.
     */
    Tuple tuple_from_vertex(size_t vid) const
    {
        if (vid >= m_vertex_connectivity.capacity() || m_vertex_connectivity[vid].m_is_removed ||
            m_vertex_connectivity[vid].m_conn_tris.empty()) {
            return Tuple();
        }
        auto fid = m_vertex_connectivity[vid][0];
        auto eid = m_tri_connectivity[fid].find((int)vid);
        return Tuple(vid, (eid + 1) % 3, fid, *this);
    }
    /**
     * Generate a edge Tuple using global fid and local eid
     * @param fid globale fid for the triangle
     * @param local_eid local eid
     * @return tuple refers to the edge
     */
    Tuple tuple_from_edge(size_t fid, size_t local_eid) const
    {
        auto vid = m_tri_connectivity[fid][(local_eid + 1) % 3];
        return Tuple(vid, local_eid, fid, *this);
    }

    std::tuple<Tuple, size_t> tuple_from_edge(const std::array<size_t, 2>& vids) const;
    /**
     * @brief tuple_from_edge for callers where a missing edge is an answer, not a bug.
     *
     * The asserting form is right for the many callers that ask for an edge they know exists.
     * collapse_edge_after is not one of them: it asks for an edge across the merged vertex
     * that the collapse may have just removed, and it already has a branch for that case.
     */
    std::optional<std::tuple<Tuple, size_t>> try_tuple_from_edge(
        const std::array<size_t, 2>& vids) const;

public:
    /**
     * @brief Start the phase where the attributes that will be modified can be recorded
     *
     */
    void start_protect_attributes()
    {
        if (p_vertex_attrs) p_vertex_attrs->begin_protect();
        if (p_edge_attrs) p_edge_attrs->begin_protect();
        if (p_face_attrs) p_face_attrs->begin_protect();
    }
    /**
     * @brief End the modification phase
     *
     */
    void release_protect_attributes()
    {
        if (p_vertex_attrs) p_vertex_attrs->end_protect();
        if (p_edge_attrs) p_edge_attrs->end_protect();
        if (p_face_attrs) p_face_attrs->end_protect();
    }
    /**
     * @brief rollback the attributes that are modified if any condition failed
     *
     */
    void rollback_protected_attributes()
    {
        if (p_vertex_attrs) p_vertex_attrs->rollback();
        if (p_edge_attrs) p_edge_attrs->rollback();
        if (p_face_attrs) p_face_attrs->rollback();
    }

    // Moved code from concurrent TriMesh

public:
    /// @see wmtk::threading::VertexMutex. Aliased rather than nested so TriMesh and TetMesh
    /// cannot drift apart again -- they already had.
    using VertexMutex = wmtk::threading::VertexMutex;

private:
    std::vector<VertexMutex> m_vertex_mutex;

    bool try_set_vertex_mutex(const Tuple& v, int threadid)
    {
        bool got = m_vertex_mutex[v.vid(*this)].trylock();
        if (got) m_vertex_mutex[v.vid(*this)].set_owner(threadid);
        return got;
    }
    bool try_set_vertex_mutex(size_t vid, int threadid)
    {
        bool got = m_vertex_mutex[vid].trylock();
        if (got) m_vertex_mutex[vid].set_owner(threadid);
        return got;
    }

    void unlock_vertex_mutex(const Tuple& v) { m_vertex_mutex[v.vid(*this)].unlock(); }
    void unlock_vertex_mutex(size_t vid) { m_vertex_mutex[vid].unlock(); }

    /**
     * @brief Per-thread buffers for the n-ring lock, so a lock acquisition allocates nothing.
     *
     * `stamp[vid] == epoch` marks a vertex already in the ball being built; the epoch is
     * bumped per acquisition instead of clearing the vector.
     */
    struct RingLockScratch
    {
        std::vector<uint32_t> stamp;
        uint32_t epoch = 0;
        std::vector<size_t> frontier;
        std::vector<size_t> next;
        std::vector<size_t> one_ring;
    };
    wmtk::threading::enumerable_thread_specific<RingLockScratch> m_ring_lock_scratch;

    /// The n-ring BFS. @p mark is the release-stack watermark to unwind to on failure.
    bool lock_vertex_ball(const size_t* seeds, size_t n_seeds, int threadid, int n, size_t mark);

protected:
    void resize_mutex(size_t v)
    {
        if (m_vertex_mutex.size() < v) m_vertex_mutex.resize(v);
    }

public:
    wmtk::threading::enumerable_thread_specific<std::vector<size_t>> mutex_release_stack;

    int release_vertex_mutex_in_stack();
    /**
     * @brief Release the mutexes taken since the release stack held @p mark entries.
     *
     * Unwinding to a watermark rather than clearing the whole stack is what lets one lock
     * acquisition be composed out of several, and what lets a failed acquisition leave the
     * caller's own locks alone. `release_vertex_mutex_in_stack()` is this with mark = 0.
     */
    int release_vertex_mutex_to(size_t mark);

    /**
     * @brief Lock every vertex within graph distance @p n of @p v, the seed included.
     *
     * A true breadth-first ball, expanded only through vertices this thread holds, so every
     * connectivity read is made under a lock. On failure it releases exactly what it took and
     * returns false; the caller must not assume anything about the mesh afterwards.
     *
     * n == 0 locks the seed alone; n == 1 the seed and its one-ring, and so on. Sizing the
     * ball is the caller's job: an operation must claim every vertex it READS as well as
     * every vertex it writes, which for an operation that also re-smooths a k-ring means k+1
     * (smoothing a vertex reads its one-ring and writes the quality of its incident faces).
     *
     * @warning This is NOT a drop-in replacement for the *_two_ring / *_one_ring helpers
     * below, which claim a strictly smaller set. Read the note on them before "simplifying"
     * one into the other -- it has a large, measured cost.
     */
    bool try_set_vertex_mutex_n_ring(const Tuple& v, int threadid, int n);
    bool try_set_vertex_mutex_n_ring(size_t vid, int threadid, int n);
    /**
     * @brief try_set_vertex_mutex_n_ring seeded from both ends of an edge.
     */
    bool try_set_edge_mutex_n_ring(const Tuple& e, int threadid, int n);

    /**
     * @name Ring lockers -- NOT balls
     *
     * The three helpers below are the ones every pass actually uses, and **none of them
     * claims the ball its name suggests**. Each walks the ring with
     *
     *     if (m_vertex_mutex[w].get_owner() == threadid) continue;
     *
     * and that `continue` skips the EXPANSION as well as the lock. So a vertex this thread
     * already holds contributes none of its neighbours. In try_set_edge_mutex_two_ring the
     * effect is systematic rather than incidental: v2 is locked up front, so walking v1's
     * ring skips straight past it, and by the time v2's ring is walked almost everything in
     * it is already owned from v1's expansion and is skipped in turn. What comes out is
     * roughly *2-ring(v1) union N(v2)* -- the vertices at distance 2 from v2 and 3 from v1
     * are simply not claimed.
     *
     * **This is deliberate, and swapping in try_set_vertex_mutex_n_ring is a performance
     * regression, not a cleanup.** Measured: routing the passes through the complete ball
     * cost +80% wall clock on the 14 challenging tetwild models at 16 threads (844s -> 1522s)
     * and -0.4% on the 16 challenging triwild ones. 3D is where it hurts, because a tet
     * vertex has ~30 neighbours and the honest 2-ring ball is enormous next to what these
     * claim.
     *
     * It is also a coverage-vs-name mismatch rather than a known race. What the operations
     * rely on is that the whole ONE-ring of the seed simplex is held, and that they hold: the
     * outer loops lock every unowned neighbour directly and only skip expanding through owned
     * ones, so `{v1, v2} u N(v1) u N(v2)` is always claimed. That set is the operation's
     * entire write set, and two operations interfere only if their one-rings meet -- in which
     * case they contend on the shared vertex and serialise. The second ring is defensive
     * margin. No failing case is known for the part that is missing; if you find one, the
     * fix is to widen the specific pass that needs it via try_set_vertex_mutex_n_ring, not to
     * widen all of them.
     *
     * A caller that genuinely needs a complete ball asks for one explicitly. The coarsening
     * pass does: it re-smooths a k-ring, so it writes the k-ring and reads the (k+1)-ring,
     * well past what a plain collapse claims.
     * @{
     */
    /// Lock v's one-ring and, partially, its two-ring. See the note above.
    bool try_set_vertex_mutex_two_ring(const Tuple& v, int threadid);
    /// Lock the edge's one-ring and, partially, its two-ring. See the note above.
    bool try_set_edge_mutex_two_ring(const Tuple& e, int threadid);
    /// Lock v and its one-ring. Complete, unlike the two-ring pair.
    bool try_set_vertex_mutex_one_ring(const Tuple& v, int threadid);
    /** @} */
    /**
     * @brief try lock the one-ring neighboring triangles' incident vertices.
     *
     * @param f Tuple refers to the face
     * @param threadid
     * @return true if all locked successfully
     */
    bool try_set_face_mutex_one_ring(const Tuple& f, int threadid);

    /**
     * @brief perform the given function for each face
     *
     */
    void for_each_face(const std::function<void(const Tuple&)>&);
    /**
     * @brief perform the given function for each edge
     *
     */
    void for_each_edge(const std::function<void(const Tuple&)>&);
    /**
     * @brief perform the given function for each vertex
     *
     */
    void for_each_vertex(const std::function<void(const Tuple&)>&);
    int NUM_THREADS = 0;

public:
    // substructure functionality

    /**
     * @brief Is a vertex part of the substructure
     *
     * @param vid Vertex ID
     */
    virtual bool vertex_is_on_surface(const size_t vid) const { return false; }

    /**
     * @brief Is an edge part of the substructure
     *
     * @param vids The vertex IDs of the edge
     */
    virtual bool edge_is_on_surface(const std::array<size_t, 2>& vids) const { return false; }

    /**
     * @brief Get all edges on the surface that are incident to vid.
     *
     * @param vid Vertex ID
     */
    simplex::SimplexCollection get_surface_edges_for_vertex(const size_t vid) const;

    /**
     * @brief Compute the order of an edge.
     *
     * The order of an edge in a TriMesh is as follows:
     * 0: the edge is not on the surface
     * 1: the edge is on the surface
     *
     * @param vids The vertex IDs of the edge
     */
    size_t get_order_of_edge(const std::array<size_t, 2>& vids) const;

    /**
     * @brief Get the order of a vertex
     *
     * The order of a vertex in a TriMesh is as follows:
     * 0: vertex is not on the surface
     * 1: vertex is on the surface
     * 2: vertex is a non-manifold vertex in the substructure
     *
     * @param vid Vertex ID
     */
    size_t get_order_of_vertex(const size_t vid) const;

    /**
     * @brief Link condition that also considers substructures.
     *
     * Implementation based on the pseudo code from the paper:
     * Vivodtzev et. al. - Substructure Topology Preserving Simplification of Tetrahedral Meshes
     *
     * The math and the pseudo code in the paper contain errors! The theory itself is correct.
     *
     * The link condition must be evaluated for the mesh and all substructures (surfaces, lines,
     * points). If there is a substructure simplex in the star, the simplex is extended with a dummy
     * vertex (e.g., an edge becomes a face) and this extended simplex must also be considered for
     * the link.
     */
    bool substructure_link_condition(const Tuple& e_tuple) const;
};

} // namespace wmtk
