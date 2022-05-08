#pragma once

#include <wmtk/utils/VectorUtils.h>
#include <type_traits>
#include <wmtk/AttributeCollection.hpp>
#include <wmtk/utils/Logger.hpp>

#include <tbb/concurrent_vector.h>
#include <tbb/spin_mutex.h>

#include <Tracy.hpp>

#include <array>
#include <cassert>
#include <map>
#include <optional>
#include <queue>
#include <vector>

namespace wmtk {
class TetMesh
{
private:
    static constexpr std::array<std::array<int, 2>, 6> m_local_edges = {
        {{{0, 1}}, {{1, 2}}, {{0, 2}}, {{0, 3}}, {{1, 3}}, {{2, 3}}}}; // local edges within a
    // tet
    static constexpr std::array<int, 4> m_map_vertex2edge = {{0, 0, 1, 3}};
    static constexpr std::array<int, 4> m_map_vertex2oppo_face = {{3, 1, 2, 0}};
    static constexpr std::array<int, 6> m_map_edge2face = {{0, 0, 0, 1, 2, 1}};
    static constexpr std::array<std::array<int, 3>, 4> m_local_faces = {
        {{{0, 1, 2}}, {{0, 2, 3}}, {{0, 1, 3}}, {{1, 2, 3}}}}; // sorted local vids
    static constexpr std::array<std::array<int, 3>, 4> m_local_edges_in_a_face = {
        {{{0, 1, 2}}, {{2, 5, 3}}, {{3, 4, 0}}, {{5, 1, 4}}}};

public:
    // Cell Tuple Navigator
    class Tuple
    {
        size_t m_global_vid = std::numeric_limits<size_t>::max();
        size_t m_local_eid = std::numeric_limits<size_t>::max();
        size_t m_local_fid = std::numeric_limits<size_t>::max();
        size_t m_global_tid = std::numeric_limits<size_t>::max();

        int m_hash = 0;

    private:
        /**
         * Construct a new Tuple object with global vertex/tetra index and local edge/face index
         *
         * @param vid vertex id (global)
         * @param eid edge id (local)
         * @param fid face id (local)
         * @param tid tetra id (global)
         * @param ts hash associated with tid
         */
        Tuple(const TetMesh& m, size_t vid, size_t local_eid, size_t local_fid, size_t tid);

    public:
        Tuple() {}

        friend TetMesh;

        /**
         * Check if the current tuple is already invalid (removed during editing).
         *
         * @param m TetMesh where the tuple belongs.
         * @return if not removed and the tuple is up to date with respect to the connectivity.
         */
        bool is_valid(const TetMesh& m) const;
        bool is_boundary_edge(const TetMesh& m) const;
        bool is_boundary_face(const TetMesh& m) const;

        /**
         * @brief prints the tuple
         *
         */
        void print_info() const;

        /**
         * @brief prints additional information
         *
         * @param m mesh
         */
        void print_info(const TetMesh& m) const;

        /**
         * returns global vertex id.
         * @param m TetMesh where the tuple belongs.
         * @return size_t
         */
        size_t vid(const TetMesh& m) const;

        /**
         * returns a global unique edge id
         *
         * @param m TetMesh where the tuple belongs.
         * @return size_t
         * @note The global id may not be consecutive. The edges are undirected and different tetra
         * share the same edge.
         */
        size_t eid(const TetMesh& m) const;

        /**
         * returns a global unique face id
         *
         * @param m TetMesh where the tuple belongs.
         * @return size_t
         * @note The global id may not be consecutive. The face are undirected.
         */
        size_t fid(const TetMesh& m) const;

        /**
         * returns global tetra id.

         * @param m TetMesh where the tuple belongs.
         * @return size_t
         */
        size_t tid(const TetMesh& m) const;

        /**
         * Switch operation. See (URL-TO-DOCUMENT) for explaination.
         *
         * @param m
         * @return Tuple another Tuple that share the same tetra, face, edge, but different vertex.
         */
        Tuple switch_vertex(const TetMesh& m) const;
        Tuple switch_edge(const TetMesh& m) const;
        Tuple switch_face(const TetMesh& m) const;

        /**
         * Switch operation for the adjacent tetra.
         *
         * @param m Mesh
         * @return Tuple for the face-adjacent tetra, sharing same face, edge, and vertex.
         * @return nullopt if the Tuple is the switch goes off the boundary.
         */
        std::optional<Tuple> switch_tetrahedron(const TetMesh& m) const;


        ////testing code
        void check_validity(const TetMesh& m) const;
        friend bool operator==(const Tuple& a, const Tuple& t)
        {
            return (
                std::tie(a.m_global_vid, a.m_local_eid, a.m_local_fid, a.m_global_tid, a.m_hash) ==
                std::tie(t.m_global_vid, t.m_local_eid, t.m_local_fid, t.m_global_tid, t.m_hash));
        }
        friend bool operator<(const Tuple& a, const Tuple& t)
        {
            return (
                std::tie(a.m_global_vid, a.m_local_eid, a.m_local_fid, a.m_global_tid, a.m_hash) <
                std::tie(t.m_global_vid, t.m_local_eid, t.m_local_fid, t.m_global_tid, t.m_hash));
        }
    };

    /**
     * (internal use) Maintains a list of tetra connected to the given vertex, and a flag to
     * mark removal.
     *
     */
    class VertexConnectivity
    {
    public:
        std::vector<size_t> m_conn_tets; // todo: always keep it sorted
        bool m_is_removed = false;

        size_t& operator[](const size_t index)
        {
            assert(index < m_conn_tets.size());
            return m_conn_tets[index];
        }

        size_t operator[](const size_t index) const
        {
            assert(index < m_conn_tets.size());
            return m_conn_tets[index];
        }

        friend bool operator==(const VertexConnectivity& l, const VertexConnectivity& r)
        {
            return std::tie(l.m_conn_tets, l.m_is_removed) ==
                   std::tie(r.m_conn_tets, r.m_is_removed); // keep the same order
        }

        void print_info() {}
    };

    /**
     * (internal use) Maintains the vertices of a given tetra.
     *
     */
    class TetrahedronConnectivity
    {
    public:
        std::array<size_t, 4> m_indices;
        bool m_is_removed = false;

        int hash = 0;

        void set_version_number(int version) { hash = version; }

        int get_version_number() { return hash; }

        size_t& operator[](size_t index)
        {
            assert(index < 4);
            return m_indices[index];
        }

        size_t operator[](size_t index) const
        {
            assert(index < 4);
            return m_indices[index];
        }

        int find(size_t v_id) const
        {
            for (int j = 0; j < 4; j++) {
                if (v_id == m_indices[j]) return j;
            }
            return -1;
        }

        int find_local_edge(size_t v1_id, size_t v2_id) const
        {
            std::array<int, 2> e;
            for (int j = 0; j < 4; j++) {
                if (v1_id == m_indices[j])
                    e[0] = j;
                else if (v2_id == m_indices[j])
                    e[1] = j;
            }
            if (e[0] > e[1]) std::swap(e[0], e[1]);
            int i =
                std::find(m_local_edges.begin(), m_local_edges.end(), e) - m_local_edges.begin();
            if (i >= m_local_edges.size()) return -1;
            return i;
        }

        int find_local_face(size_t v1_id, size_t v2_id, size_t v3_id) const
        {
            std::array<int, 3> f;
            for (int j = 0; j < 4; j++) {
                if (v1_id == m_indices[j])
                    f[0] = j;
                else if (v2_id == m_indices[j])
                    f[1] = j;
                else if (v3_id == m_indices[j])
                    f[2] = j;
            }
            std::sort(f.begin(), f.end());
            int i =
                std::find(m_local_faces.begin(), m_local_faces.end(), f) - m_local_faces.begin();
            if (i >= m_local_edges.size()) return -1;
            return i;
        }

        friend bool operator==(const TetrahedronConnectivity& l, const TetrahedronConnectivity& r)
        {
            return std::tie(l.m_indices, l.m_is_removed, l.hash) ==
                   std::tie(r.m_indices, r.m_is_removed, r.hash); // keep the same order
        }

        void print_info() {}
    };

    TetMesh();
    virtual ~TetMesh() = default;

    size_t vert_capacity() const { return current_vert_size; }
    size_t tet_capacity() const { return current_tet_size; }



    size_t vertex_size() const
    {
        int cnt = 0;
        for (auto i = 0; i < vert_capacity(); i++ ){
            if (!m_vertex_connectivity[i].m_is_removed) cnt++;
        }
        return cnt;
    }
    size_t tet_size() const
    {

        int cnt = 0;
        for (auto i = 0; i < tet_capacity(); i++ ){
            if (!m_tet_connectivity[i].m_is_removed) cnt++;
        }
        return cnt;
    }
    /**
     * Initialize TetMesh data structure
     *
     * @param n_vertices number of vertices
     * @param tets vector of array. Each element represents one tet, which is defined by four
     * vertices.
     * @note Assuming oriented and manifold, but no embedding. The maximum index in `tets` should
     * not exceed `n_vertices`
     */
    void init(size_t n_vertices, const std::vector<std::array<size_t, 4>>& tets);

    /**
     * Split an edge
     *
     * @param t Input Tuple for the edge to split.
     * @param[out] new_tets a vector of Tuples for all the newly introduced tetra.
     * @return if split succeed
     */

    bool split_edge(const Tuple& t, std::vector<Tuple>& new_tets);
    bool collapse_edge(const Tuple& t, std::vector<Tuple>& new_tets);
    bool swap_edge_44(const Tuple& t, std::vector<Tuple>& new_tets);
    bool swap_edge(const Tuple& t, std::vector<Tuple>& new_tets);
    bool swap_face(const Tuple& t, std::vector<Tuple>& new_tets);
    bool smooth_vertex(const Tuple& t);

    bool split_tet(const Tuple& t, std::vector<Tuple>& new_tets);
    bool split_face(const Tuple& t, std::vector<Tuple>& new_tets);

    /**
     * @brief Insert a triangle into a tetmesh, with known intersection information
     *
     * @param intersected_tets the tet to split
     * @param intersected_edges the edges where new points are assigned to
     * @param new_edge_vids new vertices correspond to each cut-edge
     */
    void triangle_insertion(
        const std::vector<Tuple>& intersected_tets,
        const std::vector<Tuple>& intersected_edges,
        std::vector<size_t>& new_edge_vids,
        std::vector<size_t>& new_center_vids,
        std::vector<std::array<size_t, 4>>& center_split_tets);

    /**
     * @brief Insert a point into a tetmesh inside a tet.
     * In general position, this split a tet into 4.
     * In face position, split two tets.
     * In edge position,
     * In point position, do nothing.
     * @return true
     * @return false
     */
    bool insert_point(const Tuple& t, std::vector<Tuple>& new_tets);
    virtual bool insert_point_before(const Tuple& t) { return true; };
    virtual bool insert_point_after(std::vector<Tuple>& new_tets) { return true; };
    /**
     * @brief cleans up the deleted vertices or tetrahedra, fixes the corresponding indices, and
     * reset the version number. WARNING: it invalidates all tuples!
     *
     */
    void consolidate_mesh();

    /**
     * Get all unique undirected edges in the mesh.
     *
     * @return std::vector<Tuple> each Tuple owns a distinct edge.
     */
    std::vector<Tuple> get_edges() const;
    std::vector<Tuple> get_faces() const;
    std::vector<Tuple> get_vertices() const;
    std::vector<Tuple> get_tets() const;
    virtual void for_each_edge(const std::function<void(const TetMesh::Tuple&)>&);
    virtual void for_each_face(const std::function<void(const TetMesh::Tuple&)>&);
    virtual void for_each_vertex(const std::function<void(const TetMesh::Tuple&)>&);
    virtual void for_each_tetra(const std::function<void(const TetMesh::Tuple&)>&);

public:
    template <typename T>
    using vector = tbb::concurrent_vector<T>;

public:
    AbstractAttributeContainer *p_vertex_attrs, *p_edge_attrs, *p_face_attrs, *p_tet_attrs;
    AbstractAttributeContainer vertex_attrs, edge_attrs, face_attrs, tet_attrs;
    

private:
    // Stores the connectivity of the mesh
    vector<VertexConnectivity> m_vertex_connectivity;
    vector<TetrahedronConnectivity> m_tet_connectivity;
    std::atomic_long current_vert_size;
    std::atomic_long current_tet_size;
    tbb::spin_mutex vertex_connectivity_lock;
    tbb::spin_mutex tet_connectivity_lock;
    bool vertex_connectivity_synchronizing_flag = false;
    bool tet_connectivity_synchronizing_flag = false;

    int m_t_empty_slot = 0;
    int m_v_empty_slot = 0;
    int get_next_empty_slot_t();
    int get_next_empty_slot_v();

    // TODO: subdivide_tets function should not be in the TetMesh API.
    void subdivide_tets(
        const std::vector<size_t> t_ids,
        const std::vector<bool>& mark_surface,
        const std::map<std::array<size_t, 2>, size_t>& map_edge2vid,
        std::map<std::array<size_t, 3>, std::vector<std::array<size_t, 5>>>& new_face_vids,
        const std::vector<size_t>& new_vids,
        std::vector<size_t>& new_tids,
        std::vector<size_t>& new_center_vids,
        std::vector<std::array<size_t, 4>>& center_split_tets);
    void subdivide_a_tet(
        size_t t_id,
        const std::array<int, 6>& new_v_ids,
        bool mark_surface,
        std::map<std::array<size_t, 3>, std::vector<std::array<size_t, 5>>>& new_face_vids,
        std::vector<size_t>& new_tids,
        std::vector<size_t>& new_center_vids,
        std::vector<std::array<size_t, 4>>& center_split_tets);

protected:
    virtual bool invariants(const std::vector<Tuple>&) { return true; }
    virtual bool triangle_insertion_before(const std::vector<Tuple>& faces) { return true; }
    virtual bool triangle_insertion_after(const std::vector<std::vector<Tuple>>&) { return true; }

    //// Split the edge in the tuple
    // Checks if the split should be performed or not (user controlled)
    virtual bool split_edge_before(const Tuple& t) { return true; } // check edge condition
    // This function computes the attributes for the added simplices
    // if it returns false then the operation is undone
    virtual bool split_edge_after(const Tuple& t) { return true; } // check tet condition

    //// Collapse the edge in the tuple
    // Checks if the collapse should be performed or not (user controlled)
    virtual bool collapse_edge_before(const Tuple& t) { return true; }
    // If it returns false then the operation is undone (the tuple indexes a vertex and tet that
    // survived)

    virtual bool swap_edge_44_before(const Tuple& t) { return true; }
    virtual bool swap_edge_44_after(const Tuple& t) { return true; }
    virtual bool swap_edge_before(const Tuple& t) { return true; }
    virtual bool swap_edge_after(const Tuple& t) { return true; }
    virtual bool swap_face_before(const Tuple& t) { return true; }
    virtual bool swap_face_after(const Tuple& t) { return true; }

    virtual bool collapse_edge_after(const Tuple& t) { return true; }
    virtual bool smooth_before(const Tuple& t) { return true; }
    virtual bool smooth_after(const Tuple& t) { return true; }

    virtual void resize_vertex_mutex(size_t v) {}

public:
    /**
     * @brief get a Tuple from global tetra index and __local__ edge index (from 0-5).
     *
     * @param m TetMesh where the current Tuple belongs.
     * @param tid Global tetra index
     * @param local_eid local edge index
     */
    Tuple tuple_from_edge(size_t tid, int local_eid) const;
    Tuple tuple_from_edge(const std::array<size_t,2>& vids) const;

    /**
     * @brief get a Tuple from global tetra index and __local__ face index (from 0-3).
     *
     * @param m TetMesh where the current Tuple belongs.
     * @param tid Global tetra index
     * @param local_fid local face index
     */
    Tuple tuple_from_face(size_t tid, int local_fid) const;

    /**
     * @brief get a Tuple and the global face index from global vertex index of the face.
     *
     * @param m TetMesh where the current Tuple belongs.
     * @param vids Global vertex index of the face
     */
    std::tuple<Tuple, size_t> tuple_from_face(const std::array<size_t, 3>& vids) const;

    /**
     * @brief get a Tuple from global vertex index
     *
     * @param m TetMesh where the current Tuple belongs.
     * @param vid Global vertex index
     */
    Tuple tuple_from_vertex(size_t vid) const;

    /**
     * @brief get a Tuple from global tetra index
     *
     * @param m TetMesh where the current Tuple belongs.
     * @param tid Global tetra index
     */
    Tuple tuple_from_tet(size_t tid) const;


    /**
     * @brief wrapper function from tuple
     */
    Tuple switch_vertex(const Tuple& t) const
    {
        auto loc = t.switch_vertex(*this);
        check_tuple_validity(loc);
        return loc;
    }
    Tuple switch_edge(const Tuple& t) const
    {
        auto loc = t.switch_edge(*this);
        check_tuple_validity(loc);
        return loc;
    }
    Tuple switch_face(const Tuple& t) const
    {
        auto loc = t.switch_face(*this);
        check_tuple_validity(loc);
        return loc;
    }
    std::optional<Tuple> switch_tetrahedron(const Tuple& t) const
    {
        auto loc = t.switch_tetrahedron(*this);
        if (loc.has_value()) check_tuple_validity(loc.value());
        return loc;
    }

    /**
     * @brief Get the one ring tets for a vertex
     *
     * @param t tuple pointing to a vertex
     * @return one-ring
     */
    std::vector<Tuple> get_one_ring_tets_for_vertex(const Tuple& t) const;

    /**
     * @brief Get the one ring vertices for a vertex
     *
     * @param t tuple pointing to a vertex
     * @return one-ring vertices
     */
    std::vector<Tuple> get_one_ring_vertices_for_vertex(const Tuple& t) const;
    std::vector<size_t> get_one_ring_vids_for_vertex(size_t vid, std::vector<size_t>& cache);
    std::vector<size_t> get_one_ring_vids_for_vertex(size_t vid) const;
    std::vector<size_t> get_one_ring_vids_for_vertex_adj(size_t vid) const;
    std::vector<size_t> get_one_ring_vids_for_vertex_adj(size_t vid, std::vector<size_t>& cache);

    /**
     * @brief Get the incident tets for edge
     *
     * @param t tuple pointing to an edge
     * @return incident tets
     */
    std::vector<Tuple> get_incident_tets_for_edge(const Tuple& t) const;

    /**
     * @brief Get the one ring tets for edge
     *
     * @param t tuple pointing to an edge
     * @return one ring
     */
    std::vector<Tuple> get_one_ring_tets_for_edge(const Tuple& t) const;

    /**
     * @brief
     *
     * @param m
     * @return std::vector<std::array<size_t,3>>
     */
    std::vector<std::array<size_t, 3>> vertex_adjacent_boundary_faces(const Tuple& t) const;
    /**
     * Positively oriented 4 vertices (represented by Tuples) in a tetra.
     * @return std::array<Tuple, 4> each tuple owns a different vertex.
     */
    std::array<Tuple, 4> oriented_tet_vertices(const Tuple& t) const;
    std::array<size_t, 4> oriented_tet_vids(const Tuple& t) const;
    std::array<Tuple, 3> get_face_vertices(const Tuple& t) const;

    std::array<Tuple, 6> tet_edges(const Tuple& t) const;

    void check_tuple_validity(const Tuple& t) const { t.check_validity(*this); }
    bool check_mesh_connectivity_validity() const;

    void remove_tets_by_ids(const std::vector<size_t>& tids)
    {
        for (size_t tid : tids) {
            m_tet_connectivity[tid].m_is_removed = true;
            for (int j = 0; j < 4; j++)
                vector_erase(m_vertex_connectivity[m_tet_connectivity[tid][j]].m_conn_tets, tid);
        }
        for (auto& v : m_vertex_connectivity) {
            if (v.m_is_removed) continue;
            if (v.m_conn_tets.empty()) v.m_is_removed = true;
        }
    }
    bool m_collapse_check_link_condition = true;

private:
    std::map<size_t, wmtk::TetMesh::VertexConnectivity> operation_update_connectivity_impl(
        std::vector<size_t>& affected_tid,
        const std::vector<std::array<size_t, 4>>& new_tet_conn);
    void operation_failure_rollback_imp(
        std::map<size_t, wmtk::TetMesh::VertexConnectivity>& rollback_vert_conn,
        const std::vector<size_t>& affected,
        const std::vector<size_t>& new_tet_id,
        const std::vector<wmtk::TetMesh::TetrahedronConnectivity>& old_tets);
    std::map<size_t, wmtk::TetMesh::VertexConnectivity> operation_update_connectivity_impl(
        const std::vector<size_t>& remove_id,
        const std::vector<std::array<size_t, 4>>& new_tet_conn,
        std::vector<size_t>& allocate_id);
    static std::vector<wmtk::TetMesh::TetrahedronConnectivity> record_old_tet_connectivity(
        const wmtk::TetMesh::vector<wmtk::TetMesh::TetrahedronConnectivity>& conn,
        const std::vector<size_t>& tets)
    {
        auto tet_conn = std::vector<wmtk::TetMesh::TetrahedronConnectivity>();
        for (auto i : tets) tet_conn.push_back(conn[i]);
        return tet_conn;
    }
    void start_protect_attributes()
    {
        p_vertex_attrs->begin_protect();
        p_edge_attrs->begin_protect();
        p_face_attrs->begin_protect();
        p_tet_attrs->begin_protect();
    }

    void release_protect_attributes()
    {
        p_vertex_attrs->end_protect();
        p_edge_attrs->end_protect();
        p_face_attrs->end_protect();
        p_tet_attrs->end_protect();
    }

    void rollback_protected_attributes()
    {
        p_vertex_attrs->rollback();
        p_edge_attrs->rollback();
        p_face_attrs->rollback();
        p_tet_attrs->rollback();
    }

public:
    class OperationBuilder
    {
    public:
        OperationBuilder() = default;
        ~OperationBuilder() = default;
        bool before(const Tuple&) { return true; }
        bool after(const std::vector<Tuple>&) { return true; }
        std::vector<size_t> removed_tids(const Tuple&);
        int request_vert_slots() {return 0;};
        std::vector<std::array<size_t, 4>> replacing_tets(const std::vector<size_t>&);
    };

    // dangerous usage, backdoor for private access.
    template <int id>
    class InternalOperationBuilder : public OperationBuilder{};

    template <typename T, typename = std::enable_if_t<std::is_base_of_v<OperationBuilder, T>>>
    bool customized_operation(T& op, const Tuple& tup, std::vector<Tuple>& new_tet_tuples)
    {
        if (op.before(tup) == false) return false;
        const auto& affected = op.removed_tids(tup);
        auto old_tets = record_old_tet_connectivity(m_tet_connectivity, affected);

		auto new_vnum = op.request_vert_slots();
        std::vector<size_t> new_vids(new_vnum);
        for (auto i =0; i < new_vnum; i++) {
            new_vids[i] = get_next_empty_slot_v();
        }
        const auto& new_tets = op.replacing_tets(new_vids);

        auto new_tet_id = affected;
        auto rollback_vert_conn = operation_update_connectivity_impl(new_tet_id, new_tets);

        for (auto ti : new_tet_id) new_tet_tuples.emplace_back(tuple_from_tet(ti));

        start_protect_attributes();
        if (!op.after(new_tet_tuples) || !invariants(new_tet_tuples)) { // rollback post-operation

            logger().trace("rolling back");
            operation_failure_rollback_imp(rollback_vert_conn, affected, new_tet_id, old_tets);
            return false;
        }
        release_protect_attributes();
        return true;
    }
};

} // namespace wmtk
