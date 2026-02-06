#pragma once

#include <wmtk/utils/VectorUtils.h>
#include <type_traits>
#include <wmtk/AttributeCollection.hpp>
#include <wmtk/Types.hpp>
#include <wmtk/simplex/RawSimplex.hpp>
#include <wmtk/utils/Logger.hpp>

#include <tbb/concurrent_vector.h>
#include <tbb/enumerable_thread_specific.h>
#include <tbb/spin_mutex.h>

#include <array>
#include <cassert>
#include <limits>
#include <map>
#include <optional>
#include <queue>
#include <vector>

namespace wmtk {
class TetMesh
{
private:
    /**
     * @brief local edges within a tet
     *
     */
    static constexpr std::array<std::array<int, 2>, 6> m_local_edges = {
        {{{0, 1}}, {{1, 2}}, {{0, 2}}, {{0, 3}}, {{1, 3}}, {{2, 3}}}};

    static constexpr std::array<int, 4> m_map_vertex2edge = {{0, 0, 1, 3}};
    static constexpr std::array<int, 4> m_map_vertex2oppo_face = {{3, 1, 2, 0}};
    static constexpr std::array<int, 6> m_map_edge2face = {{0, 0, 0, 1, 2, 1}};
    static constexpr std::array<std::array<int, 3>, 4> m_local_faces = {
        {{{0, 1, 2}}, {{0, 2, 3}}, {{0, 1, 3}}, {{1, 2, 3}}}}; // sorted local vids
    static constexpr std::array<std::array<int, 3>, 4> m_local_edges_in_a_face = {
        {{{0, 1, 2}}, {{2, 5, 3}}, {{3, 4, 0}}, {{5, 1, 4}}}};

public:
    // Cell Tuple Navigator
    /**
     * @brief a Tuple refers to a global vid and a global tet id, and a local edge id and local face
     * id
     *
     */
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
         * @return if not removed and the tuple is up to date with respect to the connectivity. And
         * the tet id can't be -1
         */
        bool is_valid(const TetMesh& m) const;
        /**
         * Check if the current tuple the refers to an edge is on the boundary
         *
         * @param m TetMesh where the tuple belongs.
         * @return if the edge is at the mesh boundary
         */
        bool is_boundary_edge(const TetMesh& m) const;
        /**
         * Check if the current tuple the refers to a face is on the boundary
         *
         * @param m TetMesh where the tuple belongs.
         * @return if the edge is at the mesh boundary
         */
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
         * Switch operation.
         *
         * @param m the mesh the Tuple is in
         * @return another Tuple that share the same tetra, face, edge, but different vertex.
         */
        Tuple switch_vertex(const TetMesh& m) const;
        /**
         * Switch operation.
         *
         * @param m the mesh the Tuple is in
         * @return another Tuple that share the same tetra, face, vertex, but different edge.
         */
        Tuple switch_edge(const TetMesh& m) const;
        /**
         * Switch operation.
         *
         * @param m the mesh the Tuple is in
         * @return another Tuple that share the same tetra, edge, vertex, but different edge.
         */
        Tuple switch_face(const TetMesh& m) const;

        /**
         * Switch operation for the adjacent tetra.
         *
         * @param m Mesh
         * @return Tuple for the face-adjacent tetra, sharing same face, edge, and vertex.
         * @return nullopt if the Tuple is the switch goes off the boundary.
         */
        std::optional<Tuple> switch_tetrahedron(const TetMesh& m) const;
        std::optional<Tuple> switch_tetrahedron_slow(const TetMesh& m) const;


        ////testing code
        /**
         * @brief check Tuple validity and connectivity validity
         */
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
     * A Tuple that holds a reference to the mesh.
     * This is a utility to simplify switching sequences.
     */
    class SmartTuple
    {
        Tuple m_tuple;
        const TetMesh& m_mesh;

    public:
        SmartTuple(const TetMesh& mesh, const Tuple& t)
            : m_mesh(mesh)
            , m_tuple(t)
        {}

        const Tuple& tuple() { return m_tuple; }
        const TetMesh& mesh() { return m_mesh; }

        SmartTuple& operator=(const SmartTuple& t)
        {
            m_tuple = t.m_tuple;
            return *this;
        }

        bool is_valid() const { return m_tuple.is_valid(m_mesh); }
        bool is_boundary_edge() const { return m_tuple.is_boundary_edge(m_mesh); }
        bool is_boundary_face() const { return m_tuple.is_boundary_face(m_mesh); }
        size_t vid() const { return m_tuple.vid(m_mesh); }
        size_t eid() const { return m_tuple.eid(m_mesh); }
        size_t fid() const { return m_tuple.fid(m_mesh); }
        size_t tid() const { return m_tuple.tid(m_mesh); }
        SmartTuple switch_vertex() const { return {m_mesh, m_tuple.switch_vertex(m_mesh)}; }
        SmartTuple switch_edge() const { return {m_mesh, m_tuple.switch_edge(m_mesh)}; }
        SmartTuple switch_face() const { return {m_mesh, m_tuple.switch_face(m_mesh)}; }
        std::optional<SmartTuple> switch_tetrahedron() const
        {
            const std::optional<Tuple> t = m_tuple.switch_tetrahedron(m_mesh);
            if (t) {
                return std::optional<SmartTuple>({m_mesh, t.value()});
            }
            return {};
        }
        void check_validity() const { return m_tuple.check_validity(m_mesh); }
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
    /**
     * @brief get the current largest global vid
     *
     * @return size_t
     */
    size_t vert_capacity() const { return current_vert_size; }
    /**
     * @brief get the current largest global tid
     *
     * @return size_t
     */
    size_t tet_capacity() const { return current_tet_size; }

    /**
     * @brief get the number of unremoved verticies
     *
     */
    size_t vertex_size() const
    {
        int cnt = 0;
        for (auto i = 0; i < vert_capacity(); i++) {
            if (!m_vertex_connectivity[i].m_is_removed) cnt++;
        }
        return cnt;
    }
    /**
     * @brief get the number of unremoved tets
     *
     */
    size_t tet_size() const
    {
        int cnt = 0;
        for (auto i = 0; i < tet_capacity(); i++) {
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
    void init_with_isolated_vertices(
        size_t n_vertices,
        const std::vector<std::array<size_t, 4>>& tets);

    /**
     * @brief Generate the connectivity of the mesh from an IGL-style T matrix.
     *
     * @param #T by 4 list of vertex indices.
     */
    void init(const MatrixXi& T);

    /**
     * Split an edge
     *
     * @param t Input Tuple for the edge to split.
     * @param[out] new_tets a vector of Tuples for all the newly introduced tetra.
     * @return if split succeed
     */
    bool split_edge(const Tuple& t, std::vector<Tuple>& new_tets);
    /**
     * Collapse an edge
     *
     * @param t Input Tuple for the edge to collapse.
     * @param[out] new_tets a vector of Tuples for all the newly introduced tetra.
     * @return if collapse succeed
     */
    virtual bool collapse_edge(const Tuple& t, std::vector<Tuple>& new_tets);

    bool link_condition(const Tuple& t);

    /**
     * Collapse edge connectivity change part. Constains a link condition check and the connecticity
     * update
     *
     * @param loc0 Input Tuple for the edge to collapse
     * @param[out] new_tets a vector of Tuples for all the newly introduced tetra.
     * @param[out] v1_id vertex id of the input tuple
     * @param[out] new_loc result vertex tuple
     * @param[out] rollback_vert_conn vertex connectivity got changed and will be involed in
     * rollback
     * @param[out] n1_t_ids_copy origninal (before collape) one ring tet ids connected to the input
     * vertex
     * @param[out] new_tet_id new tet ids added to v2
     * @param[out] old_tets tets tv connectivities in n1_t_ids_copy
     *
     * @return if true collapse pass link condition check
     */
    bool collapse_edge_conn(
        const Tuple& loc0,
        std::vector<Tuple>& new_edges,
        size_t& v1_id,
        Tuple& new_loc,
        std::map<size_t, wmtk::TetMesh::VertexConnectivity>& rollback_vert_conn,
        std::vector<size_t>& n1_t_ids_copy,
        std::vector<size_t>& new_tet_id,
        std::vector<TetrahedronConnectivity>& old_tets);

    /**
     * @brief Check topology after collapse connectivity change. This is a sanity check and should
     * not be necessary.
     *
     * @param new_tet_id new tet ids added to v2
     *
     * @return if true the topology is valid
     */
    bool collapse_edge_check_topology(const std::vector<size_t>& new_tet_id);

    /**
     *  rollback function for collapse edges
     *
     * @param[out] v1_id vertex id of the input tuple
     * @param[out] rollback_vert_conn vertex connectivity got changed and will be involed in
     * rollback
     * @param[out] n1_t_ids origninal (before collape) one ring tet ids connected to the input
     * vertex
     * @param[out] new_tet_id new tet ids added to v2
     * @param[out] old_tets tets tv connectivities in n1_t_ids
     *
     */
    void collapse_edge_rollback(
        size_t& v1_id,
        std::map<size_t, wmtk::TetMesh::VertexConnectivity>& rollback_vert_conn,
        std::vector<size_t>& n1_t_ids,
        std::vector<size_t>& new_tet_id,
        std::vector<TetrahedronConnectivity>& old_tets);

    /**
     *Perform 5-6 swap
     *
     * @param t Input Tuple for the edge to swap.
     * @param[out] new_tets a vector of Tuples for all the newly introduced tetra.
     * @return true if swap succeed
     * @note only happens on internal edges
     */
    bool swap_edge_56(const Tuple& t, std::vector<Tuple>& new_tets);
    /**
     *Perform 4-4 swap between 2 tets
     *
     * @param t Input Tuple for the edge to swap.
     * @param[out] new_tets a vector of Tuples for all the newly introduced tetra.
     * @return if swap succeed
     * @note only happens on internal edges
     */
    bool swap_edge_44(const Tuple& t, std::vector<Tuple>& new_tets);
    /**
     * @brief  3-2 edge swap
     * @note only swap internal edges, not on boundary.
     *
     * @param t Input Tuple for the edge to swap.
     * @param new_tets a vector of Tuples for all the newly introduced tetra.
     * @return true if swap succeed
     */
    bool swap_edge(const Tuple& t, std::vector<Tuple>& new_tets);
    /**
     * @brief  2-3 face swap
     * @param t Input Tuple for the edge to swap.
     * @param new_tets a vector of Tuples for all the newly introduced tetra.
     * @return true if swap succeed
     */
    bool swap_face(const Tuple& t, std::vector<Tuple>& new_tets);
    /**
     * Smooth a vertex
     *
     * @param t Input Tuple for the vertex
     * @note no geometry changed here
     * @return if smooth succeed
     */
    bool smooth_vertex(const Tuple& t);

    /**
     * @brief Split a tet in 4 tets.
     *
     * @param t Input tuple for the tet to split.
     * @param[out] new_t A vector of Tuples refering to the tets incident to the new vertex.
     * introduced
     * @return true, if split succeed
     */
    bool split_tet(const Tuple& t, std::vector<Tuple>& new_tets);

    /**
     * @brief Split a face in 3 faces.
     *
     * The TetMesh is assumed to be face manifold, i.e., a face has at most two incident tets.
     *
     * @param t Input tuple for the face to split.
     * @param[out] new_t A vector of Tuples refering to the tets incident to the new vertex.
     * introduced
     * @return true, if split succeed
     */
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
    /**
     * Get all unique faces in the mesh.
     *
     * @return std::vector<Tuple> each Tuple owns a distinct face.
     */
    std::vector<Tuple> get_faces() const;
    /**
     * Get all unique vertices in the mesh.
     *
     * @return std::vector<Tuple> each Tuple owns a distinct vertex
     */
    std::vector<Tuple> get_vertices() const;
    /**
     * Get all unique tet in the mesh.
     *
     * @return std::vector<Tuple> each Tuple owns a distinct tet
     */
    std::vector<Tuple> get_tets() const;
    /**
     * @brief looping through all the unique edges and perform the given function
     *
     */
    // virtual void for_each_edge(const std::function<void(const TetMesh::Tuple&)>&);

    // TODO: make this concurrent
    /**
     * @brief looping through all the unique faces and perform the given function
     *
     */
    virtual void for_each_face(const std::function<void(const TetMesh::Tuple&)>&);
    // /**
    //  * @brief looping through all the unique vertices and perform the given function
    //  *
    //  */
    // virtual void for_each_vertex(const std::function<void(const TetMesh::Tuple&)>&);
    // /**
    //  * @brief looping through all the unique tet and perform the given function
    //  *
    //  */
    // virtual void for_each_tetra(const std::function<void(const TetMesh::Tuple&)>&);

public:
    template <typename T>
    using vector = tbb::concurrent_vector<T>;

public:
    AbstractAttributeContainer* p_vertex_attrs = nullptr;
    AbstractAttributeContainer* p_edge_attrs = nullptr;
    AbstractAttributeContainer* p_face_attrs = nullptr;
    AbstractAttributeContainer* p_tet_attrs = nullptr;
    // AbstractAttributeContainer vertex_attrs, edge_attrs, face_attrs, tet_attrs;


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
    int MAX_THREADS = 128;

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

public:
    virtual bool invariants(const std::vector<Tuple>&) { return true; }

protected:
    virtual bool triangle_insertion_before(const std::vector<Tuple>& faces) { return true; }
    virtual bool triangle_insertion_after(const std::vector<std::vector<Tuple>>&) { return true; }

    //// Split the edge in the tuple
    // Checks if the split should be performed or not (user controlled)
    /**
     * @brief User specified preparations and desideratas for an edge split before changing the
     * connectivity
     * @param the edge Tuple to be split
     * @return true if the preparation succeed
     */
    virtual bool split_edge_before(const Tuple& t) { return true; } // check edge condition
    // This function computes the attributes for the added simplices
    // if it returns false then the operation is undone
    /**
     * @brief  This function computes the attributes for the added simplices. User specified
     * modifications and desideratas for after an edge split
     * @param the edge Tuple to be split
     * @return true if the modification succeed
     */
    virtual bool split_edge_after(const Tuple& t) { return true; } // check tet condition

    //// Collapse the edge in the tuple
    // Checks if the collapse should be performed or not (user controlled)
    /**
     * @brief User specified preparations and desideratas for an edge collapse before changing the
     * connectivity
     *
     * @param t edge Tuple to be collapsed
     * @return true is the preparation succeed
     */
    virtual bool collapse_edge_before(const Tuple& t) { return true; }
    // If it returns false then the operation is undone (the tuple indexes a vertex and tet that
    // survived)
    /**
     * @brief  User specified modifications and desideratas for after an edge collapse
     *
     * @param t edge Tuple that's collapsed
     * @return true if the modification succeed
     */
    virtual bool collapse_edge_after(const Tuple& t) { return true; }
    /**
     * @brief User specified preparations and desideratas for an 4-4 edge swap before changing the
     * connectivity
     *
     * @param t edge Tuple to be swaped
     * @return true if the preparation succeed
     */
    virtual bool swap_edge_44_before(const Tuple& t) { return true; }
    /**
     * @brief User specified energy to decide which of the 4 possible orientations should be
     * chosen.
     *
     * Accepts the last shown orientation if not overridden.
     *
     * @param tets New tets after performing a 4-4 swap.
     * @param op_case The operation case, where 0 are the tets before swap.
     * @return energy The swap giving the tets with the lowest energy are chosen.
     */
    virtual double swap_edge_44_energy(
        const std::vector<std::array<size_t, 4>>& tets,
        const int op_case)
    {
        return -op_case;
    }
    /**
     * @brief User specified modifications and desideratas for after a 4-4 edge swap
     *
     * @param t edge Tuple that's swaped
     * @return true if the modification succeed
     */
    virtual bool swap_edge_44_after(const Tuple& t) { return true; }
    /**
     * @brief User specified preparations and desideratas for a 5-6 edge swap before changing the
     * connectivity
     *
     * @param t edge Tuple to be swaped
     * @return true if the preparation succeed
     */
    virtual bool swap_edge_56_before(const Tuple& t) { return true; }
    /**
     * @brief User specified energy to decide which of the 5 possible orientations should be
     * chosen.
     *
     * Accepts the last shown orientation if not overridden.
     *
     * @param tets New tets after performing a 5-6 swap.
     * @param op_case The operation case, where 0 are the tets before swap.
     * @return energy The swap giving the tets with the lowest energy are chosen.
     */
    virtual double swap_edge_56_energy(
        const std::vector<std::array<size_t, 4>>& tets,
        const int op_case)
    {
        return -op_case;
    }
    /**
     * @brief User specified modifications and desideratas for after a 5-6 edge swap
     *
     * @param t edge Tuple that's swaped
     * @return true if the modification succeed
     */
    virtual bool swap_edge_56_after(const Tuple& t) { return true; }
    /**
     * @brief User specified preparations and desideratas for an 3-2 edge swap before changing the
     * conenctivity
     *
     * @param t edge Tuple to be swaped
     * @return true if the preparation succeed
     */
    virtual bool swap_edge_before(const Tuple& t) { return true; }
    /**
     * @brief User specified modifications and desideratas for after a 3-2 edge swap
     *
     * @param t edge Tuple that's swaped
     * @return true if the modification succeed
     */
    virtual bool swap_edge_after(const Tuple& t) { return true; }
    /**
     * @brief User specified preparations and desideratas for an 2-3 face swap befroe changing the
     * geometry
     *
     * @param t edge Tuple to be swaped
     * @return true if the preparation succeed
     */
    virtual bool swap_face_before(const Tuple& t) { return true; }
    /**
     * @brief User specified modifications and desideratas for after a 2-3 face swap
     *
     * @param t edge Tuple that's swaped
     * @return true if the modification succeed
     */
    virtual bool swap_face_after(const Tuple& t) { return true; }
    /**
     * @brief  User specified preparations and desideratas for smoothing a vertex
     *
     * @param t Tuple refering to a vertex Tuple
     * @return true if the preparation succeed
     */
    virtual bool smooth_before(const Tuple& t) { return true; }
    /**
     * @brief  User specified modifications and desideratas for after smoothing a vertex
     *
     * @param t Tuple refering to a vertex
     * @return true if the preparation succeed
     */
    virtual bool smooth_after(const Tuple& t) { return true; }

    /**
     * @brief User specified preparations and desideratas for a face split before changing the
     * connectivity.
     * @param t The face tuple to be split.
     * @return true if the preparation succeed.
     */
    virtual bool split_face_before(const Tuple& t) { return true; }

    /**
     * @brief Compute the attributes for the added simplices.
     *
     * User specified modifications and desideratas for after a face split
     * @param t The face tuple to be split.
     * @return true if the modification succeed
     */
    virtual bool split_face_after(const Tuple& t) { return true; }

    /**
     * @brief User specified preparations and desideratas for a tet split before changing the
     * connectivity.
     * @param t The tet tuple to be split.
     * @return true if the preparation succeed.
     */
    virtual bool split_tet_before(const Tuple& t) { return true; }

    /**
     * @brief Compute the attributes for the added simplices.
     *
     * User specified modifications and desideratas for after a tet split
     * @param t The tet tuple to be split.
     * @return true if the modification succeed
     */
    virtual bool split_tet_after(const Tuple& t) { return true; }

    // virtual void resize_vertex_mutex(size_t v) {}

public:
    /**
     * @brief get a Tuple from global tetra index and __local__ edge index (from 0-5).
     *
     * @param tid Global tetra index
     * @param local_eid local edge index
     */
    Tuple tuple_from_edge(size_t tid, int local_eid) const;
    /**
     * @brief get a Tuple from global vids of the 2 end of an edge
     *
     * @param vids an array of the 2 vertex id of the edge
     */
    Tuple tuple_from_edge(const std::array<size_t, 2>& vids) const;

    /**
     * @brief get a Tuple from global tetra index and __local__ face index (from 0-3).
     *
     * @param tid Global tetra index
     * @param local_fid local face index
     */
    Tuple tuple_from_face(size_t tid, int local_fid) const;

    /**
     * @brief get a Tuple and the global face index from global vertex index of the face.
     *
     * @param vids Global vertex index of the face
     */
    std::tuple<Tuple, size_t> tuple_from_face(const std::array<size_t, 3>& vids) const;

    /**
     * @brief get a Tuple from global vertex index
     *
     * @param vid Global vertex index
     */
    Tuple tuple_from_vertex(size_t vid) const;

    /**
     * @brief get a Tuple from global tetra index
     *
     * @param tid Global tetra index
     */
    Tuple tuple_from_tet(size_t tid) const;

    /**
     * @brief Get a Tuple from global vertex IDs.
     *
     */
    Tuple tuple_from_vids(size_t vid0, size_t vid1, size_t vid2, size_t vid3) const;

    simplex::Tet simplex_from_tet(const Tuple& t) const;

    /**
     * @brief wrapper function from Tuple::switch_vertex
     */
    Tuple switch_vertex(const Tuple& t) const
    {
        auto loc = t.switch_vertex(*this);
        check_tuple_validity(loc);
        return loc;
    }
    /**
     * @brief wrapper function from Tuple::switch_edge
     */
    Tuple switch_edge(const Tuple& t) const
    {
        auto loc = t.switch_edge(*this);
        check_tuple_validity(loc);
        return loc;
    }
    /**
     * @brief wrapper function from Tuple::switch_face
     */
    Tuple switch_face(const Tuple& t) const
    {
        auto loc = t.switch_face(*this);
        check_tuple_validity(loc);
        return loc;
    }
    /**
     * @brief wrapper function from Tuple::switch_tetrahedron
     */
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
     * @brief Get the one ring tids for vertex
     *
     * @param t a Tuple that refers to a vertex
     * @return std::vector<size_t> a vector of vids
     */
    std::vector<size_t> get_one_ring_tids_for_vertex(const Tuple& t) const;
    std::vector<size_t> get_one_ring_tids_for_vertex(const size_t vid) const;

    /**
     * @brief Get the one ring vertices for a vertex
     *
     * @param t tuple pointing to a vertex
     * @return a vector of Tupels that refers to the one-ring vertices
     */
    std::vector<Tuple> get_one_ring_vertices_for_vertex(const Tuple& t) const;
    /**
     * @brief Get the one ring vids for vertex
     *
     * @param vid size_t type vertex id
     * @param[output] cache stores a verctor of vids with duplicate
     * @return std::vector<size_t> vectotr of one-ring vids
     */
    std::vector<size_t> get_one_ring_vids_for_vertex(size_t vid, std::vector<size_t>& cache);
    /**
     * @brief Get the one ring vids for vertex
     *
     * @param vid size_t type vertex id
     * @return std::vector<size_t> a vecotr of unique one-ring vids
     */
    std::vector<size_t> get_one_ring_vids_for_vertex(size_t vid) const;
    /**
     * @brief Duplicate of the function TetMesh::get_one_ring_vids_for_vertex
     *
     */
    std::vector<size_t> get_one_ring_vids_for_vertex_adj(size_t vid) const;
    /**
     * @brief Duplicate of the function TetMesh::get_one_ring_vids_for_vertex
     *
     */
    std::vector<size_t> get_one_ring_vids_for_vertex_adj(size_t vid, std::vector<size_t>& cache);

    /**
     * @brief Get the incident tets for edge
     *
     * @param t tuple pointing to an edge
     * @return incident tets
     */
    std::vector<Tuple> get_incident_tets_for_edge(const Tuple& t) const;
    std::vector<Tuple> get_incident_tets_for_edge(const size_t vid0, const size_t vid1) const;

    std::vector<size_t> get_incident_tids_for_edge(const Tuple& t) const;
    std::vector<size_t> get_incident_tids_for_edge(const size_t vid0, const size_t vid1) const;

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
    /**
     * Positively oriented 4 vertices ids in a tetra.
     * @return std::array<size_t, 4> of the vertex ids.
     */
    std::array<size_t, 4> oriented_tet_vids(const Tuple& t) const;
    std::array<size_t, 4> oriented_tet_vids(const size_t tid) const;
    /**
     * @brief Get the 3 vertices of a face represented by Tuple
     *
     * @param t
     * @return std::array<Tuple, 3> an array of 3 Tuple points to the 3 vertices of a face
     */
    std::array<Tuple, 3> get_face_vertices(const Tuple& t) const;
    std::array<size_t, 3> get_face_vids(const Tuple& t) const;

    /**
     * @brief get the 6 edges of a tet represented by Tuples
     *
     * @param t
     * @return std::array<Tuple, 6> an array of 6 Tuples pointing to the 6 edges of a tet that share
     * the same tid
     */
    std::array<Tuple, 6> tet_edges(const Tuple& t) const;
    /**
     * wrapper function for Tuple::check_validity
     */
    void check_tuple_validity(const Tuple& t) const { t.check_validity(*this); }
    /**
     * @brief checks the validity of the connectivity of the mesh. Including the validity of each
     * Tuple
     *
     * @return true if the mesh is valid
     */
    bool check_mesh_connectivity_validity() const;
    /**
     * @brief remove the tetrahedrons in the mesh that have given tet ids
     *
     * @param tids
     */
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
    bool m_collapse_check_link_condition = true; // classical link condition
    bool m_collapse_check_topology = false; // sanity check
    bool m_collapse_check_manifold = true; // manifoldness check after collapse

private:
    std::map<size_t, VertexConnectivity> operation_update_connectivity_impl(
        std::vector<size_t>& affected_tid,
        const std::vector<std::array<size_t, 4>>& new_tet_conn);
    void operation_failure_rollback_imp(
        std::map<size_t, VertexConnectivity>& rollback_vert_conn,
        const std::vector<size_t>& affected,
        const std::vector<size_t>& new_tet_id,
        const std::vector<TetrahedronConnectivity>& old_tets);
    std::map<size_t, VertexConnectivity> operation_update_connectivity_impl(
        const std::vector<size_t>& remove_id,
        const std::vector<std::array<size_t, 4>>& new_tet_conn,
        std::vector<size_t>& allocate_id);
    static std::vector<TetrahedronConnectivity> record_old_tet_connectivity(
        const TetMesh::vector<TetrahedronConnectivity>& conn,
        const std::vector<size_t>& tets)
    {
        std::vector<TetrahedronConnectivity> tet_conn;
        for (size_t i : tets) {
            tet_conn.push_back(conn[i]);
        }
        return tet_conn;
    }

public:
    void start_protect_attributes()
    {
        if (p_vertex_attrs) {
            p_vertex_attrs->begin_protect();
        }
        if (p_edge_attrs) {
            p_edge_attrs->begin_protect();
        }
        if (p_face_attrs) {
            p_face_attrs->begin_protect();
        }
        if (p_tet_attrs) {
            p_tet_attrs->begin_protect();
        }
    }

    void release_protect_attributes()
    {
        if (p_vertex_attrs) {
            p_vertex_attrs->end_protect();
        }
        if (p_edge_attrs) {
            p_edge_attrs->end_protect();
        }
        if (p_face_attrs) {
            p_face_attrs->end_protect();
        }
        if (p_tet_attrs) {
            p_tet_attrs->end_protect();
        }
    }

    void rollback_protected_attributes()
    {
        if (p_vertex_attrs) {
            p_vertex_attrs->rollback();
        }
        if (p_edge_attrs) {
            p_edge_attrs->rollback();
        }
        if (p_face_attrs) {
            p_face_attrs->rollback();
        }
        if (p_tet_attrs) {
            p_tet_attrs->rollback();
        }
    }

public:
    class VertexMutex
    {
        tbb::spin_mutex mutex;
        int owner = std::numeric_limits<int>::max();

    public:
        bool trylock() { return mutex.try_lock(); }

        void unlock()
        {
            mutex.unlock();
            reset_owner();
        }

        int get_owner() { return owner; }

        void set_owner(int n) { owner = n; }

        void reset_owner() { owner = INT_MAX; }
    };

private:
    tbb::concurrent_vector<VertexMutex> m_vertex_mutex;

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

protected:
    void resize_vertex_mutex(size_t v) { m_vertex_mutex.grow_to_at_least(v); }

public:
    tbb::enumerable_thread_specific<std::vector<size_t>> mutex_release_stack;
    tbb::enumerable_thread_specific<std::vector<size_t>> get_one_ring_cache;

    // void init(size_t n_vertices, const std::vector<std::array<size_t, 4>>& tets);
    int release_vertex_mutex_in_stack();

    // helpers
    /**
     * @brief try lock the two-ring neighboring traingles' incident vertices
     *
     * @param v Tuple refers to the vertex
     * @param threadid
     * @return true if all locked successfully
     */
    bool try_set_vertex_mutex_two_ring(const Tuple& v, int threadid);
    /**
     * @brief try lock the two-ring neighboring traingles' incident vertices using vids
     *
     * @param v Tuple refers to the vertex
     * @param threadid
     * @return true if all locked successfully
     */
    bool try_set_vertex_mutex_two_ring_vid(const Tuple& v, int threadid);
    /**
     * @brief a duplicate of ConcurrentTetMesh::try_set_vertex_mutex_two_ring_vid that gets vids
     using the vid of the input Tuple
     *
     * @param v Tuple refers to the vertex
     * @param threadid
     * @return true if all locked successfully
     */
    bool try_set_vertex_mutex_two_ring_vid(size_t v, int threadid);

    // can be called
    /**
     * @brief try lock the two-ring neighboring triangles' incident vertices for the two ends of an
     * edge
     *
     * @param e Tuple refers to the edge
     * @param threadid
     * @return true if all locked successfully
     */
    bool try_set_edge_mutex_two_ring(const Tuple& e, int threadid = 0);
    /**
     * @brief try lock the two-ring neighboring triangles' incident vertices for the 3 vertices of a
     * face
     *
     * @param f Tuple refers to the face
     * @param threadid
     * @return true if all locked successfully
     */
    bool try_set_face_mutex_two_ring(const Tuple& f, int threadid = 0);
    /**
     * @brief locking the two-ring neighboring triangles' incident vertices given the 3 vertex
     * Tuples of the face
     *
     * @param v1 a Tuple refering to one vertex of the face
     * @param v2 a Tuple refering to one vertex of the face
     * @param v3 a Tuple refering to one vertex of the face
     * @param threadid
     * @return true if all locked successfully
     */
    bool try_set_face_mutex_two_ring(
        const Tuple& v1,
        const Tuple& v2,
        const Tuple& v3,
        int threadid = 0);
    /**
     * @brief a duplicate of ConcurrentTetMesh::try_set_face_mutex_two_ring usign the vids of the 3
     * vertices of a face
     *
     * @param v1 the vid of one vertex of the face
     * @param v2 the vid of one vertex of the face
     * @param v3 the vid of one vertex of the face
     * @param threadid
     * @return true if all locked successfully
     */
    bool try_set_face_mutex_two_ring(size_t v1, size_t v2, size_t v3, int threadid = 0);
    /**
     * @brief try lock the one-ring neighboring traingles' incident vertices
     *
     * @param v Tuple refers to the vertex
     * @param threadid
     * @return true if all locked successfully
     */
    bool try_set_vertex_mutex_one_ring(const Tuple& v, int threadid = 0);

public:
    /**
     * @brief perform the given function for each edge
     *
     */
    void for_each_edge(const std::function<void(const TetMesh::Tuple&)>&);
    /**
     * @brief perform the given function for each vertex
     *
     */
    void for_each_vertex(const std::function<void(const TetMesh::Tuple&)>&);
    /**
     * @brief perform the given function for each tet
     *
     */
    void for_each_tetra(const std::function<void(const TetMesh::Tuple&)>&);
    int NUM_THREADS = 0;
};


} // namespace wmtk
