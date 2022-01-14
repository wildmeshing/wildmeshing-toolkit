#pragma once

#include <wmtk/utils/VectorUtils.h>
#include <wmtk/utils/Logger.hpp>

#include <tbb/concurrent_vector.h>

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

    TetMesh() {}
    virtual ~TetMesh() {}

    size_t vert_capacity() const { return m_vertex_connectivity.size(); };
    size_t tet_capacity() const { return m_tet_connectivity.size(); };
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
     * @param[out] new_edges a vector of Tuples for all the edges from the newly introduced tetra.
     * @return if split succeed
     */
    bool split_edge(const Tuple& t, std::vector<Tuple>& new_edges);
    bool collapse_edge(const Tuple& t, std::vector<Tuple>& new_edges);
    bool swap_edge(const Tuple& t, Tuple& new_face);
    bool swap_face(const Tuple& t, Tuple& new_edge);
    bool smooth_vertex(const Tuple& t);

    void subdivide_tets(const std::vector<size_t> t_ids, std::map<std::array<size_t, 2>, size_t>& map_edge2vid);

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

public:
    template <typename T>
    using vector = tbb::concurrent_vector<T>;

private:
    // Stores the connectivity of the mesh
    vector<VertexConnectivity> m_vertex_connectivity;
    vector<TetrahedronConnectivity> m_tet_connectivity;
    int m_t_empty_slot = 0;
    int m_v_empty_slot = 0;
    int find_next_empty_slot_t();
    int find_next_empty_slot_v();

    void subdivide_a_tet(size_t t_id, const std::array<int, 6>& new_v_ids, bool is_add_centroid);

protected:
    virtual void insertion_update_surface_tag(size_t t_id, size_t new_t_id,
                                              int config_id, int diag_config_id, int index){}
    virtual void add_tet_centroid(const std::array<size_t, 4>& vids){}

    //// Split the edge in the tuple
    // Checks if the split should be performed or not (user controlled)
    virtual bool split_before(const Tuple& t) { return true; } // check edge condition
    // This function computes the attributes for the added simplices
    // if it returns false then the operation is undone
    virtual bool split_after(const Tuple& t) { return true; } // check tet condition

    //// Collapse the edge in the tuple
    // Checks if the collapse should be performed or not (user controlled)
    virtual bool collapse_before(const Tuple& t) { return true; }
    // If it returns false then the operation is undone (the tuple indexes a vertex and tet that
    // survived)

    virtual bool swap_edge_before(const Tuple& t) { return true; }
    virtual bool swap_edge_after(const Tuple& t) { return true; }
    virtual bool swap_face_before(const Tuple& t) { return true; }
    virtual bool swap_face_after(const Tuple& t) { return true; }

    virtual bool collapse_after(const Tuple& t) { return true; }
    virtual bool smooth_before(const Tuple& t) { return true; }
    virtual bool smooth_after(const Tuple& t) { return true; }

    // Invariants that are called on all the new or modified elements after an operation is
    // performed
    virtual bool vertex_invariant(const Tuple& t) { return true; }
    virtual bool edge_invariant(const Tuple& t) { return true; }
    virtual bool face_invariant(const Tuple& t) { return true; }
    virtual bool tetrahedron_invariant(const Tuple& t) { return true; }

    virtual void resize_attributes(size_t v, size_t e, size_t f, size_t t) {}
    virtual void move_face_attribute(size_t from, size_t to) {}
    virtual void move_edge_attribute(size_t from, size_t to) {}
    virtual void move_tet_attribute(size_t from, size_t to) {}
    virtual void move_vertex_attribute(size_t from, size_t to) {}

public:
    /**
     * @brief get a Tuple from global tetra index and __local__ edge index (from 0-5).
     *
     * @param m TetMesh where the current Tuple belongs.
     * @param tid Global tetra index
     * @param local_eid local edge index
     */
    Tuple tuple_from_edge(int tid, int local_eid) const;

    /**
     * @brief get a Tuple from global tetra index and __local__ face index (from 0-3).
     *
     * @param m TetMesh where the current Tuple belongs.
     * @param tid Global tetra index
     * @param local_fid local face index
     */
    Tuple tuple_from_face(int tid, int local_fid) const;

    /**
     * @brief get a Tuple from global vertex index
     *
     * @param m TetMesh where the current Tuple belongs.
     * @param vid Global vertex index
     */
    Tuple tuple_from_vertex(int vid) const;

    /**
     * @brief get a Tuple from global tetra index
     *
     * @param m TetMesh where the current Tuple belongs.
     * @param tid Global tetra index
     */
    Tuple tuple_from_tet(int tid) const;


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
     * Positively oriented 4 vertices (represented by Tuples) in a tetra.
     * @return std::array<Tuple, 4> each tuple owns a different vertex.
     */
    std::array<Tuple, 4> oriented_tet_vertices(const Tuple& t) const;

    std::array<Tuple, 6> tet_edges(const Tuple& t) const;

    void check_tuple_validity(const Tuple& t) const { t.check_validity(*this); }
    bool check_mesh_connectivity_validity() const;
};

} // namespace wmtk
