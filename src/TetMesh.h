//
// Created by Yixin Hu on 10/12/21.
//

#pragma once

#include <wmtk/VectorUtils.h>
#include <wmtk/Logger.hpp>

#ifdef WILDMESHING_TOOLKIT_WITH_TBB
#include <tbb/concurrent_vector.h>
#endif

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
    // YH: should be visible for all connectivity classes!
    static constexpr std::array<std::array<int, 2>, 6> m_local_edges = {
        {{{0, 1}}, {{1, 2}}, {{0, 2}}, {{0, 3}}, {{1, 3}}, {{2, 3}}}}; // local edges within a
    // tet
    static constexpr std::array<int, 6> m_map_vertex2edge = {{0, 0, 1, 3}};
    static constexpr std::array<int, 6> m_map_edge2face = {{0, 0, 0, 1, 2, 1}};
    static constexpr std::array<std::array<int, 3>, 6> m_local_faces = {
        {{{0, 1, 2}}, {{0, 2, 3}}, {{0, 1, 3}}, {{1, 2, 3}}}}; // sorted local vids
    static constexpr std::array<std::array<int, 3>, 6> m_local_edges_in_a_face = {
        {{{0, 1, 2}}, {{2, 5, 3}}, {{3, 4, 0}}, {{5, 1, 4}}}};

public:
    // Cell Tuple Navigator
    class Tuple
    {
        size_t m_vid;
        size_t m_eid;
        size_t m_fid;
        size_t m_tid;

        int m_timestamp = 0;

    public:
        Tuple() {}

        /**
         * Construct a new Tuple object with global vertex/tetra index and local edge/face index
         *
         * @param vid vertex id (global)
         * @param eid edge id (local)
         * @param fid face id (local)
         * @param tid tetra id (global)
         */
        Tuple(size_t vid, size_t eid, size_t fid, size_t tid)
            : m_vid(vid)
            , m_eid(eid)
            , m_fid(fid)
            , m_tid(tid)
        {} // DP: the counter should be initialized here?

        /**
         * Generate a Tuple from global tetra index and __local__ edge index (from 0-5).
         *
         * @param m TetMesh where the current Tuple belongs.
         * @param tid Global tetra index
         * @param local_eid local edge index
         * @return Tuple
         */
        static Tuple init_from_edge(const TetMesh& m, int tid, int local_eid);
        static Tuple init_from_face(const TetMesh& m, int tid, int local_fid);

        /**
         * TODO
         *
         * @param m
         * @param vid
         * @return Tuple
         */
        static Tuple init_from_vertex(const TetMesh& m, int vid);
        static Tuple init_from_tet(const TetMesh& m, int tid);

        /**
         * Check if the current tuple is already invalid (removed during editing).
         *
         * @param m TetMesh where the tuple belongs.
         * @return if not removed
         */
        bool is_valid(const TetMesh& m) const;
        bool is_boundary_edge(const TetMesh& m) const;
        bool is_boundary_face(const TetMesh& m) const;


        void update_version_number(const TetMesh& m);
        int get_version_number();
        bool is_version_number_valid(const TetMesh& m) const;

        void print_info() const;
        void print_info(const TetMesh& m) const;

        size_t vid() const;

        /**
         * returns a global unique edge id
         *
         * @return size_t
         * @note The global id may not be consecutive. The edges are undirected and different tetra
         * share the same edge.
         */
        size_t eid(const TetMesh& m) const;

        /**
         * returns a global unique face id
         *
         * @return size_t
         * @note The global id may not be consecutive. The face are undirected.
         */
        size_t fid(const TetMesh& m) const;

        /**
         * returns global tetra id.
         *
         * @return size_t
         */
        size_t tid() const;

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

        std::vector<Tuple> get_conn_tets(const TetMesh& m) const;

        /**
         * Positively oriented 4 vertices (represented by Tuples) in a tetra.
         * @return std::array<Tuple, 4> each tuple owns a different vertex.
         */
        std::array<Tuple, 4> oriented_tet_vertices(const TetMesh& m) const;


        ////testing code
        void check_validity(const TetMesh& m) const;
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
            assert(index >= 0 && index < m_conn_tets.size());
            return m_conn_tets[index];
        }

        size_t operator[](const size_t index) const
        {
            assert(index >= 0 && index < m_conn_tets.size());
            return m_conn_tets[index];
        }
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

        int timestamp = 0;

        void set_version_number(int version) { timestamp = version; }

        int get_version_number() { return timestamp; }

        size_t& operator[](size_t index)
        {
            assert(index >= 0 && index < 4);
            return m_indices[index];
        }

        size_t operator[](size_t index) const
        {
            assert(index >= 0 && index < 4);
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
    };

    TetMesh() {}
    virtual ~TetMesh() {}

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
    bool swap_edge(const Tuple& t);
    bool swap_face(const Tuple& t);
    bool smooth_vertex(const Tuple& t);

    void
    compact(); // cleans up the deleted vertices or tetrahedra, and fixes the corresponding indices

    void reset_timestamp()
    {
        m_timestamp = 0;
        for (auto& t : m_tet_connectivity) t.timestamp = 0;
    }

    /**
     * Get all unique undirected edges in the mesh.
     *
     * @return std::vector<Tuple> each Tuple owns a distinct edge.
     */
    std::vector<Tuple> get_edges() const;
    std::vector<Tuple> get_vertices() const;

    /**
     * Number of tetra in the mesh
     */
    size_t n_tets() const { return m_tet_connectivity.size(); }

private:
    // Stores the connectivity of the mesh
#ifdef WILDMESHING_TOOLKIT_WITH_TBB
    tbb::concurrent_vector<VertexConnectivity> m_vertex_connectivity;
    tbb::concurrent_vector<TetrahedronConnectivity> m_tet_connectivity;
#else
    std::vector<VertexConnectivity> m_vertex_connectivity;
    std::vector<TetrahedronConnectivity> m_tet_connectivity;
#endif

    int m_t_empty_slot = 0;
    int m_v_empty_slot = 0;
    int find_next_empty_slot_t();
    int find_next_empty_slot_v();

    int m_timestamp = 0;

protected:
    //// Split the edge in the tuple
    // Checks if the split should be performed or not (user controlled)
    virtual bool split_before(const Tuple& t) { return true; } // check edge condition
    // This function computes the attributes for the added simplices
    // if it returns false then the operation is undone
    virtual bool split_after(const std::vector<Tuple>& locs) { return true; } // check tet condition

    //// Collapse the edge in the tuple
    // Checks if the collapse should be performed or not (user controlled)
    virtual bool collapse_before(const Tuple& t) { return true; }
    // If it returns false then the operation is undone (the tuple indexes a vertex and tet that
    // survived)
    virtual bool collapse_after(const std::vector<Tuple>& locs) { return true; }
    
    virtual bool swap_edge_before(const Tuple& t) { return true; }
    virtual bool swap_edge_after(const Tuple& t) { return true; }
    virtual bool swap_face_before(const Tuple& t) { return true; }
    virtual bool swap_face_after(const Tuple& t) { return true; }
    virtual bool smooth_before(const Tuple &t) { return true; } 
    virtual bool smooth_after(const Tuple &t) { return true; } 
    // todo: quality, inversion, envelope: change v1 pos before this, only need to change partial
    // attributes

    //        //// Swap the edge in the tuple
    //        // Checks if the swapping should be performed or not (user controlled)
    //        virtual bool swapping_before(const Tuple &t) { return true; }
    //        // If it returns false then the operation is undone (the tuple indexes TODO)
    //        virtual bool swapping_after(const Tuple &t) { return true; }
    //        //quality, inversion
    //
    // Invariants that are called on all the new or modified elements after an operation is
    // performed
    virtual bool vertex_invariant(const Tuple& t) { return true; }
    virtual bool edge_invariant(const Tuple& t) { return true; }
    virtual bool face_invariant(const Tuple& t) { return true; }
    virtual bool tetrahedron_invariant(const Tuple& t) { return true; }

    virtual void resize_attributes(size_t v, size_t e, size_t f, size_t t) {}

public:
    /**
     * Thin wrapper for tuple functions
     */
    Tuple tuple_from_edge(int tid, int local_eid) const
    {
        auto loc = Tuple::init_from_edge(*this, tid, local_eid);
        check_tuple_validity(loc);
        return loc;
    }
    Tuple tuple_from_face(int tid, int local_fid) const
    {
        auto loc = Tuple::init_from_edge(*this, tid, local_fid);
        check_tuple_validity(loc);
        return loc;
    }
    Tuple tuple_from_vertex(int vid) const
    {
        auto loc = Tuple::init_from_vertex(*this, vid);
        check_tuple_validity(loc);
        return loc;
    }
    Tuple tuple_from_tet(int tid) const
    {
        auto loc = Tuple::init_from_tet(*this, tid);
        check_tuple_validity(loc);
        return loc;
    }

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

    std::vector<Tuple> get_conn_tets(const Tuple& t) const
    {
        auto locs = t.get_conn_tets(*this);
        for (const auto& loc : locs) check_tuple_validity(loc);
        return locs;
    }

    std::array<Tuple, 4> oriented_tet_vertices(const Tuple& t) const
    {
        auto locs = t.oriented_tet_vertices(*this);
        for (const auto& loc : locs) check_tuple_validity(loc);
        return locs;
    }

    void check_tuple_validity(const Tuple& t) const { t.check_validity(*this); }

    bool check_mesh_connectivity_validity() const;
};

} // namespace wmtk
