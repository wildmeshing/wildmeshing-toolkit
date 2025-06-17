#pragma once

#include <wmtk/utils/VectorUtils.h>
#include <wmtk/AttributeCollection.hpp>
#include <wmtk/utils/Logger.hpp>

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <tbb/concurrent_vector.h>
#include <tbb/spin_mutex.h>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <cassert>
#include <map>
#include <memory>
#include <optional>
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
        size_t m_hash = -1;

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
         * Switch operation for the adjacent triangle
         *
         * @param m Mesh
         * @return Tuple for the edge-adjacent triangle, sharing same edge, and vertex.
         * @note nullopt if the Tuple of the switch goes off the boundary.
         */
        std::optional<Tuple> switch_face(const TriMesh& m) const;

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
        inline int find(int v_id) const
        {
            for (int j = 0; j < 3; j++) {
                if (v_id == m_indices[j]) return j;
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
    void create_mesh(size_t n_vertices, const std::vector<std::array<size_t, 3>>& tris);

    /**
     * @brief Generate the connectivity of the mesh from an IGL-style F matrix.
     *
     * @param #F by 3 list of vertex indices.
     */
    void create_mesh(const Eigen::Matrix<int64_t, Eigen::Dynamic, 3>& F);

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
     * Generate a vector of Tuples from global face index
     * @note Local vid is the first of the m_idices
     * Local eid assigned counter clockwise as in the ilustrated example
     * @return vector of Tuples refering to each face
     */
    std::vector<Tuple> get_faces() const;

    /**
     * Generate a vector of Tuples for each edge
     * @note ensures the fid assigned is the smallest between faces adjacent to the
     * edge
     * @return vector of Tuples refering to unique edges
     */
    std::vector<Tuple> get_edges() const;

    /**
     * Generate a tuple using local vid and global fid
     * @param vid1, vid2 are local vids
     * @param fid globale fid for the triangle
     * @note tuple refers to vid1
     * @return vector of Tuples
     */
    Tuple init_from_edge(size_t vid1, size_t vid2, size_t fid) const;

    template <typename T>
    using vector = tbb::concurrent_vector<T>;

public:
    AbstractAttributeContainer* p_vertex_attrs = nullptr;
    AbstractAttributeContainer* p_edge_attrs = nullptr;
    AbstractAttributeContainer* p_face_attrs = nullptr;

private:
    vector<VertexConnectivity> m_vertex_connectivity;
    vector<TriangleConnectivity> m_tri_connectivity;
    std::atomic_long current_vert_size;
    std::atomic_long current_tri_size;
    tbb::spin_mutex vertex_connectivity_lock;
    tbb::spin_mutex tri_connectivity_lock;
    bool vertex_connectivity_synchronizing_flag = false;
    bool tri_connectivity_synchronizing_flag = false;
    int MAX_THREADS = 128;
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

protected:
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
        if (check_link_condition(t)) return true;
        return false;
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

public:
    /**
     * @brief get the current largest global fid
     *
     * @return size_t
     */
    size_t tri_capacity() const { return current_tri_size; }
    /**
     * @brief get the current largest global vid
     *
     * @return size_t
     */
    size_t vert_capacity() const { return current_vert_size; }
    /**
     * @brief removing the elements that are removed
     *
     * @param bnd_output when turn on will write the boundary vertices to "bdn_table.dmat"
     */
    void consolidate_mesh();
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
     * @brief verify the connectivity validity of the mesh
     * @note a valid mesh can have triangles that are is_removed == true
     */
    bool check_mesh_connectivity_validity() const;
    /**
     * @brief verify the edge manifoldness of the mesh
     */
    bool check_edge_manifold() const;

    /**
     * @brief check if edge that's represented by a Tuple is at the boundary of the mesh
     *
     * @param t Tuple refering to an edge
     */
    bool is_boundary_edge(const TriMesh::Tuple& t) const
    {
        if (!t.switch_face(*this).has_value()) return true;
        return false;
    }

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
    bool collapse_edge(const Tuple& t, std::vector<Tuple>& new_t);

    /**
     * Swap an edge
     *
     * @param t Input Tuple for the edge to be swaped.
     * @param[out] new_edges a vector of Tuples refering to the triangles incident to the new edge
     * introduced
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
     * @brief Get the one ring tris for a vertex
     *
     * @param t tuple pointing to a vertex
     * @return a vector of Tuples refering to one-ring tris
     */
    std::vector<Tuple> get_one_ring_tris_for_vertex(const Tuple& t) const;
    /**
     * @brief Get the vids of the incident one ring tris for a vertex
     *
     * @param t tuple pointing to a vertex
     * @return a vector of vids that can have duplicates
     */
    std::vector<size_t> get_one_ring_vids_for_vertex_duplicate(const size_t& t) const;

    /**
     * @brief Get the one ring edges for a vertex, edges are the incident edges
     *
     * @param t tuple pointing to a vertex
     * @return one-ring
     */
    std::vector<Tuple> get_one_ring_edges_for_vertex(const Tuple& t) const;

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

    /**
     * Generate a face Tuple using global fid
     * @param fid global fid for the triangle
     * @note Use the local vid of the first vertex among the incident vertices in the connectivity
     * of the triangle
     * @return a face Tuple
     */
    Tuple tuple_from_tri(size_t fid) const
    {
        if (fid >= m_tri_connectivity.size() || m_tri_connectivity[fid].m_is_removed)
            return Tuple();
        auto vid = m_tri_connectivity[fid][0];
        return Tuple(vid, 1, fid, *this);
    }
    /**
     * Generate avertex Tuple using local vid and global fid
     * @param vid globale vid for the triangle
     * @note tuple refers to vid
     */
    Tuple tuple_from_vertex(size_t vid) const
    {
        auto fid = m_vertex_connectivity[vid][0];
        auto eid = m_tri_connectivity[fid].find(vid);
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

private:
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
    class VertexMutex
    {
        tbb::spin_mutex mutex;
        int owner = std::numeric_limits<int>::max();

    public:
        bool trylock() { return mutex.try_lock(); }

        void unlock()
        {
            reset_owner();
            mutex.unlock();
        }

        int get_owner() { return owner; }

        void set_owner(int n) { owner = n; }

        void reset_owner() { owner = std::numeric_limits<int>::max(); }
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
    void resize_mutex(size_t v) { m_vertex_mutex.grow_to_at_least(v); }

public:
    tbb::enumerable_thread_specific<std::vector<size_t>> mutex_release_stack;

    int release_vertex_mutex_in_stack();
    /**
     * @brief try lock the two-ring neighboring traingles' incident vertices
     *
     * @param v Tuple refers to the vertex
     * @param threadid
     * @return true if all locked successfully
     */
    bool try_set_vertex_mutex_two_ring(const Tuple& v, int threadid);
    /**
     * @brief try lock the two-ring neighboring triangles' incident vertices for the two ends of an
     * edge
     *
     * @param e Tuple refers to the edge
     * @param threadid
     * @return true if all locked successfully
     */
    bool try_set_edge_mutex_two_ring(const Tuple& e, int threadid);
    /**
     * @brief get the lock for one ring neighboring triangles' incident vertices
     *
     * @param v
     * @param threadid
     * @return true if all succeed
     */
    bool try_set_vertex_mutex_one_ring(const Tuple& v, int threadid);

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
};

} // namespace wmtk
