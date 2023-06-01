#pragma once

#define USE_OPERATION_LOGGER
#include <wmtk/TriMeshTuple.h>
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

class OperationLogger;
class OperationRecorder;
class TriMeshOperationLogger;
class TriMeshOperationRecorder;
class OperationRecorder;
class TriMeshOperation;
class TriMeshTupleData;
class AttributeCollectionRecorder;
class SingleTupleTriMeshOperation;


class TriMesh
{
public:
    using Tuple = TriMeshTuple;
    friend class TriMeshTuple;
    /**
     * (internal use) Maintains a list of triangles connected to the given vertex, and a flag to
     * mark removal.
     *
     */
    class VertexConnectivity;

    /**
     * (internal use) Maintains a list of vertices of the given triangle
     *
     */
    class TriangleConnectivity;

    friend class TriMeshOperation;
    friend class TriMeshOperationLogger;
    friend class TriMeshOperationRecorder;
    friend class OperationReplayer;
    friend class AttributeCollectionRecorder;

    TriMesh();
    virtual ~TriMesh();


    // Loads the operations for a particular application.
    // mtao's note: i really don't like Mesh having so much per-application logic, we should be
    // loading operation data in the executor instead of the data storage mechanism, but that is a
    // refactor for future me
    virtual std::map<std::string, std::shared_ptr<TriMeshOperation>> get_operations() const;

    /**
     * Copy connectivity from another mesh
     * @param o the other mesh who's connectivity is being copied
     */
    void copy_connectivity(const TriMesh& o);

    /**
     * Generate the connectivity of the mesh
     * @param n_vertices Input number of vertices
     * @param tris triangle connectivity
     */
    void create_mesh(size_t n_vertices, const std::vector<std::array<size_t, 3>>& tris);

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

    // same as init_from_edge but if the edge doesn't exist it returns returns nohting
    std::optional<Tuple> init_from_edge_opt(size_t vid1, size_t vid2, size_t fid) const;

    // generates a tuple from
    std::optional<Tuple> init_from_edge_opt(size_t vid1, size_t vid2) const;

    // TODO: this is potentially very misleading. not to be allowed
    template <typename T>
    using vector = tbb::concurrent_vector<T>;

public:
    AbstractAttributeCollection* p_vertex_attrs = nullptr;
    AbstractAttributeCollection* p_edge_attrs = nullptr;
    AbstractAttributeCollection* p_face_attrs = nullptr;


protected:
    wmtk::AttributeCollection<VertexConnectivity> m_vertex_connectivity;
    wmtk::AttributeCollection<TriangleConnectivity> m_tri_connectivity;


protected:
    std::atomic_long current_vert_size;
    std::atomic_long current_tri_size;
    tbb::spin_mutex vertex_connectivity_lock;
    tbb::spin_mutex tri_connectivity_lock;
    bool vertex_connectivity_synchronizing_flag = false;
    bool tri_connectivity_synchronizing_flag = false;
    int MAX_THREADS = 128;

#if defined(USE_OPERATION_LOGGER)
    tbb::enumerable_thread_specific<std::weak_ptr<TriMeshOperationRecorder>> p_operation_recorder{
        {}};
#endif
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
    // MTAO: TODO: figure out if invariants is a property of the mesh or a property o
    virtual bool invariants(const TriMeshOperation& op);

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
    bool is_boundary_vertex(const TriMesh::Tuple& t) const;


    /**
     * @brief Count the number of the one ring tris for a vertex
     *
     * @param t tuple pointing to a vertex
     * @return one-ring tris number
     */
    size_t get_valence_for_vertex(const Tuple& t) const;

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
    std::vector<size_t> get_one_ring_vids_for_vertex_with_duplicates(const size_t& t) const;
    /**
     * @brief Get the vids of the incident one ring tris for a vertex
     *
     * @param t tuple pointing to a vertex
     * @return a vector of vids that have no duplicates
     */
    std::vector<size_t> get_one_ring_vids_for_vertex(const size_t& t) const;
    /**
     * @brief Get the one ring edges for a vertex, edges are the incident edges
     *
     * @param t
     * @return tuple pointing to the vertex referred to by t from one_ring vertices
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
    Tuple tuple_from_tri(size_t fid) const;
    /**
     * Generate avertex Tuple using local vid and global fid
     * @param vid globale vid for the triangle
     * @note tuple refers to vid
     */
    Tuple tuple_from_vertex(size_t vid) const;
    /**
     * Generate a edge Tuple using global fid and local eid
     * @param fid globale fid for the triangle
     * @param local_eid local eid
     * @return tuple refers to the edge
     */
    Tuple tuple_from_edge(size_t fid, size_t local_eid) const;

    /**
     * Generate a edge Tuple using two vids
     * @param a global vid
     * @param another global vid
     * @return tuple refers to the edge
     */
    std::optional<Tuple> tuple_from_edge_vids_opt(size_t vid1, size_t vid2) const;

    /**
     * Generate the tuples for the tuples at the boundary of a triangle
     * @param triangle for which we are computing the boundary of
     * @return array of tuples storing the three edges
     */
    std::array<Tuple, 3> triangle_boundary_edge_tuples(const Tuple& triangle) const;

    // returns edge tuples that all represent teh same edge, but attached to different triangles
    std::vector<Tuple> tris_bounded_by_edge(const Tuple& edge) const;

private:
    std::vector<size_t> tri_fids_bounded_by_edge(const Tuple& edge) const;
    std::vector<size_t> tri_fids_bounded_by_edge_vids(size_t v0, size_t v1) const;

    // private:
protected:
    /**
     * Generate the vertex connectivity of the mesh using the existing triangle structure
     * @param n_vertices Input number of vertices
     */
    void build_vertex_connectivity(size_t n_vertices);
    /**
     * @brief Start the phase where the attributes that will be modified can be recorded
     *
     */
    using ProtectedAttributeRAII = std::array<AttributeCollectionProtectRAII, 3>;
    void start_protected_attributes();
    ProtectedAttributeRAII start_protected_attributes_raii();
    /**
     * @brief Start caching the connectivity that will be modified
     */
    using ProtectedConnectivityRAII = std::array<AttributeCollectionProtectRAII, 2>;
    void start_protected_connectivity();
    ProtectedConnectivityRAII start_protected_connectivity_raii();

    /**
     * @brief End the modification phase
     *
     */
    std::array<std::optional<size_t>, 3> release_protected_attributes();

    /**
     * @brief End Caching connectivity
     *
     */
    std::optional<size_t> release_protected_connectivity();

    /**
     * @brief rollback the attributes that are modified if any condition failed
     *
     */
    void rollback_protected_attributes();

    /**
     * @brief rollback the connectivity that are modified if any condition failed
     */
    void rollback_protected_connectivity();

    /**
     * @brief rollback both connectivity and attributes
     */
    void rollback_protected();

    // Moved code from concurrent TriMesh

private:
    class VertexMutex;
    tbb::concurrent_vector<VertexMutex> m_vertex_mutex;

    bool try_set_vertex_mutex(const Tuple& v, int threadid);
    bool try_set_vertex_mutex(size_t vid, int threadid);

    void unlock_vertex_mutex(const Tuple& v);
    void unlock_vertex_mutex(size_t vid);

protected:
    void resize_mutex(size_t v);

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

class TriMesh::VertexConnectivity
{
public:
    /**
     * @brief incident triangles of a given vertex
     *
     */
    // TODO: this needs to be private
    std::vector<size_t> m_conn_tris;
    /**
     * @brief is the vertex removed
     *
     */
    bool m_is_removed = true;

    // inline size_t& operator[](const size_t index)
    //{
    //     assert(index < m_conn_tris.size());
    //     return m_conn_tris[index];
    // }

    inline size_t operator[](const size_t index) const
    {
        assert(index < m_conn_tris.size());
        return m_conn_tris[index];
    }

    void insert(const size_t value) { set_insert(m_conn_tris, value); }
    // replace the value stored at an index by another value
    void replace(const size_t index, const size_t value)
    {
        assert(index < m_conn_tris.size());
        auto it = m_conn_tris.begin() + index;
        m_conn_tris.erase(it);
        set_insert(m_conn_tris, value);
    }
    // replace a value with another value
    void replace_value(const size_t index, const size_t value)
    {
        vector_erase(m_conn_tris, index);
        insert(value);
    }
};

/**
 * (internal use) Maintains a list of vertices of the given triangle
 *
 */
class TriMesh::TriangleConnectivity
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
    bool m_is_removed = true;
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
} // namespace wmtk
